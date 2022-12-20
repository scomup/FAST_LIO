#include <set>
#include "ndt_grid.h"

#define BORDER_METER 10

template <typename PointT>
NdtGrid<PointT>::NdtGrid() : min_covar_eigvalue_mult_(0.01),
                             min_points_per_voxel_(6)
{
    neighborhood_.push_back(Eigen::Vector3i(0, 0, 0));
    neighborhood_.push_back(Eigen::Vector3i(1, 0, 0));
    neighborhood_.push_back(Eigen::Vector3i(-1, 0, 0));
    neighborhood_.push_back(Eigen::Vector3i(0, 1, 0));
    neighborhood_.push_back(Eigen::Vector3i(0, -1, 0));
    neighborhood_.push_back(Eigen::Vector3i(0, 0, 1));
    neighborhood_.push_back(Eigen::Vector3i(0, 0, -1));
}

template <typename PointT>
void NdtGrid<PointT>::setResolution(const double r)
{
    resolution_ = r;
    border_ = BORDER_METER / resolution_;
    reset();
}

template <typename PointT>
void NdtGrid<PointT>::reset()
{
    cells_.clear();
    cell_box_.setEmpty();
}

template <typename PointT>
const std::vector<std::shared_ptr<Cell>> &NdtGrid<PointT>::getCells() const
{
    return cells_;
}

template <typename PointT>
void NdtGrid<PointT>::setInput(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    reset();
    update(cloud);
}

template <typename PointT>
void NdtGrid<PointT>::update(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    grow(cloud);
    std::set<int> updated_idxs;
    for (uint i = 0; i < cloud->points.size(); i++)
    {
        Eigen::Vector3d p = cloud->points[i].getVector3fMap().template cast<double>();
        int idx = getFlatIdx(p);
        auto &cell = cells_[idx];
        if (cell == nullptr)
        {
            cell = std::make_shared<Cell>();
        }
        updated_idxs.insert(idx);
        cell->num_ += 1;
        cell->sum_ += p;
        cell->ppt_ += p * p.transpose();
    }

    // Eigen values and vectors calculated to prevent near singular matrices
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Matrix3d eigen_val;
    Eigen::Matrix3d eigen_vec;
    Eigen::Matrix3d cov;
    // Eigen values less than a threshold of max eigen value are inflated to a set fraction of the max eigen value.
    double min_covar_eigvalue;
    for (auto idx : updated_idxs)
    {
        auto &cell = cells_[idx];
        // Normalize mean
        cell->mean_ = cell->sum_ / cell->num_;

        if (cell->num_ < min_points_per_voxel_)
        {
            continue;
        }

        // Single pass covariance calculation
        cov = (cell->ppt_ - 2 * (cell->sum_ * cell->mean_.transpose())) /
                  cell->num_ +
              cell->mean_ * cell->mean_.transpose();
        cov *= (cell->num_ - 1.0) / cell->num_;

        // Normalize Eigen Val such that max no more than 100x min.
        eigensolver.compute(cov);

        eigen_val = eigensolver.eigenvalues().asDiagonal();
        eigen_vec = eigensolver.eigenvectors();

        if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 || eigen_val(2, 2) <= 0)
        {
            cell.reset();
            continue;
        }

        // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]
        min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val(2, 2);
        if (eigen_val(0, 0) < min_covar_eigvalue)
        {
            eigen_val(0, 0) = min_covar_eigvalue;

            if (eigen_val(1, 1) < min_covar_eigvalue)
            {
                eigen_val(1, 1) = min_covar_eigvalue;
            }

            cov = eigen_vec * eigen_val * eigen_vec.inverse();
        }

        cell->icov_ = cov.inverse();
        cell->icovL_ = cell->icov_.llt().matrixL();
        if (cell->icov_.maxCoeff() == std::numeric_limits<float>::infinity() ||
            cell->icov_.minCoeff() == -std::numeric_limits<float>::infinity())
        {
            cell.reset();
        }
    }
}

template <typename PointT>
const std::shared_ptr<Cell> NdtGrid<PointT>::getCell(const int idx) const
{
    return cells_[idx];
}

template <typename PointT>
const std::shared_ptr<Cell> NdtGrid<PointT>::getCell(const PointT &point) const
{
    Eigen::Vector3d p = point.getVector3fMap().template cast<double>();
    int idx = getFlatIdx(p);
    if (idx == -1)
        return nullptr;
    else
        return cells_[idx];
}

template <typename PointT>
int NdtGrid<PointT>::getFlatIdx(const Eigen::Vector3d &p) const
{
    Eigen::Vector3i idx3d = (p / resolution_).array().floor().cast<int>().matrix();
    return getFlatIdx(idx3d, cell_box_);
}

template <typename PointT>
int NdtGrid<PointT>::getFlatIdx(const Eigen::Vector3i &idx3d, const Eigen::AlignedBox3i &cell_box) const
{
    auto cell_size = cell_box.sizes();
    Eigen::Vector3i tmp = idx3d - cell_box.min();
    int idx = tmp.x() + tmp.y() * cell_size.x() + tmp.z() * cell_size.x() * cell_size.y();
    if (contains(idx3d))
        return idx;
    else
        return -1;
}

template <typename PointT>
bool NdtGrid<PointT>::contains(const Eigen::Vector3i &idx3d) const
{
    // cell_box_.contains is no good..
    if ((idx3d.array() >= cell_box_.min().array()).all() &&
        (idx3d.array() < cell_box_.max().array()).all())
        return true;
    else
        return false;
}

template <typename PointT>
int NdtGrid<PointT>::getNeighborhood7(const PointT &point, std::vector<int> &neighbors) const
{
    Eigen::Vector3d p = point.getVector3fMap().template cast<double>();
    Eigen::Vector3i pi = (p / resolution_).array().floor().cast<int>().matrix();
    int num = 0;
    for (auto i : neighborhood_)
    {
        Eigen::Vector3i n = pi + i;
        int idx = getFlatIdx(n, cell_box_);
        if (idx == -1)
            continue;
        auto &cell = cells_[idx];
        if (cell == nullptr)
            continue;
        if (cell->num_ < min_points_per_voxel_)
            continue;
        // unstable code ...
        // double dist = (cell->mean_ - p).norm();
        // if (dist > resolution_)
        //     continue;
        neighbors.push_back(idx);
        num++;
    }
    return num;
}

template <typename PointT>
void NdtGrid<PointT>::grow(typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
    Eigen::AlignedBox3i cloud_box;
    Eigen::AlignedBox3f tmp;
    for (uint i = 0; i < cloud->points.size(); i++)
    {
        Eigen::Vector3f pf = cloud->points[i].getVector3fMap();
        tmp.extend(pf);
    }

    cloud_box.extend((tmp.min() / resolution_).array().floor().cast<int>().matrix());
    cloud_box.extend((tmp.max() / resolution_).array().ceil().cast<int>().matrix());

    if (cell_box_.isEmpty())
    {
        cell_box_.extend((cloud_box.min().array() - border_).matrix());
        cell_box_.extend((cloud_box.max().array() + border_).matrix());

        cells_.clear();
        cells_.resize(cell_box_.volume());
        return;
    }
    // no need grow, if the cell_box contains the cloud_box,
    if (cell_box_.contains(cloud_box))
        return;
    Eigen::AlignedBox3i new_cell_box;
    Eigen::Vector3i new_min = cell_box_.min();
    Eigen::Vector3i new_max = cell_box_.max();

    // Using border area, to avoid frequent memory allocation.
    if (cloud_box.min().x() < cell_box_.min().x())
        new_min.x() = cloud_box.min().x() - border_;
    if (cloud_box.min().y() < cell_box_.min().y())
        new_min.y() = cloud_box.min().y() - border_;
    if (cloud_box.min().z() < cell_box_.min().z())
        new_min.z() = cloud_box.min().z() - border_;

    if (cloud_box.max().x() > cell_box_.max().x())
        new_max.x() = cloud_box.max().x() + border_;
    if (cloud_box.max().y() > cell_box_.max().y())
        new_max.y() = cloud_box.max().y() + border_;
    if (cloud_box.max().z() > cell_box_.max().z())
        new_max.z() = cloud_box.max().z() + border_;

    new_cell_box.extend(new_min);
    new_cell_box.extend(new_max);

    if ((new_cell_box.volume()) > std::numeric_limits<int32_t>::max())
    {
        fprintf(stderr, "Map is too big, or resolution is too small.\n");
        return;
    }

    std::vector<std::shared_ptr<Cell>> new_cells(new_cell_box.volume());

    auto cell_size = cell_box_.sizes();
    for (uint i = 0; i < cells_.size(); i++)
    {
        if (cells_[i] == nullptr)
            continue;
        int z = i / (cell_size.x() * cell_size.y()) + cell_box_.min().z();
        int ixy = i % (cell_size.x() * cell_size.y());
        int y = ixy / cell_size.x() + cell_box_.min().y();
        int x = ixy % cell_size.x() + cell_box_.min().x();
        int new_i = getFlatIdx(Eigen::Vector3i(x, y, z), new_cell_box);
        new_cells[new_i] = cells_[i];
    }
    cells_ = new_cells;
    cell_box_ = new_cell_box;
}

template class NdtGrid<PointXYZIT>;
template class NdtGrid<pcl::PointXYZI>;
template class NdtGrid<pcl::PointXYZ>;

// namespace NdtGrid
