/** \brief A faster voxel grid containing ndt cells.
 * \author Liu.yang
 */
#ifndef NDT_CORE_GRID_H
#define NDT_CORE_GRID_H

#include "cell.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point_types.h"

template <typename PointT>
class NdtGrid
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NdtGrid();
    const std::shared_ptr<Cell> getCell(const PointT &point) const;
    const std::shared_ptr<Cell> getCell(const int idx) const;
    const std::vector<std::shared_ptr<Cell>> &getCells() const;
    void setInput(typename pcl::PointCloud<PointT>::ConstPtr cloud);
    void update(typename pcl::PointCloud<PointT>::ConstPtr cloud);
    int getNeighborhood7(const PointT &point, std::vector<int> &neighbors) const;
    void setResolution(const double r);

private:
    void grow(typename pcl::PointCloud<PointT>::ConstPtr input);
    int getFlatIdx(const Eigen::Vector3i &idx3d, const Eigen::AlignedBox3i &cell_box) const;
    int getFlatIdx(const Eigen::Vector3d &p) const;
    bool contains(const Eigen::Vector3i &p) const;
    void reset();

    std::vector<std::shared_ptr<Cell>> cells_;
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> neighborhood_;
    Eigen::AlignedBox3i cell_box_;
    double resolution_;
    int border_; // meters
    const double min_covar_eigvalue_mult_;
    const double min_points_per_voxel_;
};

#endif