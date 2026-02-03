## Error State Extended Kalman Filter based LiDAR-inertial Odometry

Assuming an IMU is rigidly attached to the LiDAR with a known (or unknown) extrinsic $R_{il}$, $t_{il}$, The state $x$ of IMU can be defined as following:

### State

$$ 
x = [p_{wi}, R_{wi}, R_{il}, t_{il}, v_{wi}, b_{\omega}, b_a, g] \tag{1}
$$

* $p_{wi}$ The imu positon in world frame.
* $R_{wi}$ The imu rotation in world frame.
* $R_{il}$ The rotation from lidar to imu.
* $t_{il}$ The translation from lidar to imu.
* $b_{\omega}$ The gyroscope bias.
* $b_a$ The accelerometer bias.
* $g$ The gravity vector.

We omit the subscript $wi$ for convenience.

$$ 
x = [p, R, R_{il}, t_{il}, v, b_{\omega}, b_a, g] \tag{2}
$$

Because $x$ is a manifold, we can define $\boxplus$ and $\boxminus$ on $x$.

$$
if  R \in  SO3 \\ r \in \mathfrak{so}3 \\ 
R_1 \boxplus R_2 = R_1 R_2 \\
R_1 \boxminus R_2 = R_1^{-1} R_2 \\
R_1 \boxplus r = R_1 \exp{r} \\
R_1 \boxminus  r = \exp{r}^{-1} R_1  
\tag{3}
$$

$$
if a,b \in \mathbb{R}^n \\ 
a \boxplus b = a + b \\
a \boxminus b = a - b
\tag{4}
$$

### State Estimate
According to kinematics, we can calculate the differential of $x$.

$$ 
\dot{x} = [\dot{p}, \dot{R}, \dot{R_{il}}, \dot{t_{il}}, \dot{v}, \dot{b_{\omega}}, \dot{b_a}, \dot{g}] 
\tag{5}
$$

The continuous time differential equation of state is given by:

$$ 
\dot{x} = 
\begin{bmatrix}
 v \\ 
\omega - b_{\omega}-n_{\omega} \\
 0 \\
 0 \\
 R(a-b_a-n_a)+g \\
 n_{b\omega} \\
 n_{ba} \\
 0
 \end{bmatrix}
 \tag{6}
 $$

In (6), $\omega$, $a$ are the gyroscope and accelerometer measurment (i.e. the IMU input).


$$
u = [\omega, a]
\tag{7}
$$

In (6), $n_{\omega}$, $n_a$, $n_{b\omega}$, $n_{ba}$ represent the noise of gyroscope, the noise of accelerometer, the noise of gyroscope bias and the noise of accelerometer bias respectively.

$$
w = [n_{\omega}, n_a, n_{b\omega}, n_{ba}]
\tag{8}
$$

### Discrete model:
We can discretize the continuous model in (6) at the IMU sampling period $\Delta{t}$

$$ 
x_{i} = x_{i-1} \boxplus ( f(x_{i-1}, u, w) \Delta{t}) 
\tag{9}
$$

Where:

$$ 
f(x_{i-1}, u, w) = 
\begin{bmatrix}
 v \\ 
\omega - b_{\omega}-n_{\omega}\\
 0 \\
 0 \\
 R(a-b_a-n_a)+g \\
 n_{b\omega} \\
 n_{ba} \\
 0
 \end{bmatrix} 
 \tag{10}
$$


### State prediction (Forward Propagation):
We can predict the state when a new IMU input $u$ is received.

$$
\hat{x}_{i} = \bar{x}_{i-1} \boxplus ( f(\bar{x}_{i-1}, u, 0) \Delta{t})
 \tag{11}
$$

* $\bar{x}_{i-1}$: Optimal state in previous time.
* $\hat{x}_{i}$: Predicted state in current time.

#### Prediction function:
Because we don't know in advance the exact amount of noise $w$, so the $w$ are set to zeros. 

$$ 
f(x_{i-1}, u, 0) = 
\begin{bmatrix}
 v \\ 
 \omega - b_{\omega} \\
 0 \\
 0 \\
 R(a-b_a)+g \\
 0 \\
 0 \\
 0
 \end{bmatrix} \tag{12}
$$

####  State with superscript

* $x$: The ground truth of state.
* $\bar{x}$: Optimal state.
* $\hat{x}$: Predicted value of the state.
* $\~{x}$: The state error.

####  Error state

Although we do not know the specific noise, but we can evaluate the uncertainty of state error by propagation of covariance.

$\~{x}$ is the error between ground-true x and predicted x

$$ 
x = \hat{x} \boxplus \~{x} \\
\~{x} = x \boxminus \hat{x} 
\tag{13}
$$

Plugging (9) and (11) into (13), we get:

$$ 
\~{x}= (x \boxplus ( f(x, u, w) \Delta{t})) 
\boxminus 
(\hat{x} \boxplus ( f(\hat{x}, u, 0) \Delta{t}))  \\
\~{x}= (\hat{x} \boxplus \~{x} \boxplus ( f(x, u, w) \Delta{t})) 
\boxminus 
(\hat{x} \boxplus ( f(\hat{x}, u, 0) \Delta{t}))
\tag{14}
$$

After linearizing (14), we obtain the following:

$$
\~{x}= F_{\~x}\~x + F_{w}w 
\tag{15}
$$

Because the proof of $F_{\~x}$ and $ F_{w}$ are too complex, so we compute them in Appendix A.

Denoting the covariance of white noises w as Q, then the propagated covariance P can be computed iteratively.

$$
\hat{P_i}= F_{\~x}\hat{P_{i-1}}F_{\~x}^T +  F_{w}QF_{w}^T 
\tag{16}
$$

$\hat{P_{i-1}}$ is previous covariance.

### Iterated state update:

We use MAP (Max A Posteriori estimation) to correct $\hat{x}$ when LiDAR measurements are received.

* Prediction error function  (prior):

$$
p(x) = p(\hat{x} \boxplus \~{x}) = p(\hat{x}) + J \~{x} \\
= \hat{x}^\kappa \boxminus\hat{x} + J \~{x} \\
\cong d + J\~{x}\sim \mathcal{N}(0,P)
\tag{17}
$$

Where:

$$
d=\hat{x}^{\kappa}\boxminus\hat{x}
$$

* Observation error function (likelihood):

$$
h(x) = h(\hat{x} \boxplus \~{x}) = h(\hat{x}) + H\~{x} \\
\cong z + H\~{x}\sim \mathcal{N}(0,R)
\tag{18}
$$

$$
d=\hat{x}^{\kappa}\boxminus\hat{x}
$$

Combining the prior with the likelihood yields the posteriori:

$$
\argmin_{\substack{\~{x}}} ( \|z + H\~{x}\|_{R^{-1}}^2 + \|d + J\~{x}\|_{\hat{P}^{-1}}^2  )
\tag{19}
$$

We can solve (19) by gauss newton method.
Iterative update x ($\kappa$: The number of iterations)

$$
K=PH^T(HPH^T+R)^{−1}
\tag{20}
$$

$$
\~{x}_{i}^{\kappa+1} = \hat{x}_{i}^{\kappa} \boxplus  ( -Kz - (I - KH)J^{-1} (\hat{x}^{\kappa}_{i}\boxminus\hat{x}_{i})) 
\tag{21}
$$

When $\~{x}$ converges, update P

$$
\bar{P}=(I - KH)P
\tag{22}
$$

where:

$$
P=J^{-1} \hat{P}_{i} J^{-T}
\tag{23}
$$

We show the proof of (21) and (22) In Appendix B.

### Error State Extended Kalman Filter
Finally, the ESEKF can be formalized as following:

### Prediction step
Predict x

$$
\hat{x}_{i} = \bar{x}_{i-1} \boxplus ( f(\bar{x}_{i-1}, u, w)|_{w=0} \Delta{t}) 
\tag{24}
$$

Predict P 

$$
\hat{P}_{i} = F_x \bar{P}_{i-1}F_x^T + F_w Q F_w^T
\tag{25}
$$

### Update step
Correct x

$$
K=PH^T(HPH^T+R)^{−1}
\tag{27}
$$

$$
\bar{x}_{i} = \hat{x}_{i}^{\kappa} \boxplus  ( -Kz - (I - KH)J^{-1} (\hat{x}^{\kappa}_{i}\boxminus\hat{x}_{i}))
\tag{28}
$$

Correct P 
$$
\bar{P}=(I - KH)P
\tag{29}
$$
