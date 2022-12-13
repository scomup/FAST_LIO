## Error State Extended Kalman Filter

### State
$$ 
x = [p_{wi}, R_{wi}, R_{il}, t_{il}, v_{wi}, b_{\omega}, b_a, g] \tag{1}
$$

* $p_{wi}$ The imu positon in imu frame.
* $R_{wi}$ The rotation form lidar to imu.
* $R_{il}$ The translation form lidar to imu.
* $t_{il}$ The imu velocity in world frame.
* $b_{\omega}$ The gyroscope bias.
* $b_a$ The accelerometer bias.
* $g$ The gravity vector.

$$ 
x = [p, R, R_{il}, t_{il}, v, b_{\omega}, b_a, g] \tag{2}
$$

### State Estimate
The continuous time differential equation of state is given by:
$$ 
\dot{x} = [\dot{p}, \dot{R}, \dot{R_{il}}, \dot{t_{il}}, \dot{v}, \dot{b_{\omega}}, \dot{b_a}, \dot{g}] \tag{3}
$$
$$ 
\dot{x} = 
\begin{bmatrix}
 v \\ 
 R(\omega - b_{\omega}-n_{\omega})\times \\
 0 \\
 0 \\
 R(a-b_a-n_a)-g \\
 n_{\omega} \\
 n_{a} \\
 0
 \end{bmatrix} \tag{4}
$$
Discrete model:
$$ 
x_{i} = x_{i-1} \boxplus ( f(x_{i-1}, u, w) \Delta{t}) \tag{5}
$$
where:
$$ 
f(x_{i-1}, u, w) = 
\begin{bmatrix}
 v \\ 
 R(\omega - b_{\omega}-n_{\omega})\times \\
 0 \\
 0 \\
 R(a-b_a-n_a)-g \\
 n_{b\omega} \\
 n_{ba} \\
 0
 \end{bmatrix} \tag{6}
$$

input:
$$
u=[\omega,a] \tag{7}
$$
noise:
$$
w=[n_{\omega},n_a,n_{b\omega},n_{ba}]  \tag{8}
$$

### State prediction (Forward Propagation):

$$
\hat{x}_{i} = \bar{x}_{i-1} \boxplus ( f(\bar{x}_{i-1}, u, 0) \Delta{t})
 \tag{9}
$$

* $\bar{x}_{i-1}$: Optimal state in previous time.
* $\hat{x}_{i}$: predicted state in current time.

#### prediction function:
$$ 
f(x_{i-1}, u, 0) = 
\begin{bmatrix}
 v \\ 
 R(\omega - b_{\omega})\times \\
 0 \\
 0 \\
 R(a-b_a)-g \\
 0 \\
 0 \\
 0
 \end{bmatrix} \tag{10}
$$


####  State with superscript

* $x$: The ground truth of state.
* $\bar{x}$: Optimal state.
* $\hat{x}$: predicted value of the state.
* $\~{x}$: The state error .

####  Error state

$$ 
x = \hat{x} \boxplus \~{x} \\
\~{x} = x \boxminus \hat{x} \tag{11}
$$

$$ 
\~{x} = x \boxminus \hat{x} \\
= (x \boxplus ( f(x, u, w) \Delta{t})) 
\boxminus 
(\hat{x} \boxplus ( f(\hat{x}, u, 0) \Delta{t}))  \\
= (\hat{x} \boxplus \~{x} \boxplus ( f(x, u, w) \Delta{t})) 
\boxminus 
(\hat{x} \boxplus ( f(\hat{x}, u, 0) \Delta{t})) \\
= F_{\~x}\~x + F_{w}w \tag{12}
$$

$$
g(\~{x}, w) \stackrel{\mathrm{def}}{=}   f(x, u, w) \Delta{t}
\tag{13}
$$

$$
G(\~{x},g) \stackrel{\mathrm{def}}{=} \hat{x} \\= (\hat{x} \boxplus \~{x} \boxplus g(\~{x}, w)) 
\boxminus 
(\hat{x} \boxplus g(0, 0)) \\
\tag{14}
$$

$$
F_{\~x}=\frac{\partial G}{\partial \~x} + \frac{\partial G}{\partial g} \frac{\partial g}{\partial \~x} \tag{15}
$$

$$
F_{w}=\frac{\partial G}{\partial g} \frac{\partial g}{\partial w} \tag{16}
$$

### Iterated state update:

### Error State Extended Kalman Filter

We want to estimate the error state, and use it to update the current state.

We want estimate the error State $\hat{s}$, which  
### Prediction step
$$
\hat{x}_{i} = \bar{x}_{i-1} \boxplus ( f(\bar{x}_{i-1}, u, w)|_{w=0} \Delta{t})\\
\hat{P}_{i} = F_x \bar{P}_{i-1}F_x^T + F_w Q F_w^T
$$

### Update step
$$
K=PH^T(HPH^T+R)^{−1} \\
\bar{x}_{i} = \hat{x}_{i} \boxplus  (( -Kz - (I - KH)J^{-1} (\hat{x}^{\kappa}_{i}\boxminus\hat{x}_{i})) \\
\bar{P}=(I - KH)P
$$

where:
$$
P=J^{-1} \hat{P}_{i} J^{-T}
$$

Observation error function (posteriori):
$$
h(x) = h(\hat{x} \boxplus \~{x}) = h(\hat{x}) + H\~{x} \\
\cong z + H\~{x}\sim \mathcal{N}(0,R)
$$

Prediction error function  (prior):

$$
p(x) = p(\hat{x} \boxplus \~{x}) = p(\hat{x}) + J \~{x} \\
= \hat{x}^\kappa \boxminus\hat{x} + J \~{x} \\
\cong d + J\~{x}\sim \mathcal{N}(0,P)
$$

$$
d=\hat{x}^{\kappa}\boxminus\hat{x}
$$
Combining the prior with the posteriori yields the maximum a-posteriori estimate:

$$
\argmin_{\substack{\~{x}}} ( \|z + H\~{x}\|_{R^{-1}}^2 + \|d + J\~{x}\|_{\hat{P}^{-1}}^2  )
$$

$$
\argmin_{\substack{\~{x}}} ( \frac{1}{2} \mathcal{R}^T \Sigma \mathcal{R} )
$$


$$
\mathcal{R}  = 
\begin{bmatrix}
 z + H\~{x}       \\  
 d + J\~{x}       \\  
\end{bmatrix}
$$

$$
\Sigma  = 
\begin{bmatrix}
 R^{-1} &   \\  
 & \hat{P}^{-1}  \\  
\end{bmatrix} 
$$

$$
\mathcal{J} =
\begin{bmatrix}
 H \\  
 J \\  
\end{bmatrix} 
$$

$$
\mathcal{H} = \mathcal{J}^T \Sigma \mathcal{J} 
$$

$$
\mathcal{g} = \mathcal{J}^T \Sigma \mathcal{R} 
$$

$$
\mathcal{H} = H^T R^{-1} H + J^T \hat{P}^{-1} J
= H^T R^{-1} H + P^{-1}
$$

$$
\mathcal{H}^{-1} = (H^T R^{-1} H + P^{-1})^{-1} \\
=P - PH^T(R+HPH^T)^{-1}HP \\
=P - KHP \\
=(I - KH)P \\
=KRH^{-T}
$$

https://en.wikipedia.org/wiki/Woodbury_matrix_identity
$$
\Delta{\~{x}} = - \mathcal{H}^{-1} \mathcal{g} \\
=-\mathcal{H}^{-1} \mathcal{J}^T \Sigma \mathcal{R} \\
= -\mathcal{H}^{-1}(H^TR^{-1}(z+H\~{x})) -\mathcal{H}^{-1}(J^T\hat{P}^{-1}(d+J\~{x})) \\
= -KRH^{-T}(H^TR^{-1}(z+H\~{x})) -(I - KH)(J^{-1} \hat{P} J^{-T})(J^T\hat{P}^{-1}(d+J\~{x})) \\
= -K(z+H\~{x}) - (I - KH)(J^{-1} (d+J\~{x})) \\
= -Kz-KH\~{x} - (I - KH)J^{-1} d - \~{x} + KH\~{x} \\
= -Kz - (I - KH)J^{-1} d - \~{x} \\
$$

$$
\~{x}^{ \kappa + 1} = \~{x}^{ \kappa } \boxplus \Delta{\~{x}} \\
\~{x}^{ \kappa + 1} = -Kz - (I - KH)J^{-1} d
$$

$$
\bar{P}=\mathcal{H}^{-1}=(I - KH)P
$$
https://onlinelibrary.wiley.com/doi/pdf/10.1002/9780470824566.app1
