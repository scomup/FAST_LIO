## Error State Extended Kalman Filter

### State
$$ 
s = [p_{wi}, R_{wi}, R_{il}, t_{il}, v_{wi}, b_{\omega}, b_a, g] \tag{1}
$$

* $p_{wi}$ The imu positon in imu frame.
* $R_{wi}$ The rotation form lidar to imu.
* $R_{il}$ The translation form lidar to imu.
* $t_{il}$ The imu velocity in world frame.
* $b_{\omega}$ The gyroscope bias.
* $b_a$ The accelerometer bias.
* $g$ The gravity vector.

### Error State

* $s$: The ground truth of state.
* $\bar{s}$: Optimal state.
* $\hat{s}$: Estimated value of the state.
* $\~{s}$: The state error.

$$ 
s = \hat{s} \boxplus \~{s} \\
\~{s} = s \boxminus \hat{s}
$$


### Error State Extended Kalman Filter

We want to estimate the error state, and use it to update the current state.

We want estimate the error State $\hat{s}$, which  
### Prediction step
$$
s_t = f(s_{t-1}, u) \\
P_t = F_sP_{t-1}F_s^T + F_wP_{t-1}F_w^T
$$