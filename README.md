# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model

### States

Based on the vehicle kinematic model, the state variables include the $x$ and $y$ locations, the orientation $\psi$, the speed $v$, the two errors w.r.t. the offsets from the center of the lane, crosstrack error $cte$ and orientation error $e\psi$.

### Actuators

Actuators include the steering angle and the acceleration (throttle).

### Update equations

 The update equations for the model:
 
 $$ x[t+1] = x[t] + v[t] * cos(\psi[t]) * dt $$
 
 $$ y[t+1] = y[t] + v[t] * sin(\psi[t]) * dt $$
 
 $$ \psi[t+1] = \psi[t] + v[t] / Lf * \delta[t] * dt $$
 
 $$ v[t+1] = v[t] + a[t] * dt $$
 
 $$ cte[t+1] = f(x[t]) - y[t] + v[t] * sin(e\psi[t]) * dt $$
 
 $$ e\psi[t+1] = \psi[t] - \psi des[t] + v[t] * \delta[t] / Lf * dt $$

 where $cte[t] = f(x[t]) - y[t]$, $e\psi[t] = \psi[t] - \psi des[t]$, $f(x[t])$ is the fitted polynomial at $x[t]$ and $\psi des[t]$ is the derivative of $f(x[t])$.
 
## Timestep Length and Elapsed Duration (N & dt)

The time horizon ($T$) was chosen to 2 s after experiments. It was shown that the MPC could drive safely around the track with $T = 2$ s. Time step duration ($dt$) was set equal to the latancy of the simulation (0.1 s), hense, 20 time steps was used.

The general guidelines say that $T$ should be as large as possible, while $dt$ should be as small as possible. After tuning these hyperparameters, $T = 2$ and $dt = 0.1$ works the best.

Once $N$ and $dt$ are fixed, we need to tune other hyperparameters to drive the vehicle safely. These hyperparameters are the weights of different cost items and the reference speed.

## Polynomial Fitting and MPC Preprocessing

All computations are performed in the vehicle coordinate system. The coordinates of waypoints in vehicle coordinates are obtained by first shifting the origin to the current poistion of the vehicle and a subsequet 2D rotation to align the x-axis with the heading direction. The waypoints are obtained in the frame of the vehicle. A third order polynomial is then fitted to the waypoints. The transformation between coordinate systems is implemented. The transformation used is 

```
 waypoints_xs[i] = dx * cos(-psi) - dy * sin(-psi);
 waypoints_ys[i] = dy * cos(-psi) + dx * sin(-psi);
```
where `waypoints_xs, waypoints_ys` denote coordinates in the vehicle coordinate system. 



## Model Predictive Control with Latency

The 100 ms latency is considered in the kinematic model, as follows:

```
const double current_px = 0.0 + v * dt;
const double current_py = 0.0;
const double current_psi = 0.0 + v * (-delta) / Lf * dt;
const double current_v = v + a * dt;
const double current_cte = cte + v * sin(epsi) * dt;
const double current_epsi = epsi + v * (-delta) / Lf * dt;
```


## Results

**Result 1**: [YouTube video](https://youtu.be/ls2YlXilbEg)


**Result 2**: [YouTube video](https://youtu.be/4q-qolE37m4)

## Discussion

Given a set of hyperparameters, the vehicle may drive safely for one loop. But the performance is not stable for different loops or new simulations. Therefore, the vehicle drives safely for one loop does not mean we have the optimal set of hyperparameters. Monta Carlo runs are needed and parameter tuning continued until a stable drive.

The parameter tuning takes time, one possible method is decreasing the speed.



## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.


