# Kalman Filter for Cursor Position Estimate

![Demo](media/output.gif)

This is a minimal Python demo that uses the Kalman filter to estimate the position of the mouse cursor. The system assumes constant velocity motion and Gaussian noise, making it a linear-Gaussian system. Under these assumptions, the Kalman filter provides the best linear unbiased estimate of the system state. For this demo, the recursive estimator will track the 4-dimensional state of the cursor:

$$
\mathbf x_k = 
\begin{bmatrix}
x_k \\
y_k \\
\dot x_k \\
\dot y_k 
\end{bmatrix}
\tag{1}
$$

Where
- $x$, $y$ represent the cursor's 2D position [pixels]
- $\dot x$, $\dot y$ represent the cursor's velocities along each axis [pixels/s]

