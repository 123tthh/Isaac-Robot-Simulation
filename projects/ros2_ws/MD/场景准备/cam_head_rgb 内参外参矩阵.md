$$
{相机标定参数}  \\

{内参矩阵}\\
相机内参 f_x, f_y, c_x, c_y 如下：

\\

\mathbf{K} = \begin{bmatrix}
f_x & 0   & c_x \\
0   & f_y & c_y \\
0   & 0   & 1
\end{bmatrix}
= \begin{bmatrix}
386.610 & 0      & 323.369 \\
0      & 386.121 & 240.390 \\
0      & 0      & 1
\end{bmatrix}

\\

{畸变系数}
\\


\mathbf{d} = [k_1, k_2, p_1, p_2, k_3] = 
[-0.055595,\; 0.063234,\; -0.000160,\; 0.000960,\; -0.020771]


\\

{相机到基座的变换矩阵}


\mathbf{T}_{\text{cam}}^{\text{base}} =
\begin{bmatrix}
 0.05477221 & -0.05318410 &  0.99708147 &  0.27415172 \\
-0.99813362 & -0.02992383 &  0.05323388 & -0.00957896 \\
 0.02700530 & -0.99813627 & -0.05472383 &  1.24538354 \\
 0          &  0          &  0          &  1
\end{bmatrix}
$$

/World/Robot/camera_H_link/Realsense_H
/World/Robot/camera_L_link/Realsense_L
/World/Robot/camera_R_link/Realsense_R
