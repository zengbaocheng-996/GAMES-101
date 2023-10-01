# 3D 平移

$$
T(t_x,t_y,t_z)\cdot{
\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)
}=
\left(
\begin{matrix}
1&0&0&t_x\\
0&1&0&t_y\\
0&0&1&t_z\\
0&0&0&1
\end{matrix}
\right)\cdot{\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)=\left(
\begin{matrix}
x+t_x\\
y+t_y\\
z+t_z\\
1
\end{matrix}
\right)=\left(
\begin{matrix}
x'\\
y'\\
z'\\
1
\end{matrix}
\right)}
$$

# 3D 缩放

$$
S(s_x,s_y,s_z)\cdot{
\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)
}=
\left(
\begin{matrix}
s_x&0&0&0\\
0&s_y&0&0\\
0&0&s_z&0\\
0&0&0&1
\end{matrix}
\right)\cdot{\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)=\left(
\begin{matrix}
s_xx\\
s_yy\\
s_zz\\
1
\end{matrix}
\right)=\left(
\begin{matrix}
x'\\
y'\\
z'\\
1
\end{matrix}
\right)}
$$

# 3D 旋转

###### 绕 x 轴旋转

$$
R_x(\alpha)\cdot{
\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)
}=
\left(
\begin{matrix}
1&0&0&0\\
0&cos\alpha&-sin\alpha&0\\
0&sin\alpha&cos\alpha&0\\
0&0&0&1
\end{matrix}
\right)\cdot{\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)=\left(
\begin{matrix}
x'\\
y'\\
z'\\
1
\end{matrix}
\right)}
$$

###### 绕 y 轴旋转

$$
R_y(\alpha)\cdot{
\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)
}=
\left(
\begin{matrix}
cos\alpha&0&sin\alpha&0\\
0&1&0&0\\
-sin\alpha&0&cos\alpha&0\\
0&0&0&1
\end{matrix}
\right)\cdot{\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)=\left(
\begin{matrix}
x'\\
y'\\
z'\\
1
\end{matrix}
\right)}
$$

###### 绕 z 轴旋转

$$
R_z(\alpha)\cdot{
\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)
}=
\left(
\begin{matrix}
cos\alpha&-sin\alpha&0&0\\
sin\alpha&cos\alpha&0&0\\
0&0&1&0\\
0&0&0&1
\end{matrix}
\right)\cdot{\left(
\begin{matrix}
x\\
y\\
z\\
1
\end{matrix}
\right)=\left(
\begin{matrix}
x'\\
y'\\
z'\\
1
\end{matrix}
\right)}
$$

# 3D 点

3D 空间中一点 (x, y, z) 可被表示为以下三种形式

$$
\begin{align}
&(x,y,z,1)\\
&(kx,ky,kz,k\neq0)\\
&(xz,yz,z^2,z\neq0)
\end{align}
$$

# 模型视图变换

简单来说就是把相机和模型按约定放在坐标系上

$$
M_{view}=R_{view}T_{view}
$$

# 投影变换

投影变换包含正交投影和透视投影

###### 正交投影 Orthographic projection

$$
[left,right]\times[bottom,top]\times[far,near]\rightarrow[-1,1]^3
$$

$$
M_{ortho}=S_{ortho}T_{ortho}=\left(
\begin{matrix}
\frac{2}{r-l}&0&0&0\\
0&\frac{2}{t-b}&0&0\\
0&0&\frac{2}{n-f}&0\\
0&0&0&1
\end{matrix}
\right)
\left(
\begin{matrix}
1&0&0&-\frac{r+l}{2}\\
0&1&0&-\frac{t+b}{2}\\
0&0&1&-\frac{n+f}{2}\\
0&0&0&1
\end{matrix}
\right)
$$

###### 透视投影 Perspective projection

简单来说就是把远平面上的点映射到近平面上

$$
M_{persp}=M_{ortho}M_{persp\rightarrow{ortho}}=M_{ortho}\left(
\begin{matrix}
n&0&0&0\\
0&n&0&0\\
0&0&n+f&-nf\\
0&0&1&0
\end{matrix}
\right)
$$

$$
tan\frac{fovY}{2}=\frac{top}{|n|}
$$

$$
aspect=\frac{width}{height}=\frac{right}{top}
$$

# 作业 1

```c++
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float alpha = rotation_angle/180.0 * std::acos(-1);
    model << std::cos(alpha), - std::sin(alpha), 0, 0,
             std::sin(alpha),   std::cos(alpha), 0, 0,
                           0,                 0, 1, 0,
                           0,                 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // std::cout<<eye_fov/2<<std::endl;
    float top = zNear * std::tan(eye_fov/2/180.0 * std::acos(-1));
    float right = aspect_ratio * top;
    Eigen::Matrix4f T_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f S_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Persp2Ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    T_ortho << 1/right,     0, 				0, 0,
                     0, 1/top, 				0, 0,
    		         0,     0, 2/(zNear-zFar), 0,
                     0,     0,              0, 1;

    S_ortho << 1, 0, 0, 			  0,
    		   0, 1, 0, 			  0,
    		   0, 0, 1, -(zNear+zFar)/2,
    		   0, 0, 0, 			  1;

    Persp2Ortho <<  zNear,     0,          0,           0,
    				    0, zNear,          0,           0,
    				    0,     0, zNear+zFar, -zNear*zFar,
    					0,     0,          1,           0;
	
    projection = S_ortho * T_ortho * Persp2Ortho;
    return projection;
}
```

