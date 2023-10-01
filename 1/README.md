# 2D 平移

$$
T(t_x,t_y)\cdot{
\left(
\begin{matrix}
x\\
y\\
1
\end{matrix}
\right)
}=
\left(
\begin{matrix}
1&0&t_x\\0&1&t_y\\0&0&1
\end{matrix}
\right)\cdot{\left(
\begin{matrix}
x\\y\\1
\end{matrix}
\right)=\left(
\begin{matrix}
x+t_x\\y+t_y\\1
\end{matrix}
\right)=\left(
\begin{matrix}
x'\\y'\\1
\end{matrix}
\right)}
$$

# 2D 缩放

$$
S(s_x,s_y)\cdot{
\left(
\begin{matrix}
x\\y\\1
\end{matrix}
\right)
}=
\left(
\begin{matrix}
s_x&0&0\\0&s_y&0\\0&0&1
\end{matrix}
\right)\cdot{\left(
\begin{matrix}
x\\y\\1
\end{matrix}
\right)=\left(
\begin{matrix}
s_xx\\s_yt_y\\1
\end{matrix}
\right)=\left(
\begin{matrix}
x'\\y'\\1
\end{matrix}
\right)}
$$

# 2D 旋转

$$
R(\alpha)\cdot{
\left(
\begin{matrix}
x\\y\\1
\end{matrix}
\right)
}=
\left(
\begin{matrix}
cos\alpha&-sin\alpha&0\\sin\alpha&cos\alpha&0\\0&0&1
\end{matrix}
\right)\cdot{\left(
\begin{matrix}
x\\y\\1
\end{matrix}
\right)=\left(
\begin{matrix}
xcos\alpha-ysin\alpha\\xsin\alpha+ycos\alpha\\1
\end{matrix}
\right)=\left(
\begin{matrix}
x'\\y'\\1
\end{matrix}
\right)}
$$

# 2D 举个例子

###### 先逆时针旋转 45 度再平移 (1, 0)

$$
T(1,0)
\cdot{R(45°)}\cdot{
\left(
\begin{matrix}
x\\y\\1
\end{matrix}
\right)
}
=\left(
\begin{matrix}
1&0&1\\0&1&0\\0&0&1
\end{matrix}
\right)
\left(
\begin{matrix}
cos45°&-sin45°&0\\sin45°&cos45°&0\\0&0&1
\end{matrix}
\right)\
\left(
\begin{matrix}
x\\y\\1
\end{matrix}
\right)
=\left(
\begin{matrix}
x'\\y'\\1
\end{matrix}
\right)
$$

###### 注意

$$
T(1,0)
\cdot{R(45°)}\neq{
R(45°)\cdot{T(1,0)}}
$$

# 作业 0

给定一个点 P=(2, 1)，将该点绕原点先逆时针旋转 45°，再平移 (1, 2)，计算出变换后点的坐标（要求用齐次坐标进行计算）。

```c++
Eigen::Vector3f Calculate()
{
    Eigen::Vector3f p(2.0f, 1.0f, 0.0f);
    Eigen::Matrix3f T, R;
    T << 1, 0, 1,
         0, 1, 2,
         0, 0, 1;
    float alpha = 45.0/180 * std::acos(-1);
    R << std::cos(alpha),-std::sin(alpha), 0,
         std::sin(alpha), std::cos(alpha), 0,
                       0,               0, 1;
    // std::cout<<"translate"<<std::endl;
    // std::cout<<T<<std::endl;
    // std::cout<<"alpha"<<std::endl;
    // std::cout<<alpha<<std::endl;
    // std::cout<<"rotate"<<std::endl;
    // std::cout<<R<<std::endl;
    return T * R * p;
}

int main()
{
    std::cout << Calculate() << std::endl;
}
```

