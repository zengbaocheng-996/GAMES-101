#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

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