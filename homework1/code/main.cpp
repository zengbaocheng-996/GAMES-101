#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

Eigen::Vector3f Calculate()
{
    Eigen::Vector3f p(2.0f, 1.0f, 0.0f);
    Eigen::Matrix3f T,R;
    T<<1,0,1,
       0,1,2,
       0,0,1;
    R<<std::cos(45.0/180*std::acos(-1)),-std::sin(45.0/180*std::acos(-1)),0,
       std::sin(45.0/180*std::acos(-1)), std::cos(45.0/180*std::acos(-1)),0,
                  0,            0,1;
    // std::cout<<45/180*std::acos(-1)<<std::endl;
    // std::cout<<"trans"<<std::endl;
    // std::cout<<T<<std::endl;
    // std::cout<<"rot"<<std::endl;
    // std::cout<<R<<std::endl;
    return R*T*p;
}


int main(){
    std::cout<<Calculate()<<std::endl;
}