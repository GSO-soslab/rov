#include <eigen3/Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <ros/ros.h>
#include "tf_conversions/tf_eigen.h"

#define PI 3.1415926

int main()
{
  Eigen::Quaterniond Q(1,0,0,0); // w,x,y,z
  Eigen::Matrix3d R = Q.toRotationMatrix();
  std::cout<<"R: "<< R<<std::endl;

/** roation matrix to Euler angle **/
  // Eigen::Matrix3d R;

  // R <<    0.0113627,     0.99993 , -0.0032232,   
  //        0.999888,  -0.0113307,  0.00975484 ,  
  //       0.00971763, -0.00333368,   -0.999947; 

  // tf::Matrix3x3 ma;
  // tf::matrixEigenToTF(R,ma);

  // double roll, pitch, yaw;
  // ma.getRPY(roll, pitch, yaw);
  // std::cout<<"RPY[roll:  "<<roll<<" pitch: "<<pitch<< " yaw: "<<yaw<<"]"<<std::endl;

/** rotaion matrix multiply**/
  // Eigen::Matrix3d R1;
  // R1 = Eigen::AngleAxisd(PI/2.0, Eigen::Vector3d::UnitZ()) * 
  //      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
  //      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

  // Eigen::Matrix3d R2;
  // R2 << 0.011362662503790503, 0.9998882224979653, 0.00971763400653785,
  //     0.9999302479992342, -0.011330740940760782, -0.0033336835368381165,
  //     -0.003223202912534585, 0.009754835703046718, -0.9999472256791313; 

  // Eigen::Matrix3d R3;
  // R3 = R1*R2;

  // std::cout<<R3<<std::endl;

/** inverse Transmforation matrix**/
  // Eigen::Matrix4d T;
  // T << 
  //     0.011362662503790503, 0.9998882224979653, 0.00971763400653785, -0.043854791823369293,
  //     0.9999302479992342, -0.011330740940760782, -0.0033336835368381165, -0.3111845498969625,
  //     -0.003223202912534585, 0.009754835703046718, -0.9999472256791313, -0.2777771992579014,
  //     0.0, 0.0, 0.0, 1.0;

  // std::cout<<"T: \n"<<T<<std::endl;

  // Eigen::Matrix4d T_;
  // T_ = T.inverse();

  // std::cout<<"T inverse: \n"<<T_<<std::endl;

  // Eigen::Matrix4d T__;
  // T__ = T*T_;
  // std::cout<<"T__: \n"<<T__<<std::endl;
  
}