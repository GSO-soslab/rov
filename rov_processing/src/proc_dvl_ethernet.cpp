
// ros
#include <ros/ros.h>

#include "ds_sensor_msgs/Dvl.h"
#include "ds_sensor_msgs/NortekDF21.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// filter
#include <Eigen/Dense>
#include "kalman-cpp/kalman.hpp"

#include <mutex>
#include <queue>
#include <thread>

#define PI 3.1415926

class ProcessDvl
{
public:
  ProcessDvl()
  {
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud", 5);
    depth_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/depth", 5);
    twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/twist", 5);

    sub_dvl = nh.subscribe<ds_sensor_msgs::NortekDF21>("/dvl", 1, &ProcessDvl::dvlCallback, this);

    // load params
    nh.param<double>("air_pressure", air_pressure, 0.0);
    nh.param<double>("standard_soundSpeed", standard_soundSpeed, 1500);
    nh.param<double>("estimated_soundSpeed", estimated_soundSpeed, 1500);
    nh.param<double>("z_B_D", z_B_D, 1.0);
    nh.param<bool>("pub_pointcloud", pub_pointcloud, false);

    // init 
    init();
  }
  
  void dvlCallback(const ds_sensor_msgs::NortekDF21::ConstPtr& msg);

  void init();
  void process();

private:
  // ros
  ros::NodeHandle nh;
  ros::Publisher depth_pub;
  ros::Publisher twist_pub;
  ros::Publisher cloud_pub;

  // ros::Subscriber odom_sub;
  ros::Subscriber sub_dvl;

  // buff
  std::queue<ds_sensor_msgs::NortekDF21::ConstPtr> msg_Buf_;
  std::mutex m_lock_;

  // current profile related
  KalmanFilter depth_kf_;
  Eigen::VectorXd y_depth;

  bool depth_initialized = false;
  double init_depth;

  // buttom track related
  double temp;
  double original_soundSpeed;
  double correct_ratio;
  double beam_angle;

  KalmanFilter twist_kf_;
  Eigen::VectorXd y_twist;

  // parameters
  double air_pressure;
  double estimated_soundSpeed;
  double standard_soundSpeed;
  double z_B_D;
  bool pub_pointcloud;
};

void ProcessDvl::init()
{
/** depth related **/
  int n1 = 1; // Number of states
  int m1 = 1; // Number of measurements
  double dt1 = 1.0/2; // Time step, not used actually

  Eigen::MatrixXd A1(n1, n1); // System dynamics matrix
  Eigen::MatrixXd C1(m1, n1); // Jacobain of measurement model
  Eigen::MatrixXd Q1(n1, n1); // Process noise covariance
  Eigen::MatrixXd R1(m1, m1); // Measurement noise covariance
  Eigen::MatrixXd P1(n1, n1); // Estimate error covariance
  A1 << 1;
  C1 << 1;
  Q1 << 0.01;
  R1 << 0.25*0.25;
  P1 << 0.001;
  depth_kf_.reset(dt1, A1, C1, Q1, R1, P1);

  // init filter
  y_depth.resize(m1); //measurement
  y_depth << 0.0;
  depth_kf_.init(0, y_depth);

/** velocity related **/
  temp = 0.0;
  correct_ratio = 1.0;

  beam_angle = 25*PI/180;

  // filter
  int n2 = 3; // Number of states
  int m2 = 3; // Number of measurements
  double dt2 = 1.0/4; // Time step, not used actually

  Eigen::MatrixXd A2(n2, n2); // System dynamics matrix
  Eigen::MatrixXd C2(m2, n2); // Output matrix
  Eigen::MatrixXd Q2(n2, n2); // Process noise covariance
  Eigen::MatrixXd R2(m2, m2); // Measurement noise covariance
  Eigen::MatrixXd P2(n2, n2); // Estimate error covariance
  A2 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  C2 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Q2 << 0.0008, 0, 0, 0, 0.0008, 0, 0, 0, 0.0005;
  R2 << 0.01, 0 , 0, 0, 0.01, 0, 0, 0, 0.01;
  P2 << 0.001, 0 , 0, 0, 0.001, 0, 0, 0, 0.001;
  twist_kf_.reset(dt2, A2, C2, Q2, R2, P2);

  // init filter
  y_twist.resize(m2); //measurement
  y_twist << 0.0, 0.0, 0.0;
  twist_kf_.init(0, y_twist);

}

void ProcessDvl::dvlCallback(const ds_sensor_msgs::NortekDF21::ConstPtr& msg)
{
  m_lock_.lock();
  msg_Buf_.push(msg);
  m_lock_.unlock();
}

void ProcessDvl::process()
{
  while(1)
  {
    // check buttom track data
    if(!msg_Buf_.empty()) {
    /** Get data **/
      m_lock_.lock();
      
      //// header
      std_msgs::Header header_dvl = msg_Buf_.front()->header;

      //// 3d velocity
      geometry_msgs::Vector3 twist;

      twist.x = msg_Buf_.front()->velX;
      twist.y = msg_Buf_.front()->velY;

        if (msg_Buf_.front()->velZ1 == -32.768f && msg_Buf_.front()->velZ2 != -32.768f) 
            twist.z = msg_Buf_.front()->velZ2;
        else if (msg_Buf_.front()->velZ1 != -32.768f && msg_Buf_.front()->velZ2 == -32.768f)
            twist.z = msg_Buf_.front()->velZ1;
        else 
            twist.z = (msg_Buf_.front()->velZ1 + msg_Buf_.front()->velZ2)/ 2.0;

      //// pressure
      double pressure = msg_Buf_.front()->pressure;
      //// original sound speed
      original_soundSpeed = msg_Buf_.front()->speed_sound;

      //// veritcal distance
      double distance[4]; 
      distance[0] = msg_Buf_.front()->distBeam[0];
      distance[1] = msg_Buf_.front()->distBeam[1];
      distance[2] = msg_Buf_.front()->distBeam[2];
      distance[3] = msg_Buf_.front()->distBeam[3];

      msg_Buf_.pop();
      m_lock_.unlock();

    /****************************** Handle depth ******************************/

      //// depth initialization
      if(!depth_initialized){ 
        depth_initialized = true;
        init_depth = (pressure - air_pressure)*10;
      }

      //// update filter
      y_depth << init_depth - (pressure - air_pressure)*10;
      depth_kf_.update(y_depth);

      //// publish depth msg
      geometry_msgs::PoseWithCovarianceStamped depth_msg;
      depth_msg.header = header_dvl;
      depth_msg.header.frame_id = "odom";
      depth_msg.pose.pose.position.z = depth_kf_.state()(0); 
      depth_msg.pose.covariance[14] = depth_kf_.covariance()(0,0);
      depth_pub.publish(depth_msg);

    /**************************** Handle velocity ***********************/
      // correct sound speed
      ////Mackenzie
      // speed = 1448.96 + 4.591*sci_water_temp - 5.304/100*sci_water_temp.^2 ...
      //     +2.374/10000 *sci_water_temp.^3 + 1.34*(salinity-35) + 1.63/100*sci_water_pressure...
      //     +1.675/10^7 *sci_water_pressure.^2 - 1.025/100*sci_water_temp.*(salinity-35)...
      //     - 7.139/10^13*sci_water_temp.*sci_water_pressure.^3

      // double estimated_soundSpeed = standard_soundSpeed * sqrt( (273.15+temp) / 273.15);

      correct_ratio = estimated_soundSpeed / original_soundSpeed;
      
      double v_x = twist.x * correct_ratio;
      double v_y = twist.y * correct_ratio;
      double v_z = twist.z * correct_ratio;

      // filter for inf. data and do kalman filter
      if (abs(v_x)>5 || abs(v_y)>5 || abs(v_z)>5) {
        twist_kf_.update(y_twist); // use previous one
      }
      else {
        y_twist << v_x, v_y, v_z;
        twist_kf_.update(y_twist);
      }
      // publish msg
      geometry_msgs::TwistWithCovarianceStamped twist_msg;
      twist_msg.header = header_dvl;
      twist_msg.twist.twist.linear.x = twist_kf_.state()(0);  
      twist_msg.twist.twist.linear.y = twist_kf_.state()(1); 
      twist_msg.twist.twist.linear.z = twist_kf_.state()(2); 
      twist_msg.twist.covariance[0] = twist_kf_.covariance()(0,0);
      twist_msg.twist.covariance[7] = twist_kf_.covariance()(1,1);
      twist_msg.twist.covariance[14] = twist_kf_.covariance()(2,2);
      twist_pub.publish(twist_msg);

    /**************************** Handle pointcloud ***********************/

      // setup the pointcloud for 4 points with only XYZ property
      sensor_msgs::PointCloud2 cloud_msg;
      sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
      modifier.setPointCloud2FieldsByString(1, "xyz");    
      modifier.resize(4); 

      // re-generate 3D location of target point
      std::vector<Eigen::Vector3d> points;

      double beam_azimuth[] = {PI/4.0, -PI/4.0, -3.0*PI/4.0, 3.0*PI/4.0};
      for (int i = 0; i < 4; i++) {
          Eigen::Vector3d pt;
          pt(0) =  distance[i] * correct_ratio * tan(beam_angle) * cos(beam_azimuth[i]);
          pt(1) =  distance[i] * correct_ratio * tan(beam_angle) * sin(beam_azimuth[i]);
          pt(2) =  distance[i] * correct_ratio;
          points.push_back(pt);
      }

      // setup the points XYZ
      sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(cloud_msg, "z");

      for (size_t i = 0; i < 4; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
          const Eigen::Vector3d& point = points.at(i);
          *ros_pc2_x = point(0);
          *ros_pc2_y = point(1);
          *ros_pc2_z = point(2);
      }

      // setup header
      cloud_msg.header = header_dvl;

      if(pub_pointcloud)
        cloud_pub.publish(cloud_msg);
    }

    if(depth_initialized) {

      //// publish global and odom frame tf: init_depth + distance between base and dvl

      // get diatnce between base and dvl
      // tf::TransformListener listener;
      // tf::StampedTransform trans_B_D;
      // listener.waitForTransform("base_link", "dvl", ros::Time(0), ros::Duration(1.0));
      // listener.lookupTransform("base_link", "dvl", ros::Time(0), trans_B_D);
      // double z_B_D= abs(trans_B_D.getOrigin().z()); 

      double distance = (init_depth + z_B_D) * -1;

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(0.0, 0.0, distance) );
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/world", "/odom"));
    }

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Process_DVL_Node");

  ProcessDvl  dvl_processer;

  std::thread mainThread{&ProcessDvl::process, &dvl_processer};

  ros::spin();

  // mainThread.join();
  //TODO: catch ctrl+c to close thread

  return 0;
}