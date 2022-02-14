
// ros
#include <ros/ros.h>
#include <nortek_dvl/ButtomTrack.h>
#include <nortek_dvl/CurrentProfile.h>
#include <nortek_dvl/CellMeasure.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
// filter
#include <Eigen/Dense>
#include "kalman.hpp"

#include <mutex>
#include <queue>
#include <thread>

#define PI 3.1415926

class ProcessDvl
{
public:
  ProcessDvl()
  {
    depth_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/rov/processed/dvl/depth_filtered", 5);
    twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/rov/processed/dvl/twist_filtered", 5);

    buttom_track_sub = nh.subscribe<nortek_dvl::ButtomTrack> 
                       ("/rov/sensors/dvl/buttom_track", 1, &ProcessDvl::buttomTrackCallback, this);
    current_profile_sub = nh.subscribe<nortek_dvl::CurrentProfile> 
                        ("/rov/sensors/dvl/current_profile", 1, &ProcessDvl::currentProfileCallback, this);

    // load params
    nh.param<double>("air_pressure", air_pressure, 0.0);
    nh.param<double>("standard_soundSpeed", standard_soundSpeed, 1500);
    nh.param<bool>("print_depth", print_depth, false);

    // init 
    init();
  }
  
  void buttomTrackCallback(const nortek_dvl::ButtomTrack::ConstPtr& bt);
  void currentProfileCallback(const nortek_dvl::CurrentProfile::ConstPtr& cp);

  void init();
  void process();

private:
  // ros
  ros::NodeHandle nh;
  ros::Publisher depth_pub;
  ros::Publisher twist_pub;

  // ros::Subscriber odom_sub;
  ros::Subscriber buttom_track_sub;
  ros::Subscriber current_profile_sub;

  // buff
  std::queue<nortek_dvl::ButtomTrack::ConstPtr> bt_Buf_;
  std::queue<nortek_dvl::CurrentProfile::ConstPtr> cp_Buf_;
  std::mutex m_lock_;

  // current profile related
  KalmanFilter depth_kf_;
  Eigen::VectorXd y_depth;

  bool is_depth_first = true;
  double init_depth;

  // buttom track related
  double temp;
  double standard_soundSpeed;
  double original_soundSpeed;
  double correct_ratio;

  double x_factor, y_factor, z_factor;

  KalmanFilter twist_kf_;
  Eigen::VectorXd y_twist;

  // parameters
  double air_pressure;
  bool print_depth;
};

void ProcessDvl::init()
{
/** Currect profile related **/
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

/** Buttom track related **/
  temp = 0.0;
  correct_ratio = 1.0;
  // range measurement in DVL frame
  x_factor = sin(25*PI/180) * cos(45*PI/180);
  y_factor = sin(25*PI/180) * sin(45*PI/180);
  z_factor = cos(25*PI/180);

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

void ProcessDvl::buttomTrackCallback(const nortek_dvl::ButtomTrack::ConstPtr& bt)
{
  m_lock_.lock();
  bt_Buf_.push(bt);
  m_lock_.unlock();
}

void ProcessDvl::currentProfileCallback(const nortek_dvl::CurrentProfile::ConstPtr& cp)
{
  m_lock_.lock();
  cp_Buf_.push(cp);
  m_lock_.unlock();
}

void ProcessDvl::process()
{
  while(1)
  {
    // check current profile data
    if(!cp_Buf_.empty()) {
    /** Get data **/
      m_lock_.lock();

      std_msgs::Header cp_header = cp_Buf_.front()->header;
      double pressure = cp_Buf_.front()->pressure;
      double pressure_std = cp_Buf_.front()->pressure_std_dev;
      temp = cp_Buf_.front()->temperature;
      original_soundSpeed = cp_Buf_.front()->sound_speed;
      
      cp_Buf_.pop();
      m_lock_.unlock();

    /** Kalman filter to depth noise **/
      // get init position
      if(is_depth_first){ 
        is_depth_first = false;
        init_depth = pressure - air_pressure;
      }
      // update filter
      y_depth << init_depth - (pressure - air_pressure);
      depth_kf_.update(y_depth);

      if (print_depth)
        std::cout<<"depth to sea level: "<< pressure - air_pressure <<std::endl;

      // publish msg
      geometry_msgs::PoseWithCovarianceStamped depth_msg;
      depth_msg.header = cp_header;
      depth_msg.header.frame_id = "odom";
      depth_msg.pose.pose.position.z = depth_kf_.state()(0); 
      depth_msg.pose.covariance[14] = depth_kf_.covariance()(0,0);
      depth_pub.publish(depth_msg);
    }

    // check buttom track data
    if(!bt_Buf_.empty()) {
    /** Get data **/
      m_lock_.lock();

      std_msgs::Header bt_header = bt_Buf_.front()->header;
      geometry_msgs::Vector3 twist = bt_Buf_.front()->speed;
      auto range = bt_Buf_.front()->vertical_distance;


      bt_Buf_.pop();
      m_lock_.unlock();

    /** Filter velocity **/
      // correct sound speed
      ////Mackenzie
      // speed = 1448.96 + 4.591*sci_water_temp - 5.304/100*sci_water_temp.^2 ...
      //     +2.374/10000 *sci_water_temp.^3 + 1.34*(salinity-35) + 1.63/100*sci_water_pressure...
      //     +1.675/10^7 *sci_water_pressure.^2 - 1.025/100*sci_water_temp.*(salinity-35)...
      //     - 7.139/10^13*sci_water_temp.*sci_water_pressure.^3

      // double estimated_soundSpeed = standard_soundSpeed * sqrt( (273.15+temp) / 273.15);
      double estimated_soundSpeed = 1417; //;

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
      twist_msg.header = bt_header;
      twist_msg.twist.twist.linear.x = twist_kf_.state()(0);  
      twist_msg.twist.twist.linear.y = twist_kf_.state()(1); 
      twist_msg.twist.twist.linear.z = twist_kf_.state()(2); 
      twist_msg.twist.covariance[0] = twist_kf_.covariance()(0,0);
      twist_msg.twist.covariance[7] = twist_kf_.covariance()(1,1);
      twist_msg.twist.covariance[14] = twist_kf_.covariance()(2,2);
      twist_pub.publish(twist_msg);
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