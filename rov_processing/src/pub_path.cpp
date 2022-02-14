#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define PI (3.1415926535897932346)
#define ANGLE (99.29932199390001)

class TestPath
{

public:
    TestPath()
    {
        sub_odom_1 = nh.subscribe<nav_msgs::Odometry> ("/odometry_1", 1, &TestPath::odomCallback1,this);
        pub_path_1= nh.advertise<nav_msgs::Path>("/path_1",1);

        sub_odom_2 = nh.subscribe<nav_msgs::Odometry> ("/odometry_1", 1, &TestPath::odomCallback2,this);
        pub_path_2= nh.advertise<nav_msgs::Path>("/path_2",1);

    }

    void odomCallback1(const nav_msgs::Odometry::ConstPtr& msg );

    void odomCallback2(const nav_msgs::Odometry::ConstPtr& msg );

private:
    ros::NodeHandle nh;

    ros::Subscriber sub_odom_1;
    ros::Subscriber sub_odom_2;
    ros::Publisher pub_path_1; 
    ros::Publisher pub_path_2; 
    // ros::Publisher pub_euler; 

    nav_msgs::Path path1;
    nav_msgs::Path path2;

};

void TestPath::odomCallback2(const nav_msgs::Odometry::ConstPtr& msg )
{ 
/************************** publish  path ***************************************/
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose = msg->pose.pose;

    path2.header.frame_id = msg->header.frame_id;
    path2.header.stamp = msg->header.stamp;
    path2.poses.push_back(pose);

    pub_path_2.publish(path2);

/************************** publish imu heading ***************************************/
    // // get quaternion
    // tf::Quaternion q_imu(
    //     msg->orientation.x,
    //     msg->orientation.y,
    //     msg->orientation.z,
    //     msg->orientation.w);
    // tf::Matrix3x3 m_imu(q_imu);
    // // convert to euler
    // geometry_msgs::Vector3 euler_imu;
    // m_imu.getRPY(euler_imu.x, euler_imu.y, euler_imu.z);

    // pub_euler.publish(euler_imu);
}


void TestPath::odomCallback1(const nav_msgs::Odometry::ConstPtr& msg )
{ 
/************************** publish  path ***************************************/
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose = msg->pose.pose;

    path1.header.frame_id = msg->header.frame_id;
    path1.header.stamp = msg->header.stamp;
    path1.poses.push_back(pose);

    pub_path_1.publish(path1);

/************************** publish gps's base_link tf, so we can see base_link in rviz***************************************/
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Quaternion q;
    // transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    // q.setW(msg->pose.pose.orientation.w);
    // q.setX(msg->pose.pose.orientation.x);
    // q.setY(msg->pose.pose.orientation.y);
    // q.setZ(msg->pose.pose.orientation.z);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, msg->header.frame_id, msg->child_frame_id));

}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "path_node");

        TestPath tester;

        ros::spin();

        return 0;
}











