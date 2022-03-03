// ros
#include <ros/ros.h>
#include "ds_sensor_msgs/NortekDF21.h"
#include <std_msgs/string.h>
#include <sensor_msgs/PointCloud2.h>

class SimpleExample
{
public:
    SimpleExample() {

        sub = nh.subscribe<ds_sensor_msgs::NortekDF21>("/rov/sensors/dvl/df21", 1, &SimpleExample::callback, this);

        pub = nh.advertise<std_msgs::string>("/test",1);
    }

    ~SimpleExample() {}

    void callback(const ds_sensor_msgs::NortekDF21::ConstPtr& input);


private:
    ros::NodeHandle nh;

    ros::Subscriber sub;
    ros::Publisher pub;

};

void SimpleExample::callback(const ds_sensor_msgs::NortekDF21::ConstPtr& input) {

    auto cloud = sensor_msgs::PointCloud2{};

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Simple_Example_Node");

  SimpleExample  example;
  ros::spin();

  return 0;
}