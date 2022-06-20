
// ros
#include <ros/ros.h>
#include <nortek_dvl/ButtomTrack.h>


class TestBT
{
public:
    TestBT() {
        sub = nh.subscribe<nortek_dvl::ButtomTrack> ("/rov/sensors/dvl/buttom_track", 1, &TestBT::callback, this);

    }

    ~TestBT() {}

    void callback(const nortek_dvl::ButtomTrack::ConstPtr& input);


private:
    ros::NodeHandle nh;

    ros::Subscriber sub;

};

void TestBT::callback(const nortek_dvl::ButtomTrack::ConstPtr& input) {

  printf("t:%0.9f,dt_1:%f, dt_2:%f, v:%f,%f,%f\n",
        input->header.stamp.toSec(), input->dt_1, input->dt_2,
        input->speed.x, input->speed.y, input->speed.z);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Simple_Example_Node");

  TestBT  example;
  ros::spin();

  return 0;
}