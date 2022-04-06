
// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
// ros msg type header
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h> 

// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

class SaveImage
{
public:
    SaveImage(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) 
     : nh_(nh), nh_private_(nh_private), path("/tmp")
    {
        sub = nh_.subscribe<sensor_msgs::Image> ("/image", 10, 
              &SaveImage::msgCallback, this);

        pub = nh_.advertise<std_msgs::String>("/published_str",1);

        service = nh_.advertiseService("/trigger_save", 
                  &SaveImage::srvCallback, this); 

        is_save = false;

        count = 0;

        if (!nh_private_.getParam("path", path))
            ROS_WARN("[Tools-Save-Image]: no path provied, will save to %s", path.c_str());
    }

    ~SaveImage()
    {
    }

    void msgCallback(const sensor_msgs::Image::ConstPtr& input);

    bool srvCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub;
    ros::Publisher pub;
    ros::ServiceServer service;

    bool is_save;
    std::mutex m_lock_;

    unsigned long count;
    std::string path;
};

bool SaveImage::srvCallback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
    response.success = true; // boring, but valid response info
    response.message = "here is a response string";

    is_save = true;

    return true;
}

void SaveImage::msgCallback(const sensor_msgs::Image::ConstPtr& input)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try{
        cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8); //MONO8
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::string str_sec = std::to_string(input->header.stamp.sec);
    std::string str_nsec = std::to_string(input->header.stamp.nsec);
    if(str_nsec.size()<9)
        ROS_WARN("less then 9, %s", str_nsec.c_str());

    if( is_save) {
        is_save = false;

        cv::imwrite(path + str_sec + str_nsec + ".png", cv_ptr->image);

        printf("\nSaved img with t:%lf with count:%ld\n", input->header.stamp.toSec(), count);
        count++;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Save_Image_Node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    SaveImage  example(nh, nh_private);

    ros::spin();
   

    return 0;
}