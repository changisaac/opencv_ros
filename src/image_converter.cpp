//have astra_launch astra.launch running

#include <ros/ros.h>
//used to subscribe and publish images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int32.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  int depth;

  //constructor
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    //window to display for processed image
    //cv::namedWindow(OPENCV_WINDOW);
  }

  //destructor
  ~ImageConverter()
  {
    //destroy display window
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //ptr to image stream
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //transform depth image to 
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    depth = cv_ptr->image.at<short int>(cv::Point(240, 320));//you can change 240,320 to your interested pixel
    
    //if processing incoming image  
    /* 
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(240, 320), 10, CV_RGB(255,0,0));
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
   */ 
  }
};

int main(int argc, char** argv)
{
  //start node
  ros::init(argc, argv, "image_converter");
  //start node handler
  ros::NodeHandle n;
  //start image converter (deoth to 2D array)
  ImageConverter ic;
  //start publisher for sending depth values
  ros::Publisher depth_chatter = n.advertise<std_msgs::Int32>("depth_chatter", 1);
  //set output hz
  ros::Rate loop_rate(30);

  ROS_INFO("Depth Query Node Initialized\n");  

  while (ros::ok())
  {
    std_msgs::Int32 depth_pub;
    depth_pub.data = ic.depth;
    //debug:
    //ROS_INFO("Depth: %d", ic.depth);
    depth_chatter.publish(depth_pub);
    loop_rate.sleep();
    ros::spinOnce();
  }
  

  return 0;
}






