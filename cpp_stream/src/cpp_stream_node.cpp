#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <linux/v4l2-controls.h>
#include <cv_bridge/cv_bridge.h>
#include "arducam_mipicamera.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "image_publisher");
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Publisher pub = it.advertise("camera/image", 1);
     
   CAMERA_INSTANCE camera_instance;
   int res = arducam_init_camera(&camera_instance);
   printf("Open CAMERA and initiallize\n");
   int width = 1280;
   int height = 800;
   printf("Setting the resolution...\n");
   res = arducam_set_resolution(camera_instance, &width, &height);
   printf("Current resolution is %dx%d\n", width, height);
   IMAGE_FORMAT fmt = {IMAGE_ENCODING_RAW_BAYER, 50};

    unsigned int index = 0;
    struct camera_ctrl support_cam_ctrl;
    while (!arducam_get_support_controls(camera_instance, &support_cam_ctrl, index++)) {
        int value = 0;
        if (arducam_get_control(camera_instance, support_cam_ctrl.id, &value)) {
            printf("Get ctrl %s fail.\n", support_cam_ctrl.desc);
        }
        printf("index: %d, CID: 0x%08X, desc: %s, min: %d, max: %d, default: %d, current: %d\n",
            index, support_cam_ctrl.id, support_cam_ctrl.desc, support_cam_ctrl.min_value,
            support_cam_ctrl.max_value, support_cam_ctrl.default_value, value);
    }

   // Get FPS, exposure and gain parameters 
   
   int exposure, updated_exposure;
   int gain, updated_gain;
   int fps, updated_fps;
   nh.getParam(ros::this_node::getName() + "//exposure", exposure);
   nh.getParam(ros::this_node::getName() + "//gain", gain);
   nh.getParam(ros::this_node::getName() + "//fps", fps);

   res = arducam_set_mode(camera_instance, 13);
   arducam_set_control(camera_instance, V4L2_CID_EXPOSURE, exposure);
   arducam_set_control(camera_instance, V4L2_CID_GAIN, gain);
   printf("set mode result: %d\n", res);
   
   while (nh.ok()) {
      BUFFER *buffer = arducam_capture(camera_instance, &fmt, 3000);
      ros::Time ros_time_now = ros::Time::now();
      cv::Mat image = cv::Mat(cv::Size(width,height ), CV_8UC1, buffer->data);
      cv::Mat resized_image;
      
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
      msg->header.stamp = ros_time_now;//ros::Time::now();
      pub.publish(msg);
      arducam_release_buffer(buffer);
      ros::spinOnce();
   }
   arducam_close_camera(camera_instance);
 }
