#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class ImageStitcher {
public:
  typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  
  ImageStitcher() : 
    left_info_sub_(
        nh_.subscribe("left_camera_info", 1,
        std::bind(&ImageStitcher::info_cb_once, this, std::placeholders::_1, left_mapx_, left_mapy_, left_info_sub_))),
    front_info_sub_(
        nh_.subscribe("front_camera_info", 1,
        std::bind(&ImageStitcher::info_cb_once, this, std::placeholders::_1, front_mapx_, front_mapy_, front_info_sub_))),
    right_info_sub_(
        nh_.subscribe("right_camera_info", 1,
        std::bind(&ImageStitcher::info_cb_once, this, std::placeholders::_1, right_mapx_, right_mapy_, right_info_sub_))),
    stitched_img_pub_(
        nh_.advertise<sensor_msgs::Image>("stitched_image", 1)),
    left_img_sub_(nh_, "left_image", 1),
    front_img_sub_(nh_, "front_image", 1),
    right_img_sub_(nh_, "right_image", 1),
    sync_(SyncPolicy(10), front_img_sub_, right_img_sub_, left_img_sub_) {
  }

  void spin() {
    ros::Rate r(10);
    
    bool had_initialized = false;
    while(ros::ok()) {
      if (!had_initialized &&
          !front_mapx_.empty() && !front_mapy_.empty() &&
          !right_mapx_.empty() && !right_mapy_.empty() &&
          !left_mapx_.empty() && !left_mapy_.empty()) {
           
        sync_.registerCallback(boost::bind(&ImageStitcher::sync_images_cb, this, _1, _2, _3));
        had_initialized = true;
      }
      
      r.sleep();
      ros::spinOnce();
    }
  }

private:
  void sync_images_cb(
      const sensor_msgs::ImageConstPtr& front_img,
      const sensor_msgs::ImageConstPtr& right_img,
      const sensor_msgs::ImageConstPtr& left_img) {
    
    std::array<cv::Mat, 3> images = {
      cv_bridge::toCvShare(left_img)->image,
      cv_bridge::toCvShare(front_img)->image,
      cv_bridge::toCvShare(right_img)->image
    };
    
    const int cols_sum = images[0].cols + images[1].cols + images[2].cols;
    cv::Mat stitched_image(cv::Size(cols_sum, images[0].rows), cv_bridge::getCvType(front_img->encoding));
    cv::Rect roi_rect;
    for(const cv::Mat& img : images) {
      roi_rect.width = img.cols;
      roi_rect.height = img.rows;
      cv::Mat roi(stitched_image, roi_rect);
      img.copyTo(roi);
      roi_rect.x += img.cols;
    }
      
    cv_bridge::CvImage out_msg;
    out_msg.header   = front_img->header;
    out_msg.encoding = front_img->encoding;
    out_msg.image    = stitched_image;

    stitched_img_pub_.publish(out_msg.toImageMsg());

    ROS_ERROR("sync_images_cb!");
  }

  void info_cb_once(const sensor_msgs::CameraInfoConstPtr& info_msg,
      cv::Mat& mapx, cv::Mat& mapy, ros::Subscriber& subscriber) {
    sensor_msgs::CameraInfo info = *info_msg;
    cv::Mat_<double> D(1, info.D.size(), &info.D[0]);
    cv::Matx33d K(&info.K[0]);
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat(), K,
        cv::Size(info.width, info.height), CV_32FC1, mapx, mapy);
    subscriber.shutdown();
  }

  ros::NodeHandle nh_;
  //std::array<cv::Mat, 3> mapx_;
  //std::array<cv::Mat, 3> mapy_;
  cv::Mat left_mapx_, left_mapy_;
  cv::Mat front_mapx_, front_mapy_;
  cv::Mat right_mapx_, right_mapy_;
  
  ros::Subscriber left_info_sub_;
  ros::Subscriber front_info_sub_;
  ros::Subscriber right_info_sub_;
  ros::Publisher stitched_img_pub_;

  message_filters::Subscriber<sensor_msgs::Image> left_img_sub_;
  message_filters::Subscriber<sensor_msgs::Image> front_img_sub_;
  message_filters::Subscriber<sensor_msgs::Image> right_img_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;

};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "simple_image_stitcher");
  ImageStitcher node;
  node.spin();

  return 0;
}


