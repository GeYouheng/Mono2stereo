#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <string>

#define WIDTH_NEW 840
#define HEIGHT_NEW 360
#define F 200

using namespace cv;

class Rectify
{
public:
  void initial(std::string fileName)
  {
    cv::FileStorage fSettings(fileName, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;

    DistCoef = cv::Mat(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.k3"];
    DistCoef.at<float>(3) = fSettings["Camera.k4"];

    // DistCoef = cv::Mat::zeros(4, 1, CV_32F);

    int width = fSettings["Camera.width"];
    int height = fSettings["Camera.height"];

    Mat intrinsic_mat(K);
    intrinsic_mat.copyTo(new_intrinsic_mat);
    //调节视场大小,乘的系数越小视场越大
    new_intrinsic_mat.at<float>(0, 0) = F;
    new_intrinsic_mat.at<float>(1, 1) = F;
    //调节校正图中心，建议置于校正图中心
    new_intrinsic_mat.at<float>(0, 2) = 0.5 * WIDTH_NEW;
    new_intrinsic_mat.at<float>(1, 2) = 0.5 * HEIGHT_NEW;

    std::cout<<fileName<<"'s new_intrinsic_mat:\n"<<new_intrinsic_mat<<std::endl;
  }

  void rectify(Mat image_original, Mat &image_rect)
  {
    //cv::fisheye::undistortImage(image_original, image_rect, K, DistCoef, new_intrinsic_mat, Size(WIDTH_NEW, HEIGHT_NEW));
    cv::fisheye::undistortImage(image_original, image_rect, K, DistCoef, new_intrinsic_mat, Size(WIDTH_NEW, HEIGHT_NEW));

  }

private:
  cv::Mat K;
  cv::Mat DistCoef;
  cv::Mat new_intrinsic_mat;

};

void converterCallback(const sensor_msgs::ImagePtr &msg);

image_transport::Publisher pub_left_original;
image_transport::Publisher pub_right_original;
image_transport::Publisher pub_left_360p;
image_transport::Publisher pub_right_360p;
image_transport::Publisher pub_left_rect;
image_transport::Publisher pub_right_rect;

Rectify Rectify_left, Rectify_right;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "converter");
  ros::NodeHandle m;
  ros::Subscriber sub = m.subscribe("/usb_cam/image_raw", 1, converterCallback);
  ros::NodeHandle nh;
  image_transport::ImageTransport it_left_original(nh);
  image_transport::ImageTransport it_right_original(nh);
  image_transport::ImageTransport it_left_360p(nh);
  image_transport::ImageTransport it_right_360p(nh);
  image_transport::ImageTransport it_left_rect(nh);
  image_transport::ImageTransport it_right_rect(nh);
  pub_left_original = it_left_original.advertise("mono2stereo/left_original", 1);
  pub_right_original = it_right_original.advertise("mono2stereo/right_original", 1);
  pub_left_360p = it_left_360p.advertise("mono2stereo/left_360p", 1);
  pub_right_360p = it_right_360p.advertise("mono2stereo/right_360p", 1);
  pub_left_rect = it_left_rect.advertise("mono2stereo/left_rect", 1);
  pub_right_rect = it_right_rect.advertise("mono2stereo/right_rect", 1);

  Rectify_left.initial("/home/kudzu/catkin_ws/src/mono2stereo/setting_left_360p.yaml");
  Rectify_right.initial("/home/kudzu/catkin_ws/src/mono2stereo/setting_right_360p.yaml");

  ros::spin();
}

void converterCallback(const sensor_msgs::ImagePtr &msg)
{
  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::Mat image_left = image(cv::Rect(0, 0, image.cols / 2, image.rows));
  cv::Mat image_right = image(cv::Rect(image.cols / 2, 0, image.cols / 2, image.rows));
  cv::Mat image_left_360p, image_right_360p, image_left_720p, image_right_720p, image_left_rect, image_right_rect;
  cv::resize(image_left, image_left_360p, cv::Size(0, 0), 1, 0.5, cv::INTER_LINEAR);
  cv::resize(image_right, image_right_360p, cv::Size(0, 0), 1, 0.5, cv::INTER_LINEAR);
  Rectify_left.rectify(image_left_360p,image_left_rect);
  Rectify_right.rectify(image_right_360p,image_right_rect);

  sensor_msgs::ImagePtr msg_left_original = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left).toImageMsg();
  sensor_msgs::ImagePtr msg_right_original = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right).toImageMsg();
  sensor_msgs::ImagePtr msg_left_360p = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left_360p).toImageMsg();
  sensor_msgs::ImagePtr msg_right_360p = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right_360p).toImageMsg();
  sensor_msgs::ImagePtr msg_left_rect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left_rect).toImageMsg();
  sensor_msgs::ImagePtr msg_right_rect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right_rect).toImageMsg();

  msg_left_original->header = msg->header;
  msg_right_original->header = msg->header;
  msg_left_360p->header = msg->header;
  msg_right_360p->header = msg->header;
  msg_left_rect->header = msg->header;
  msg_right_rect->header = msg->header;

  pub_left_original.publish(msg_left_original);
  pub_right_original.publish(msg_right_original);
  pub_left_360p.publish(msg_left_360p);
  pub_right_360p.publish(msg_right_360p);
  pub_left_rect.publish(msg_left_rect);
  pub_right_rect.publish(msg_right_rect);

  ros::spinOnce();
}
