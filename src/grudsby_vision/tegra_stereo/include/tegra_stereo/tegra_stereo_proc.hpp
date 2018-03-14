#pragma once

#include <memory>
#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>

#include <image_geometry/stereo_camera_model.h>
#include <camera_calibration_parsers/parse.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "tegra_stereo/disparity_method.h"

namespace tegra_stereo
{

using message_filters::sync_policies::ExactTime;

class TegraStereoProc : public nodelet::Nodelet
{
    using SubscriberFilter_t = image_transport::SubscriberFilter;
    using InfoSubscriber_t   = message_filters::Subscriber<sensor_msgs::CameraInfo>;
    using ImageExactPolicy_t = ExactTime<sensor_msgs::Image, sensor_msgs::Image>;
    using ImageExactSync_t = message_filters::Synchronizer<ImageExactPolicy_t>;
    using InfoExactPolicy_t = ExactTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>;
    using InfoExactSync_t = message_filters::Synchronizer<InfoExactPolicy_t>;

public:
    TegraStereoProc();
    ~TegraStereoProc();

    virtual void onInit();
    void imageCallback (const sensor_msgs::ImageConstPtr &l_image_msg, const sensor_msgs::ImageConstPtr &r_image_msg);
    void infoCallback (const sensor_msgs::CameraInfoConstPtr &l_info_msg, const sensor_msgs::CameraInfoConstPtr &r_info_msg);

private:
    std::once_flag calibration_initialized_flag_;

    boost::shared_ptr<image_transport::ImageTransport> imageTransport_;

    SubscriberFilter_t left_raw_sub_, right_raw_sub_;
    InfoSubscriber_t left_info_sub_, right_info_sub_;

    boost::shared_ptr<ImageExactSync_t> image_exact_sync_;
    boost::shared_ptr<InfoExactSync_t> info_exact_sync_;

    ros::Publisher pub_disparity_;
    ros::Publisher pub_points_;
    ros::Publisher pub_points2_;

    image_transport::CameraPublisher pub_rect_left_;
    image_transport::CameraPublisher pub_rect_right_;

    image_transport::CameraPublisher pub_disparity_raw_;
    image_transport::CameraPublisher pub_depth_Image_;

    //
    sensor_msgs::CameraInfo mCameraInfoLeft_;
    sensor_msgs::CameraInfo mCameraInfoRight_;
    // camera models
    image_geometry::PinholeCameraModel left_model_;
    image_geometry::PinholeCameraModel right_model_;
    image_geometry::StereoCameraModel stereo_model_;

    // scratch buffer for dense point cloud
    mutable cv::Mat_<cv::Vec3f> dense_points_;

    // filter for disparity image
    //cv::Ptr<cv::ximgproc::FastGlobalSmootherFilter> disp_filter_;

    // stereo matching parameters, more in configuration.h
    ///
    /// \brief p1_ penalties for small disparity changes
    ///
    int p1_;
    ///
    /// \brief p2_ penalties for larger disparity discontinuities
    ///
    int p2_;

    bool rectifyImages_;
    int queue_size_;
    float elapsed_time_ms_acc_;
    uint32_t elapsed_time_counter_;
    std::string out_left_frame_id;
    std::string out_right_frame_id;

    void publishRectifiedImages (const cv::Mat &left_rect,
                                 const cv::Mat &right_rect,
                                 const sensor_msgs::ImageConstPtr &l_image_msg,
                                 const sensor_msgs::ImageConstPtr &r_image_msg);
    bool processRectified(const cv::Mat &left_rect_cv, const cv::Mat &right_rect_cv, const sensor_msgs::ImageConstPtr &leftImgPtr);
    void processDisparity (const cv::Mat &disparity, const std_msgs::Header &header, stereo_msgs::DisparityImagePtr &disparityMsgPtr);
    void publishPointcloud (const cv::Mat &disparity);
    void processPoints(const stereo_msgs::DisparityImageConstPtr& disparityMsgPrt,
                                        const cv::Mat& color, const std::string& encoding,
                                        sensor_msgs::PointCloudPtr& points) const;
    void processPoints2(const stereo_msgs::DisparityImageConstPtr& disparityMsgPrt,
                                         const cv::Mat& color, const std::string& encoding,
                                         sensor_msgs::PointCloud2Ptr &points) const;
    sensor_msgs::ImagePtr processDepthImage(const stereo_msgs::DisparityImageConstPtr& disparityMsgPrt) const;
};

}  // namespace

