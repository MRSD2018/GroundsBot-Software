/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tegra_stereo/tegra_stereo_proc.hpp"
namespace tegra_stereo
{

TegraStereoProc::TegraStereoProc() {}
TegraStereoProc::~TegraStereoProc() {}

void TegraStereoProc::onInit()
{

    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    private_nh.param<int> ("P1", p1_, 20);
    private_nh.param<int> ("P2", p2_, 100);

    private_nh.param<int> ("queue_size", queue_size_, 1u);
    private_nh.param<bool> ("rectify_images", rectifyImages_, true);
    private_nh.param<std::string> ("out_left_frame_id", out_left_frame_id, "");
    private_nh.param<std::string> ("out_right_frame_id", out_right_frame_id, "");

    // ros sub pubs
    imageTransport_ = boost::make_shared<image_transport::ImageTransport> (private_nh);

    left_raw_sub_.subscribe (*imageTransport_.get(), "/stereo_camera/left/image_raw", queue_size_);
    right_raw_sub_.subscribe (*imageTransport_.get(), "/stereo_camera/right/image_raw", queue_size_);

    left_info_sub_.subscribe (private_nh, "/stereo_camera/left/camera_info", queue_size_);
    right_info_sub_.subscribe (private_nh, "/stereo_camera/right/camera_info", queue_size_);

    pub_rect_left_ = imageTransport_->advertiseCamera ("left/image_rect", 1);
    pub_rect_right_ = imageTransport_->advertiseCamera("right/image_rect", 1);
    pub_disparity_raw_ = imageTransport_->advertiseCamera("disparity_raw/image_raw", 1);
    pub_depth_Image_ = imageTransport_->advertiseCamera("depth/image", 1);
    pub_disparity_ = private_nh.advertise<stereo_msgs::DisparityImage> ("disparity", 1);
    pub_points_ = private_nh.advertise<sensor_msgs::PointCloud> ("points", 1);
    pub_points2_ = private_nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);

    // Synchronize input topics
    info_exact_sync_ = boost::make_shared<InfoExactSync_t> (InfoExactPolicy_t (10u), left_info_sub_, right_info_sub_);
    info_exact_sync_->registerCallback (boost::bind (&TegraStereoProc::infoCallback, this, _1, _2));

    image_exact_sync_ = boost::make_shared<ImageExactSync_t> (ImageExactPolicy_t (10u), left_raw_sub_, right_raw_sub_);

    //camera calibration files
    std::string cameraCalibrationFileLeft;
    std::string cameraCalibrationFileRight;
    private_nh.param<std::string> ("camera_calibration_file_left", cameraCalibrationFileLeft, "");
    private_nh.param<std::string> ("camera_calibration_file_right", cameraCalibrationFileRight, "");

    if (! (cameraCalibrationFileRight.empty() && cameraCalibrationFileLeft.empty()))
    {
        std::string cameraName;
        NODELET_INFO_STREAM ("Stereo calibration left:" << cameraCalibrationFileLeft);
        NODELET_INFO_STREAM ("Stereo calibration right:" << cameraCalibrationFileRight);
        camera_calibration_parsers::readCalibration (cameraCalibrationFileLeft, cameraName, mCameraInfoLeft_);
        camera_calibration_parsers::readCalibration (cameraCalibrationFileRight, cameraName, mCameraInfoRight_);

        left_model_.fromCameraInfo (mCameraInfoLeft_);
        right_model_.fromCameraInfo (mCameraInfoRight_);
        stereo_model_.fromCameraInfo (mCameraInfoLeft_, mCameraInfoRight_);
	std::call_once (calibration_initialized_flag_, [ &, this] () {
		image_exact_sync_->registerCallback (boost::bind (&TegraStereoProc::imageCallback, this, _1, _2));
	});
        NODELET_INFO ("Stereo calibration initialized from file");

    }
    else
    {
        NODELET_INFO ("Stereo calibration files are not specified ,waiting for camera_info messages");
    }
	

    // Initialize Semi-Global Matcher
    init_disparity_method (static_cast<uint8_t> (p1_), static_cast<uint8_t> (p2_));
    NODELET_INFO ("Init done; P1 %d; P2: %d", p1_, p2_);
}

void TegraStereoProc::infoCallback (
    const sensor_msgs::CameraInfoConstPtr &l_info_msg,
    const sensor_msgs::CameraInfoConstPtr &r_info_msg)
{

    std::call_once (calibration_initialized_flag_, [ &, this] ()
    {

        mCameraInfoLeft_ = *l_info_msg;
        mCameraInfoRight_ = *r_info_msg;
        left_model_.fromCameraInfo (l_info_msg);
        right_model_.fromCameraInfo (r_info_msg);
        stereo_model_.fromCameraInfo (l_info_msg, r_info_msg);
        //we do not need to listen to this anymore
        right_info_sub_.unsubscribe();
        left_info_sub_.unsubscribe();
        image_exact_sync_->registerCallback (boost::bind (&TegraStereoProc::imageCallback, this, _1, _2));
        NODELET_INFO ("Stereo calibration initialized from first message");
    });
}

void TegraStereoProc::imageCallback (
    const sensor_msgs::ImageConstPtr &l_image_msg,
    const sensor_msgs::ImageConstPtr &r_image_msg)
{

    //Convert to CV format MONO 8bit
    cv_bridge::CvImageConstPtr left_raw_ptr  = cv_bridge::toCvShare (l_image_msg, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImageConstPtr right_raw_ptr = cv_bridge::toCvShare (r_image_msg, sensor_msgs::image_encodings::MONO8);

    if (rectifyImages_)
    {
        cv::Mat left_rect;
        cv::Mat right_rect;

        left_model_.rectifyImage (left_raw_ptr->image, left_rect, cv::INTER_LINEAR);
        right_model_.rectifyImage (right_raw_ptr->image, right_rect, cv::INTER_LINEAR);
        publishRectifiedImages(left_rect, right_rect, l_image_msg, r_image_msg);
        processRectified(left_rect, right_rect, l_image_msg);
    }
    else
    {
        processRectified(left_raw_ptr->image, right_raw_ptr->image, l_image_msg);
    }


}

void TegraStereoProc::publishRectifiedImages (const cv::Mat &left_rect,
        const cv::Mat &right_rect,
        const sensor_msgs::ImageConstPtr &l_image_msg,
        const sensor_msgs::ImageConstPtr &r_image_msg)
{


    if(pub_rect_left_.getNumSubscribers() > 0 || true)
    {
	NODELET_INFO("Publishing Left rect image");
        sensor_msgs::ImagePtr left_rect_msg = cv_bridge::CvImage (l_image_msg->header, l_image_msg->encoding, left_rect).toImageMsg();
        if(out_left_frame_id.length() > 0)
        {
            left_rect_msg->header.frame_id = out_left_frame_id;
        }
        mCameraInfoLeft_.header = left_rect_msg->header;
        pub_rect_left_.publish (left_rect_msg, boost::make_shared<sensor_msgs::CameraInfo>(mCameraInfoLeft_));
    }

    if(pub_rect_right_.getNumSubscribers() >0 || true)
    {
        sensor_msgs::ImagePtr right_rect_msg = cv_bridge::CvImage (r_image_msg->header, r_image_msg->encoding, right_rect).toImageMsg();
        if(out_right_frame_id.length() > 0)
        {
            right_rect_msg->header.frame_id = out_right_frame_id;
        }
        mCameraInfoRight_.header = right_rect_msg->header;
        pub_rect_right_.publish(right_rect_msg, boost::make_shared<sensor_msgs::CameraInfo>(mCameraInfoRight_));
    }
}

bool TegraStereoProc::processRectified(const cv::Mat &left_rect_cv, const cv::Mat &right_rect_cv, const sensor_msgs::ImageConstPtr &leftImgPtr)
{
  // Do block matching to produce the disparity image
  if (
          pub_disparity_raw_.getNumSubscribers()>0 ||
          pub_disparity_.getNumSubscribers() >0 ||
          pub_points_.getNumSubscribers() > 0 ||
          pub_points2_.getNumSubscribers() >0 ||
          pub_depth_Image_.getNumSubscribers() > 0
          )
  {

    float elapsed_time_ms;
    // Compute disparity CUDA
    cv::Mat disparity_raw = compute_disparity_method (left_rect_cv, right_rect_cv, &elapsed_time_ms);
    
    // timing
    elapsed_time_ms_acc_ += elapsed_time_ms;
    elapsed_time_counter_++;

    //Print average every 1000 frames
    if(elapsed_time_counter_ %1000 == 0)
    {
        elapsed_time_ms_acc_ = 1000000/elapsed_time_ms_acc_;
        NODELET_INFO ("Disparity computation at %f [fps] (1000 samples avg);", static_cast<double>(elapsed_time_ms_acc_));
        elapsed_time_ms_acc_ = 0.0;
    }

    // filter disparity ARM (remove noisy measurments)
    const int sigma = 5;
    cv::Mat disp_edge;			cv::Laplacian (disparity_raw, disp_edge, CV_16S, 5);
    cv::Mat disp_abs;			cv::convertScaleAbs(disp_edge, disp_abs);
    cv::Mat disp_smt;			cv::GaussianBlur(disp_abs, disp_smt, cv::Size(sigma*5, sigma*5), sigma, sigma);
    					disp_smt.convertTo(disp_smt, CV_8U);
    cv::Mat disparity_edge_threshold;	cv::threshold (disp_smt, disparity_edge_threshold, 40, 255, cv::THRESH_BINARY_INV);
    cv::Mat disparity_filtered;		cv::bitwise_and (disparity_raw, disparity_edge_threshold, disparity_filtered, disparity_edge_threshold);

    //publish raw disparity output in pixels
    if(pub_disparity_raw_.getNumSubscribers() >0)
    {
        sensor_msgs::ImagePtr raw_disp_msg = cv_bridge::CvImage (leftImgPtr->header, sensor_msgs::image_encodings::MONO8, disparity_filtered).toImageMsg();
        if(out_left_frame_id.length() > 0)
        {
            raw_disp_msg->header.frame_id = out_left_frame_id;
        }
        mCameraInfoLeft_.header = leftImgPtr->header;
        pub_disparity_raw_.publish (raw_disp_msg, boost::make_shared<sensor_msgs::CameraInfo>(mCameraInfoLeft_));
    }

    //scale the disparity and publish
    stereo_msgs::DisparityImagePtr disparity_msgPtr = boost::make_shared<stereo_msgs::DisparityImage>();
    processDisparity (disparity_filtered, leftImgPtr->header, disparity_msgPtr);

    if(pub_disparity_.getNumSubscribers()>0)
    {
        pub_disparity_.publish (disparity_msgPtr);
    }

    if(pub_depth_Image_.getNumSubscribers() >0)
    {
        sensor_msgs::ImagePtr imgPtr = processDepthImage(disparity_msgPtr);
        mCameraInfoLeft_.header = imgPtr->header;
        pub_depth_Image_.publish (imgPtr, boost::make_shared<sensor_msgs::CameraInfo>(mCameraInfoLeft_));
    }

    // Project disparity image to 3d point cloud
    if (pub_points_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloudPtr points = boost::make_shared<sensor_msgs::PointCloud>();
        processPoints(disparity_msgPtr, left_rect_cv, leftImgPtr->encoding, points);
        pub_points_.publish(points);
    }

    // Project disparity image to 3d point cloud
    if (pub_points2_.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2Ptr points2 = boost::make_shared<sensor_msgs::PointCloud2>();
        processPoints2(disparity_msgPtr, left_rect_cv, leftImgPtr->encoding, points2);
        pub_points2_.publish(points2);
    }

  }

  return true;
}

void TegraStereoProc::processDisparity (const cv::Mat &disparity, const std_msgs::Header &header, stereo_msgs::DisparityImagePtr &disparityMsgPtr)
{

    // convert to ros disparity message
    static const double DPP = 1.0; // disparities per pixel
    static const double inv_dpp = 1.0 / DPP;


    disparityMsgPtr->header = disparityMsgPtr->image.header = header;
    if(out_left_frame_id.length() > 0)
    {
        disparityMsgPtr->header.frame_id = out_left_frame_id;
        disparityMsgPtr->image.header.frame_id = out_left_frame_id;
    }

    auto &dimage = disparityMsgPtr->image;
    dimage.height = left_model_.cameraInfo().height;
    dimage.width = left_model_.cameraInfo().width;
    dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    dimage.step = dimage.width * sizeof (float);
    dimage.data.resize (dimage.step * dimage.height);
    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

    // Stereo parameters
    disparityMsgPtr->f = stereo_model_.right().fx();
    disparityMsgPtr->T = stereo_model_.baseline();

    // Disparity search range
    disparityMsgPtr->min_disparity = 0;
    disparityMsgPtr->max_disparity = 127;
    disparityMsgPtr->delta_d = inv_dpp;

    // We convert from fixed-point to float disparity and also adjust for any
    // x-offset between
    // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
    disparity.convertTo (dmat, dmat.type(), inv_dpp,
                         - (stereo_model_.left().cx() - stereo_model_.right().cx()));

    ROS_ASSERT (dmat.data == &dimage.data[0]);
}


inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return (((pt[2]!=image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2])) && !std::isnan(pt[2])) &&
		pt[2] <= 5.0) && pt[2] >= 0.5;
}

void TegraStereoProc::processPoints(const stereo_msgs::DisparityImageConstPtr& disparityMsgPrt,
                                    const cv::Mat& color, const std::string& encoding,
                                    sensor_msgs::PointCloudPtr &pointsMsgPtr) const
{
    pointsMsgPtr->header = disparityMsgPrt->header;
    if(out_left_frame_id.length() > 0)
    {
        pointsMsgPtr->header.frame_id = out_left_frame_id;
    }
  // Calculate dense point cloud
  const sensor_msgs::Image& dimage = disparityMsgPrt->image;
  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  stereo_model_.projectDisparityImageTo3d(dmat, dense_points_, true);

  // Fill in sparse point cloud message
  pointsMsgPtr->points.resize(0);
  pointsMsgPtr->channels.resize(3);
  pointsMsgPtr->channels[0].name = "rgb";
  pointsMsgPtr->channels[0].values.resize(0);
  pointsMsgPtr->channels[1].name = "u";
  pointsMsgPtr->channels[1].values.resize(0);
  pointsMsgPtr->channels[2].name = "v";
  pointsMsgPtr->channels[2].values.resize(0);

  for (int32_t u = 0; u < dense_points_.rows; ++u) {
    for (int32_t v = 0; v < dense_points_.cols; ++v) {
      if (isValidPoint(dense_points_(u,v))) {
        // x,y,z
        geometry_msgs::Point32 pt;
        pt.x = dense_points_(u,v)[0];
        pt.y = dense_points_(u,v)[1];
        pt.z = dense_points_(u,v)[2];
        pointsMsgPtr->points.push_back(pt);
        // u,v
        pointsMsgPtr->channels[1].values.push_back(u);
        pointsMsgPtr->channels[2].values.push_back(v);
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  pointsMsgPtr->channels[0].values.reserve(pointsMsgPtr->points.size());
  if (encoding == enc::MONO8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v) {
        if (isValidPoint(dense_points_(u,v))) {
          uint8_t g = color.at<uint8_t>(u,v);
          int32_t rgb = (g << 16) | (g << 8) | g;
          pointsMsgPtr->channels[0].values.push_back(*(float*)(&rgb));
        }
      }
    }
  }
  else if (encoding == enc::RGB8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v) {
        if (isValidPoint(dense_points_(u,v))) {
          const cv::Vec3b& rgb = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          pointsMsgPtr->channels[0].values.push_back(*(float*)(&rgb_packed));
        }
      }
    }
  }
  else if (encoding == enc::BGR8) {
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
      for (int32_t v = 0; v < dense_points_.cols; ++v) {
        if (isValidPoint(dense_points_(u,v))) {
          const cv::Vec3b& bgr = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          pointsMsgPtr->channels[0].values.push_back(*(float*)(&rgb_packed));
        }
      }
    }
  }
  else {
    ROS_WARN("Could not fill color channel of the point cloud, unrecognized encoding '%s'", encoding.c_str());
  }
}

void TegraStereoProc::processPoints2(const stereo_msgs::DisparityImageConstPtr& disparityMsgPrt,
                                     const cv::Mat& color, const std::string& encoding,
                                     sensor_msgs::PointCloud2Ptr &pointsMsgPrt) const
{
  // Calculate dense point cloud
  const sensor_msgs::Image& dimage = disparityMsgPrt->image;
  pointsMsgPrt->header = disparityMsgPrt->header;
  if(out_left_frame_id.length() > 0)
  {
      pointsMsgPrt->header.frame_id = out_left_frame_id;
  }


  const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
  stereo_model_.projectDisparityImageTo3d(dmat, dense_points_, true);

  int32_t margin_x = 100; // pixels at the edge of the frame to avoid
  int32_t margin_y = 40;

  // Fill in sparse point cloud message
  pointsMsgPrt->height = dense_points_.rows-margin_y*2;
  pointsMsgPrt->width  = dense_points_.cols-margin_x*2;
  pointsMsgPrt->fields.resize (4);
  pointsMsgPrt->fields[0].name = "x";
  pointsMsgPrt->fields[0].offset = 0;
  pointsMsgPrt->fields[0].count = 1;
  pointsMsgPrt->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  pointsMsgPrt->fields[1].name = "y";
  pointsMsgPrt->fields[1].offset = 4;
  pointsMsgPrt->fields[1].count = 1;
  pointsMsgPrt->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  pointsMsgPrt->fields[2].name = "z";
  pointsMsgPrt->fields[2].offset = 8;
  pointsMsgPrt->fields[2].count = 1;
  pointsMsgPrt->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  pointsMsgPrt->fields[3].name = "rgb";
  pointsMsgPrt->fields[3].offset = 12;
  pointsMsgPrt->fields[3].count = 1;
  pointsMsgPrt->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  //points->is_bigendian = false; ???
  pointsMsgPrt->point_step = 16;
  pointsMsgPrt->row_step = pointsMsgPrt->point_step * pointsMsgPrt->width;
  pointsMsgPrt->data.resize (pointsMsgPrt->row_step * pointsMsgPrt->height);
  pointsMsgPrt->is_dense = false; // there may be invalid points

  int i = 0;
  for (int32_t u = margin_y; u < dense_points_.rows-margin_y; ++u) {
    for (int32_t v = margin_x; v < dense_points_.cols-margin_x; ++v, ++i) {
      if (isValidPoint(dense_points_(u,v))) {
        // x,y,z,rgba
        memcpy (&pointsMsgPrt->data[i * pointsMsgPrt->point_step + 0], &dense_points_(u,v)[0], sizeof (float));
        memcpy (&pointsMsgPrt->data[i * pointsMsgPrt->point_step + 4], &dense_points_(u,v)[1], sizeof (float));
        memcpy (&pointsMsgPrt->data[i * pointsMsgPrt->point_step + 8], &dense_points_(u,v)[2], sizeof (float));
      }
    }
  }

  // Fill in color
  namespace enc = sensor_msgs::image_encodings;
  i = 0;
  // mono8	
  if (encoding == enc::MONO8) {
    for (int32_t u = margin_y; u < dense_points_.rows-margin_y; ++u) {
      for (int32_t v = margin_x; v < dense_points_.cols-margin_x; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
	  uint8_t g = color.at<uint8_t>(u,v);
          int32_t rgb = (g << 16) | (g << 8) | g;
          memcpy (&pointsMsgPrt->data[i * pointsMsgPrt->point_step + 12], &rgb, sizeof (int32_t));
        }
      }
    }
  //RGB8
  } else if (encoding == enc::RGB8) {
    for (int32_t u = margin_y; u < dense_points_.rows-margin_y; ++u) {
      for (int32_t v = margin_x; v < dense_points_.cols-margin_x; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
	  const cv::Vec3b& rgb = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
          memcpy (&pointsMsgPrt->data[i * pointsMsgPrt->point_step + 12], &rgb_packed, sizeof (int32_t));
        }
      }
    }
  //BGR8
  } else if (encoding == enc::BGR8) {
    for (int32_t u = margin_y; u < dense_points_.rows-margin_y; ++u) {
      for (int32_t v = margin_x; v < dense_points_.cols-margin_x; ++v, ++i) {
        if (isValidPoint(dense_points_(u,v))) {
	  const cv::Vec3b& bgr = color.at<cv::Vec3b>(u,v);
          int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
          memcpy (&pointsMsgPrt->data[i * pointsMsgPrt->point_step + 12], &rgb_packed, sizeof (int32_t));
        }
      }
    }
  // otherwise
  } else {
	  ROS_WARN("Could not fill color channel of the point cloud, unrecognized encoding '%s'", encoding.c_str());
  }
}


sensor_msgs::ImagePtr TegraStereoProc::processDepthImage(const stereo_msgs::DisparityImageConstPtr& disparityMsgPrt) const
{
    // Calculate dense point cloud
    const sensor_msgs::Image& dimage = disparityMsgPrt->image;
    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
    stereo_model_.projectDisparityImageTo3d(dmat, dense_points_, true);


    cv::Mat_<float> cvDepthImage(dense_points_.rows, dense_points_.cols);

    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    //copy the Z value to depth matrix
    for (int32_t u = 0; u < dense_points_.rows; ++u)
    {
        for (int32_t v = 0; v < dense_points_.cols; ++v)
        {
            if (isValidPoint(dense_points_(u,v)))
            {
                cvDepthImage(u,v) = dense_points_(u,v)[2];
            }
            else
            {
                cvDepthImage(u,v) = bad_point;
            }
        }
    }
    sensor_msgs::ImagePtr depthImagePtr = cv_bridge::CvImage (disparityMsgPrt->header, sensor_msgs::image_encodings::TYPE_32FC1, cvDepthImage).toImageMsg();
    if(out_left_frame_id.length() > 0)
    {
        depthImagePtr->header.frame_id = out_left_frame_id;
    }

    return depthImagePtr;
}

}  // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (tegra_stereo::TegraStereoProc, nodelet::Nodelet)
