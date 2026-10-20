#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

// Mensaje custom con ID
#include "aruco_detector/TimingLogger.hpp"
#include <aruco_detector/ArucoMarkers.h>

class ArUcoDetector {
public:
  ArUcoDetector() : it(nh), have_cam_info(false) {
    ros::NodeHandle pnh("~");
    pnh.param("camera_offset_x", camera_offset_x_, 0.884938213);  // en m
    pnh.param("camera_offset_y", camera_offset_y_, -0.184078095); // en m
    pnh.param("camera_pitch", camera_pitch_, 0.741557322);        // en radianes
    pnh.param("image_show", image_show_, false);
    pnh.param("marker_length", marker_length_, 0.050f);

    // Suscripciones
    image_sub = it.subscribe("/camera/color/image_raw", 1,
                             &ArUcoDetector::imageCallback, this);
    depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1,
                             &ArUcoDetector::depthCallback, this);
    cam_info_sub = nh.subscribe("/camera/color/camera_info", 1,
                                &ArUcoDetector::camInfoCb, this);

    // Publicadores
    aruco_markers =
        nh.advertise<aruco_detector::ArucoMarkers>("/aruco_markers", 10);

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  }

private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Subscriber depth_sub;
  ros::Subscriber cam_info_sub;

  ros::Publisher aruco_markers;
  tf::TransformBroadcaster br;

  cv::Mat cameraMatrix, distCoeffs;
  bool have_cam_info;

  cv::Ptr<cv::aruco::Dictionary> dictionary;
  float marker_length_;

  double camera_offset_x_;
  double camera_offset_y_;
  double camera_pitch_;
  bool image_show_;

  cv::Mat latest_depth;
  ros::Time latest_depth_stamp;

  void camInfoCb(const sensor_msgs::CameraInfoConstPtr &info) {
    cameraMatrix =
        (cv::Mat1d(3, 3) << info->K[0], info->K[1], info->K[2], info->K[3],
         info->K[4], info->K[5], info->K[6], info->K[7], info->K[8]);
    distCoeffs = cv::Mat(info->D).clone();
    have_cam_info = true;
  }

  void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr =
          cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      latest_depth = cv_ptr->image.clone();
      latest_depth_stamp = msg->header.stamp;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception (depth): %s", e.what());
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    if (!have_cam_info || latest_depth.empty()) {
      ROS_WARN_THROTTLE(2.0, "Esperando calibración y depth...");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    const double start_ms = TimingLogger::nowMs();
    cv::aruco::detectMarkers(gray, dictionary, corners, ids);
    const double elapsed_ms = TimingLogger::nowMs() - start_ms;

    TimingLogger::log("/home/sara/aruco_timing.csv", "aruco_detection",
                      elapsed_ms);

    if (ids.empty()) {
      if (image_show_) {
        cv::imshow("ArUco Detection", cv_ptr->image);
      }

      cv::waitKey(1);
      return;
    }

    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

    std::vector<cv::Point3f> objectPoints = {
        {-marker_length_ / 2.f, marker_length_ / 2.f, 0},
        {marker_length_ / 2.f, marker_length_ / 2.f, 0},
        {marker_length_ / 2.f, -marker_length_ / 2.f, 0},
        {-marker_length_ / 2.f, -marker_length_ / 2.f, 0}};

    aruco_detector::ArucoMarkers markers;
    markers.header = msg->header;

    for (size_t i = 0; i < ids.size(); i++) {
      cv::Vec3d rvec, tvec;
      cv::solvePnP(objectPoints, corners[i], cameraMatrix, distCoeffs, rvec,
                   tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

      // Centro del marcador en píxeles
      cv::Point2f center(0, 0);
      for (auto &p : corners[i])
        center += p;
      center *= 0.25;

      int u = static_cast<int>(center.x);
      int v = static_cast<int>(center.y);
      double depth_m = NAN;
      if (u >= 0 && v >= 0 && u < latest_depth.cols && v < latest_depth.rows) {
        uint16_t d = latest_depth.at<uint16_t>(v, u);
        if (d > 0)
          depth_m = d * 0.001; // mm → m
      }

      double dist_pnp = cv::norm(tvec);

      // ROS_INFO("Marker %d: PnP=%.3f m  Depth=%.3f m (px=%d,%d)", ids[i],
      // dist_pnp, depth_m, u, v);

      // Publicar mensaje PoseWithId
      aruco_detector::PoseWithId data;
      data.id = ids[i];
      // Distancia 3D cámara–marcador (por si la quieres loguear)
      // Coordenadas en cámara
      double x_cam = tvec[0]; // derecha
      double y_cam = tvec[1]; // abajo (no la usamos en 2D)
      double z_cam = tvec[2]; // hacia delante

      double x_forward =
          z_cam * std::cos(camera_pitch_) - y_cam * std::sin(camera_pitch_);

      double x_from_camera = x_forward;
      double y_from_camera = -x_cam;

      double x_base = camera_offset_x_ + x_from_camera;
      double y_base = camera_offset_y_ + y_from_camera;

      double range = std::hypot(x_base, y_base);
      double bearing = std::atan2(y_base, x_base);

      // Publicar en el mensaje
      data.id = ids[i];
      data.distance = range;
      data.theta = bearing;
      data.phi = acos(
          z_cam /
          dist_pnp); // 0.0;  // si el mensaje lo pide pero el UKF no lo usa
      markers.markers.emplace_back(data);
    }
    aruco_markers.publish(markers);

    if (image_show_) {
      cv::imshow("ArUco Detection", cv_ptr->image);
    }
    cv::waitKey(1);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_detector");
  ArUcoDetector detector;
  ros::spin();
  return 0;
}
