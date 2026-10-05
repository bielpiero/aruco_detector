#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

// Mensaje custom con ID
#include <aruco_detector/ArucoMarkers.h>

class ArUcoDetector {
public:
    ArUcoDetector() :
        it(nh),
        have_cam_info(false)
    {
        ros::NodeHandle pnh("~");
        pnh.param("theta_offset", theta_offset_, 0.0);  // en radianes

        // Suscripciones
        image_sub = it.subscribe("/camera/color/image_raw", 1, &ArUcoDetector::imageCallback, this);
        depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ArUcoDetector::depthCallback, this);
        cam_info_sub = nh.subscribe("/camera/color/camera_info", 1, &ArUcoDetector::camInfoCb, this);

        // Publicadores
        aruco_markers = nh.advertise<aruco_detector::ArucoMarkers>("/aruco_markers", 10);

        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        markerLength = 0.059; // metros

        ROS_INFO("ArUcoDetector: theta_offset_ (pitch) = %.3f rad", theta_offset_);
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
    double markerLength;

    double theta_offset_;

    cv::Mat latest_depth;
    ros::Time latest_depth_stamp;

    void camInfoCb(const sensor_msgs::CameraInfoConstPtr& info) {
        cameraMatrix = (cv::Mat1d(3,3) <<
            info->K[0], info->K[1], info->K[2],
            info->K[3], info->K[4], info->K[5],
            info->K[6], info->K[7], info->K[8]);
        distCoeffs = cv::Mat(info->D).clone();
        have_cam_info = true;
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            latest_depth = cv_ptr->image.clone();
            latest_depth_stamp = msg->header.stamp;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (depth): %s", e.what());
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!have_cam_info || latest_depth.empty()) {
            ROS_WARN_THROTTLE(2.0, "Esperando calibración y depth...");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, dictionary, corners, ids);

        if (ids.empty()) {
            cv::imshow("ArUco Detection", cv_ptr->image);
            cv::waitKey(1);
            return;
        }

        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

        std::vector<cv::Point3f> objectPoints = {
            {-markerLength/2.f,  markerLength/2.f, 0},
            { markerLength/2.f,  markerLength/2.f, 0},
            { markerLength/2.f, -markerLength/2.f, 0},
            {-markerLength/2.f, -markerLength/2.f, 0}
        };
        
        aruco_detector::ArucoMarkers markers;
        //markers.header.stamp = msg->header.stamp;
        //markers.header.frame_id = "camera_color_optical_frame";
        for (size_t i = 0; i < ids.size(); i++) {
            cv::Vec3d rvec, tvec;
            cv::solvePnP(objectPoints, corners[i], cameraMatrix, distCoeffs,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

            // Centro del marcador en píxeles
            cv::Point2f center(0,0);
            for (auto &p : corners[i]) center += p;
            center *= 0.25;

            int u = static_cast<int>(center.x);
            int v = static_cast<int>(center.y);
            double depth_m = NAN;
            if (u >= 0 && v >= 0 && u < latest_depth.cols && v < latest_depth.rows) {
                uint16_t d = latest_depth.at<uint16_t>(v, u);
                if (d > 0) depth_m = d * 0.001; // mm → m
            }

            double dist_pnp = cv::norm(tvec);

            //ROS_INFO("Marker %d: PnP=%.3f m  Depth=%.3f m (px=%d,%d)", ids[i], dist_pnp, depth_m, u, v);

            // Publicar mensaje PoseWithId
            aruco_detector::PoseWithId data;
            data.id = ids[i];
            // data.distance = depth_m;//dist_pnp;
            // data.theta = atan2(tvec[1], tvec[0]);
            // data.phi = acos(tvec[2] / dist_pnp);
            // Distancia correcta en el espacio 3D de la cámara
            // double dist = cv::norm(tvec);

            // // Proyección al plano horizontal del robot/cámara
            // double dx = tvec[0];
            // double dy = tvec[2];     // OJO: RealSense tiene z hacia adelante, y hacia abajo

            // // Convertir al plano X-Y del robot (depende de mi TF real)
            // double range = sqrt(dx*dx + dy*dy);
            // double bearing = atan2(dy, dx);

            // // Publicar solo lo que el UKF espera
            // data.id = ids[i];
            // data.distance = range;
            // data.theta = bearing;
            // Distancia 3D cámara–marcador (por si la quieres loguear)
            double dist_3d = cv::norm(tvec);

            // Coordenadas en cámara
            double x_cam = tvec[0]; // derecha
            double y_cam = tvec[1]; // abajo (no la usamos en 2D)
            double z_cam = tvec[2]; // hacia delante
            
            double pitch = theta_offset_; // lee del parámetro ROS

            // 1. Compensar inclinación vertical de la cámara
            double x_forward =  z_cam * cos(pitch) - y_cam * sin(pitch);
            double z_up      =  z_cam * sin(pitch) + y_cam * cos(pitch);

            // Proyección al plano del robot (x hacia delante, y hacia la izquierda)
            double x_robot = x_forward;     // delante
            double y_robot = -x_cam;    // izquierda

            // Medidas rango + bearing que espera el UKF
            double range   = std::sqrt(x_robot*x_robot + y_robot*y_robot);
            double bearing = std::atan2(y_robot, x_robot);  // relativo al eje delantero

            // Publicar en el mensaje
            data.id       = ids[i];
            data.distance = range;
            data.theta    = bearing;
            data.phi = acos(z_cam / dist_pnp);//0.0;  // si el mensaje lo pide pero el UKF no lo usa
            markers.markers.emplace_back(data);
            
        }
        aruco_markers.publish(markers);

        cv::imshow("ArUco Detection", cv_ptr->image);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector");
    ArUcoDetector detector;
    ros::spin();
    return 0;
}
