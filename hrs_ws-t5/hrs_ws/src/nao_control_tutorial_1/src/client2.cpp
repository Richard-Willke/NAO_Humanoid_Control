#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nao_control_tutorial_1/MoveJoints.h>

class CameraClient {

    ros::Subscriber image_subscriber;
    ros::Publisher image_publisher;
    ros::ServiceClient service_client;
    cv::Mat img_msg;
    cv_bridge::CvImagePtr cv_ptr;
    cv::Point2f aruco_center;

public:
    CameraClient(ros::NodeHandle& nh) {
        // Initialize the subscriber, publisher, and service client
        image_subscriber = nh.subscribe("/nao_robot/camera/top/camera/image_raw", 10, &CameraClient::image_callback_top, this);
        image_publisher = nh.advertise<sensor_msgs::Image>("/nao_custom_image", 10);
        service_client = nh.serviceClient<nao_control_tutorial_1::MoveJoints>("move_joints");
        aruco_center = cv::Point2f(-1.0f, -1.0f); // Initialize with invalid coordinates
    }

    void arucoFunction() {
        // Camera matrix and distortion coefficients
        double camera_matrix_data[9] = {551.543059, 0.0, 327.382898,
                                        0.0, 553.736023, 225.02638,
                                        0.0, 0.0, 1.0};
        cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_data);

        double dist_coeffs_data[5] = {-0.066494, 0.095481, -0.000279, 0.002292, 0.0};
        cv::Mat dist_coeffs = cv::Mat(1, 5, CV_64F, dist_coeffs_data);

        // Load ArUco dictionary and parameters
        cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        cv::Mat gray;
        cv::cvtColor(img_msg, gray, cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point2f> > corners;
        std::vector<int> ids;
        cv::aruco::detectMarkers(gray, aruco_dict, corners, ids, parameters);

        if (!ids.empty()) {
            float center_x = 0.0f, center_y = 0.0f;
            aruco_center = cv::Point2f(center_x / corners.size(), center_y / corners.size());
            cv::circle(img_msg, aruco_center, 20, cv::Scalar( 255, 0, 0 ),5);

            std::cout<< " Center diff is " << (center_x / corners.size() - 320 ) << (center_y / corners.size() - 240)<< std::endl; 
        }

        // Draw detected markers
        cv::aruco::drawDetectedMarkers(img_msg, corners, ids);
        cv::imshow("Aruco Marker", img_msg);
        cv::waitKey(1);
    }

    void image_callback_top(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            img_msg = cv_ptr->image;
            arucoFunction();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void call_service() {
        std::cout << "ar center: " << aruco_center.x << std::endl; 
        if (aruco_center.x != -1.0f && aruco_center.y != -1.0f) {
            nao_control_tutorial_1::MoveJoints srv;
            srv.request.target_pose.push_back(aruco_center.x);
            srv.request.target_pose.push_back(aruco_center.y);
            srv.request.set_angles = false;

            if (service_client.call(srv)) {
                ROS_INFO("Service call successful.");
            } else {
                ROS_ERROR("Service call failed.");
            }
        }
    }
};

/*
int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_client");
    ros::NodeHandle nh;

    CameraClient camera_client(nh);
    camera_client.call_service();

    ros::spin();
    return 0;
}
*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    CameraClient client_(nh);

    ros::Rate loop_rate(5); // Set loop rate to 5 Hz (adjust as needed)

    while (ros::ok()) {
        try {
            client_.call_service(); // Call the service
        } catch (...) {
            // Catch any exceptions and continue
        }

        ros::spinOnce(); // Process any pending callbacks
        loop_rate.sleep(); // Sleep to maintain loop rate
    }

    cv::destroyAllWindows(); // Close OpenCV windows on exit
    return 0;
}
