#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class BallDetector : public rclcpp::Node
{
public:
    BallDetector()
    : Node("ball_detector"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing BallDetector node...");

        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("ball_coordinates", 10);
        RCLCPP_INFO(this->get_logger(), "Publisher created for 'ball_coordinates'.");

        lower_green_ = cv::Scalar(35, 100, 100);
        upper_green_ = cv::Scalar(85, 255, 255);

        color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&BallDetector::image_callback, this, std::placeholders::_1));
        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", 10, std::bind(&BallDetector::depth_callback, this, std::placeholders::_1));
        color_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10, std::bind(&BallDetector::color_info_callback, this, std::placeholders::_1));
        depth_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/depth/camera_info", 10, std::bind(&BallDetector::depth_info_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscriptions created for camera topics.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received color image");
        try {
            color_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            RCLCPP_INFO(this->get_logger(), "Color image converted successfully");
            detect_ball();
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received depth image");
        try {
            depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            RCLCPP_INFO(this->get_logger(), "Depth image converted successfully");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received color camera info");
        // Process camera color info if needed
    }

    void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received depth camera info");
        depth_intrinsics_["width"] = msg->width;
        depth_intrinsics_["height"] = msg->height;
        depth_intrinsics_["ppx"] = msg->k[2];
        depth_intrinsics_["ppy"] = msg->k[5];
        depth_intrinsics_["fx"] = msg->k[0];
        depth_intrinsics_["fy"] = msg->k[4];
    }

    void detect_ball()
    {
        RCLCPP_INFO(this->get_logger(), "Starting ball detection");

        if (color_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No color image received, skipping ball detection.");
            return;
        }

        cv::Mat hsv_image;
        cv::cvtColor(color_image_, hsv_image, cv::COLOR_BGR2HSV);
        cv::Mat mask_green;
        cv::inRange(hsv_image, lower_green_, upper_green_, mask_green);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_green, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        RCLCPP_INFO(this->get_logger(), "Found %lu green contours", contours.size());

        int detected_balls = 0;
        int published_balls = 0;

        for (const auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500) {
                double epsilon = 0.02 * cv::arcLength(contour, true);
                std::vector<cv::Point> approx_polygon;
                cv::approxPolyDP(contour, approx_polygon, epsilon, true);
                cv::Moments M = cv::moments(approx_polygon);
                if (M.m00 != 0) {
                    int cX = static_cast<int>(M.m10 / M.m00);
                    int cY = static_cast<int>(M.m01 / M.m00);
                    cv::Point center(cX, cY);
                    detected_balls++;

                    RCLCPP_INFO(this->get_logger(), "Ball detected at pixel coordinates (%d, %d)", cX, cY);

                    if (!depth_image_.empty() && !depth_intrinsics_.empty()) {
                        if (center.y >= 0 && center.y < depth_image_.rows && center.x >= 0 && center.x < depth_image_.cols) {
                            // Assuming the depth value is in millimeters
                            float depth_value_mm = depth_image_.at<float>(center.y, center.x);
                            float depth_value = depth_value_mm / 1000.0f; // Convert to meters

                            if (depth_value > 0) {
                                // Calculate the 3D coordinates in the camera frame
                                float x = (center.x - depth_intrinsics_["ppx"]) * depth_value / depth_intrinsics_["fx"];
                                float y = (center.y - depth_intrinsics_["ppy"]) * depth_value / depth_intrinsics_["fy"];
                                float z = depth_value;

                                RCLCPP_INFO(this->get_logger(), "Depth value (in meters): %f", depth_value);
                                RCLCPP_INFO(this->get_logger(), "Calculated coordinates: x=%f, y=%f, z=%f", x, y, z);

                                geometry_msgs::msg::PointStamped point_stamped;
                                point_stamped.header.stamp = this->get_clock()->now();
                                point_stamped.header.frame_id = "camera_depth_optical_frame";
                                point_stamped.point.x = x;
                                point_stamped.point.y = y;
                                point_stamped.point.z = z;

                                // Perform the transformation using tf2
                                try {
                                    geometry_msgs::msg::TransformStamped transform_stamped;
                                    transform_stamped = tf_buffer_.lookupTransform("base_link", "camera_depth_optical_frame", tf2::TimePointZero);

                                    RCLCPP_INFO(this->get_logger(), "Transforming using: translation x=%f, y=%f, z=%f; rotation w=%f, x=%f, y=%f, z=%f", 
                                                transform_stamped.transform.translation.x, 
                                                transform_stamped.transform.translation.y, 
                                                transform_stamped.transform.translation.z,
                                                transform_stamped.transform.rotation.w,
                                                transform_stamped.transform.rotation.x,
                                                transform_stamped.transform.rotation.y,
                                                transform_stamped.transform.rotation.z);

                                    geometry_msgs::msg::PointStamped transformed_point;
                                    tf2::doTransform(point_stamped, transformed_point, transform_stamped);

                                    RCLCPP_INFO(this->get_logger(), "Transformed coordinates: x=%f, y=%f, z=%f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

                                    publisher_->publish(transformed_point);
                                    published_balls++;
                                } catch (tf2::TransformException &ex) {
                                    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
                                }
                            }
                        }
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Detected balls: %d, Published balls: %d", detected_balls, published_balls);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_subscription_;

    cv::Mat color_image_;
    cv::Mat depth_image_;
    cv::Scalar lower_green_;
    cv::Scalar upper_green_;
    std::map<std::string, float> depth_intrinsics_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BallDetector>();
    RCLCPP_INFO(node->get_logger(), "BallDetector node started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
