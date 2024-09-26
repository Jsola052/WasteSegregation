#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/point_types_conversion.h>
#include <vector>

class GreenBallFilter : public rclcpp::Node
{
public:
    GreenBallFilter() : Node("green_ball_filter")
    {
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
            std::bind(&GreenBallFilter::pointCloudCallback, this, std::placeholders::_1));
        
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/green_ball_points", rclcpp::QoS(10).reliable());

        green_ball_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/green_ball_position", rclcpp::QoS(10).reliable());
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        // Create a cloud for filtered green points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZHSV point_hsv;
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], point_hsv);

            // Adjusted for green detection in HSV color space with updated ranges to filter out dark green
            if (point_hsv.h > 60 && point_hsv.h < 180 && point_hsv.s > 0.5 && point_hsv.v > 0.4) 
            {
                inliers->indices.push_back(i); 
            }
        }

        // Extract the green points
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false); 
        extract.filter(*green_cloud);

        // Convert the filtered PCL point cloud back to ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*green_cloud, output_msg);
        output_msg.header = msg->header;

        point_cloud_publisher_->publish(output_msg);

        if (!green_cloud->points.empty())
        {
            for (const auto& point : green_cloud->points)
            {
                if (!isPointAlreadyPublished(point))
                {
                    published_points_.push_back(point);
                    RCLCPP_INFO(this->get_logger(), "Green ball detected at: x: %.2f, y: %.2f, z: %.2f", point.x, point.y, point.z);

                    geometry_msgs::msg::PointStamped point_stamped;
                    point_stamped.header = msg->header; 
                    point_stamped.point.x = point.x;
                    point_stamped.point.y = point.y;
                    point_stamped.point.z = point.z;

                    green_ball_publisher_->publish(point_stamped);
                }
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No green points found.");
        }
    }

    bool isPointAlreadyPublished(const pcl::PointXYZRGB &point)
    {
        const double threshold = 0.041;  // Slightly increased threshold to account for noise

        for (const auto& published_point : published_points_)
        {
            double distance = std::sqrt(std::pow(point.x - published_point.x, 2) +
                                        std::pow(point.y - published_point.y, 2) +
                                        std::pow(point.z - published_point.z, 2));

            if (distance < threshold)
            {
                return true; 
            }
        }
        return false; 
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr green_ball_publisher_;

    std::vector<pcl::PointXYZRGB> published_points_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GreenBallFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
