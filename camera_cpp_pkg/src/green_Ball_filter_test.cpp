#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/point_types_conversion.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>
#include <vector>
#include <std_msgs/msg/string.hpp>

class GreenBallFilter : public rclcpp::Node
{
public:
    GreenBallFilter() : Node("green_ball_filter")
    {
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
            std::bind(&GreenBallFilter::cloudCallback, this, std::placeholders::_1));

        cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/ur_tools", 10,
            [this](const std_msgs::msg::String::SharedPtr cmd){cmdCallback(cmd);});
        
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/green_ball_points", rclcpp::QoS(10).reliable());

        green_ball_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/green_ball_position", rclcpp::QoS(10).reliable());
    }

private:

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud = msg; 
    }
    void cmdCallback(const std_msgs::msg::String::SharedPtr cmd){
        if (cmd->data == "Standby") {
        pointCloudCallback(cloud);
    } else {
        cloud.reset();  // Reset the shared pointer
    }
    }
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        // Create a cloud for filtered green points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Filter green points
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZHSV point_hsv;
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], point_hsv);

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

        // Cluster the green points
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(green_cloud);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.05); // Distance threshold for clustering (adjust as needed)
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(green_cloud);
        ec.extract(cluster_indices);

        // For each cluster (representing a ball), calculate and publish the centroid
        for (const auto& cluster : cluster_indices)
        {
            Eigen::Vector3f centroid(0.0, 0.0, 0.0);
            size_t point_count = cluster.indices.size();

            // Sum up the coordinates of the cluster
            for (const auto& index : cluster.indices)
            {
                const auto& point = green_cloud->points[index];
                centroid[0] += point.x;
                centroid[1] += point.y;
                centroid[2] += point.z;
            }

            // Calculate the centroid
            centroid /= static_cast<float>(point_count);

            // Create a pcl point for centroid checking
            pcl::PointXYZRGB centroid_point;
            centroid_point.x = centroid[0];
            centroid_point.y = centroid[1];
            centroid_point.z = centroid[2];

            // if (!isPointAlreadyPublished(centroid_point))
            // {
                published_points_.push_back(centroid_point);

                // Publish the centroid of the current green ball
                geometry_msgs::msg::PointStamped point_stamped;
                point_stamped.header = msg->header;
                point_stamped.point.x = centroid[0];
                point_stamped.point.y = centroid[1];
                point_stamped.point.z = centroid[2];

                green_ball_publisher_->publish(point_stamped);

                RCLCPP_INFO(this->get_logger(), "Green ball centroid detected at: x: %.2f, y: %.2f, z: %.2f",
                            centroid[0], centroid[1], centroid[2]);
            // }
        }

        // Publish the filtered green points as a PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*green_cloud, output_msg);
        output_msg.header = msg->header;
        point_cloud_publisher_->publish(output_msg);
    }

    // bool isPointAlreadyPublished(const pcl::PointXYZRGB &point)
    // {
    //     const double threshold = 0.041;

    //     for (const auto& published_point : published_points_)
    //     {
    //         double distance = std::sqrt(std::pow(point.x - published_point.x, 2) +
    //                                     std::pow(point.y - published_point.y, 2) +
    //                                     std::pow(point.z - published_point.z, 2));

    //         if (distance < threshold)
    //         {
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr green_ball_publisher_;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

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
