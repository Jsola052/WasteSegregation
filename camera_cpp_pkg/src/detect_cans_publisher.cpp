#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class CanLocator : public rclcpp::Node
{
public:
  CanLocator()
  : Node("can_locator")
  {
    declare_parameter<std::string>("api_key", "");
    declare_parameter<std::string>(
      "endpoint",
      "https://fiuaiservice.com/api/cv/instance_segmentation/LLW"
    );
    get_parameter("api_key", api_key_);
    get_parameter("endpoint", endpoint_);

    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", 1,
      std::bind(&CanLocator::imageCallback, this, std::placeholders::_1)
    );
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", rclcpp::SensorDataQoS(),
      std::bind(&CanLocator::cloudCallback, this, std::placeholders::_1)
    );
    pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/can_position", 10
    );
  }

private:
  cv::Mat                             latest_image_;
  std_msgs::msg::Header               image_header_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
  std_msgs::msg::Header               cloud_header_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pos_pub_;

  std::string api_key_, endpoint_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      latest_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
      image_header_ = msg->header;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    processIfReady();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->height > 1 && cloud->width > 1) {
      latest_cloud_ = cloud;
      cloud_header_ = msg->header;
      processIfReady();
    }
  }

  void processIfReady()
  {
    if (latest_image_.empty() || !latest_cloud_) {
      return;
    }

    // --- encode to PNG
    std::vector<uchar> buf;
    cv::imencode(".png", latest_image_, buf);

    // --- call LLW API
    std::string resp = sendToAPI(buf);
    if (resp.empty()) {
      clearBuffers();
      return;
    }

    // --- parse JSON
    auto j = json::parse(resp);
    auto &masks = j["masks"];
    auto &confs = j["conf"];
    auto &cls   = j["classes"];

    int W = latest_image_.cols;
    int H = latest_image_.rows;

    // --- compute centroids & publish
    for (size_t i = 0; i < masks.size(); ++i) {
      double su = 0, sv = 0; int cnt = 0;
      for (auto &pt : masks[i]) {
        int u = int(pt[0].get<double>() * W);
        int v = int(pt[1].get<double>() * H);
        su += u; sv += v; ++cnt;
      }
      if (cnt == 0) continue;
      int u = int(su / cnt), v = int(sv / cnt);
      size_t idx = size_t(v) * latest_cloud_->width + size_t(u);
      if (idx >= latest_cloud_->points.size()) continue;
      auto &p = latest_cloud_->points[idx];
      if (std::isnan(p.x)) continue;

      geometry_msgs::msg::PointStamped ps;
      ps.header = cloud_header_;
      ps.point.x = p.x;
      ps.point.y = p.y;
      ps.point.z = p.z;
      pos_pub_->publish(ps);

      RCLCPP_INFO(
        get_logger(),
        "Can at x=%.3f y=%.3f z=%.3f (conf=%.2f)",
        p.x, p.y, p.z, confs[i].get<double>()
      );
    }

    clearBuffers();
  }

  void clearBuffers()
  {
    latest_image_.release();
    latest_cloud_.reset();
  }

  static size_t curlWrite(void *ptr, size_t size, size_t nmemb, void *userp)
  {
    auto *s = reinterpret_cast<std::string*>(userp);
    s->append(reinterpret_cast<char*>(ptr), size*nmemb);
    return size*nmemb;
  }

  std::string sendToAPI(const std::vector<uchar> &png)
  {
    CURL *c = curl_easy_init();
    if (!c) { RCLCPP_ERROR(get_logger(), "curl init failed"); return {}; }

    std::string response;
    curl_easy_setopt(c, CURLOPT_URL, endpoint_.c_str());
    struct curl_slist *hdrs = nullptr;
    hdrs = curl_slist_append(hdrs, ("Authorization: Bearer "+api_key_).c_str());
    curl_easy_setopt(c, CURLOPT_HTTPHEADER, hdrs);

    curl_mime *mime = curl_mime_init(c);
    curl_mimepart *part = curl_mime_addpart(mime);
    curl_mime_name(part, "file");
    curl_mime_data(part, reinterpret_cast<const char*>(png.data()), png.size());
    curl_mime_filename(part, "img.png");
    curl_easy_setopt(c, CURLOPT_MIMEPOST, mime);

    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, curlWrite);
    curl_easy_setopt(c, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(c);
    curl_slist_free_all(hdrs);
    curl_mime_free(mime);
    curl_easy_cleanup(c);

    if (res != CURLE_OK) {
      RCLCPP_ERROR(get_logger(), "curl failed: %s", curl_easy_strerror(res));
      return {};
    }
    return response;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanLocator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}