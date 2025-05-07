#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_loader.hpp>
#include <rome_hud/hud_item.hpp>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <unistd.h>
#include <fstream> 

const std::string ROME_TMP = "/tmp/rome/";

class HudNode : public rclcpp::Node
{
public:
    HudNode() : Node("hud")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/gimbal_camera/image_raw", qos, std::bind(&HudNode::image_callback, this, std::placeholders::_1));
        publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>("/camera/stream", qos_pub);

        save_pid(get_name());
    }

public:
    void save_pid(const char* name){
        pid_t pid = getpid();
        std::ofstream pid_file(ROME_TMP + name + ".pid");
        pid_file << pid << std::endl;
        pid_file.close();
    }

    void assign_plugin(std::shared_ptr<rome_hud::HudItem> plugin)
    {
        this->plugins_.push_back(plugin);
    }

private:
    std::vector<std::shared_ptr<rome_hud::HudItem>> plugins_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        try
        {
            auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
            for (auto &plugin : this->plugins_)
            {
                plugin->render(img);
            }

            auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
            publisher_->publish(*msg_.get());
        }
        catch (const cv_bridge::Exception &e)
        {
            auto logger = rclcpp::get_logger("my_subscriber");
            RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<HudNode>();
    executor.add_node(node);
    pluginlib::ClassLoader<rome_hud::HudItem> loader("rome_hud", "rome_hud::HudItem");
    std::vector<std::string> classes = loader.getDeclaredClasses();
    for (const auto &name : classes)
    {
        auto plugin = loader.createSharedInstance(name);
        plugin->initialize(node);
        node->assign_plugin(plugin);
        RCLCPP_INFO(node->get_logger(), "plugin %s, \n", name.c_str());
    }
    executor.spin();
    rclcpp::shutdown();
}