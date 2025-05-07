// https://www.theconstruct.ai/how-to-integrate-opencv-with-a-ros2-c-node/
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "rclcpp/rclcpp.hpp"
#include <pluginlib/class_loader.hpp>
#include <rome_hud/hud_item.hpp>
#include <vector>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class HudNode : public rclcpp::Node
{
public:
    HudNode() : Node("hud_tester")
    {

        timer_ = create_wall_timer(500ms, std::bind(&HudNode::timer_callback, this));

        this->render_hud();
    }

    public: void assign_plugin(std::shared_ptr<rome_hud::HudItem> plugin){
        this->plugins_.push_back(plugin);
    }

private:
    std::vector<std::shared_ptr<rome_hud::HudItem>> plugins_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "Hello, world!");
        this->render_hud();
    }

    void render_hud()
    {
        cv::Mat img(512, 640, CV_8UC3);
        cv::randu(img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        for (auto &plugin : this->plugins_)
        {
            plugin->render(img);
        }

        cv::imshow("HUD", img);
        cv::waitKey(500);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HudNode>();
    pluginlib::ClassLoader<rome_hud::HudItem> loader("rome_hud", "rome_hud::HudItem");
    std::vector<std::string> classes = loader.getDeclaredClasses();
    for (const auto &name : classes)
    {
        auto plugin = loader.createSharedInstance(name);
        plugin->initialize(node);
        node->assign_plugin(plugin);
        RCLCPP_INFO(node->get_logger(), "plugin %s, \n", name.c_str());
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
}