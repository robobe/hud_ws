#include <rclcpp/rclcpp.hpp>
#include <rome_hud/hud_item.hpp>
#include <iostream>
#include <std_msgs/msg/bool.hpp>

// ros2 topic pub -1 /std_msgs/msg/Bool "{data: True}"
// ros2 topic pub -1 /std_msgs/msg/Bool "{data: False}"

namespace rome_hud
{
    class FID : public rome_hud::HudItem
    {
        virtual void initialize(const rclcpp::Node::SharedPtr &node) override
        {
            this->node_ = node;
            // this->subscription_ = this->node_->create_subscription<std_msgs::msg::Bool>(
            //     "/master_caution",
            //     10,
            //     std::bind(&FID::handler_, this, std::placeholders::_1));

            RCLCPP_INFO(rclcpp::get_logger("hud_plugins"), "init fid counter");
        }
        virtual void render(cv::Mat frame) override
        {
            fid_++;
            cv::putText(frame, //target image
            std::to_string(fid_),
            cv::Point(10, frame.rows / 2), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            CV_RGB(118, 185, 0), //font color
            2);
        }

    private:
        int fid_ = 0;
        rclcpp::Node::SharedPtr node_;
        
        
    };
};

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rome_hud::FID, rome_hud::HudItem)