#include <rclcpp/rclcpp.hpp>
#include <rome_hud/hud_item.hpp>
#include <iostream>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

// ros2 topic pub -1 /std_msgs/msg/Bool "{data: True}"
// ros2 topic pub -1 /std_msgs/msg/Bool "{data: False}"

const std::string POS_X = "master_caution_pox_x";
const std::string POS_Y = "master_caution_pox_y";
const std::string RADIUS = "master_caution_radius";
const std::string ENABLE = "master_caution_enabled";

namespace rome_hud
{
    class MasterCaution : public rome_hud::HudItem
    {
        virtual void initialize(const rclcpp::Node::SharedPtr &node) override
        {
            this->node_ = node;
            this->node_->declare_parameter<int>(POS_X, 100);
            this->node_->declare_parameter<int>(POS_Y, 100);
            this->node_->declare_parameter<int>(RADIUS, 10);
            this->node_->declare_parameter<bool>(ENABLE, true);

            this->sub_ = this->node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics_agg",
                10,
                std::bind(&MasterCaution::cb, this, std::placeholders::_1));

            RCLCPP_INFO(rclcpp::get_logger("hud_plugins"), "init master caution");
        }
        virtual void render(cv::Mat frame) override
        {
            int x = this->node_->get_parameter(POS_X).as_int();
            int y = this->node_->get_parameter(POS_Y).as_int();
            int radius = this->node_->get_parameter(RADIUS).as_int();
            bool is_enabled = this->node_->get_parameter(ENABLE).as_bool();
            if (! is_enabled){
                return;
            }
            if (is_alert_ == 1)
            {

                cv::Point center(x, y);                           // Declaring the center point
                cv::Scalar line_Color(0, 0, 255);                     // Color of the circle
                int thickness = -1;                                   // thickens of the line
                circle(frame, center, radius, line_Color, thickness); // Using circle()function to draw the line//
            }
        }

    private:
        bool is_alert_ = false;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
        void cb(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
        {
            for (const auto &status : msg->status)
            {
                process_diagnostic_status(status);
            }
        }
        void process_diagnostic_status(const diagnostic_msgs::msg::DiagnosticStatus &status)
        {
            if (status.name == "/master_caution")
            {
                this->is_alert_ = status.level != 0;
            }
            
            
        }
    };
};

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rome_hud::MasterCaution, rome_hud::HudItem)