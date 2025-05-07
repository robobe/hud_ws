#include <rclcpp/rclcpp.hpp>
#include <rome_hud/hud_item.hpp>
#include <iostream>
#include <std_msgs/msg/string.hpp>
#include "rome_interfaces/msg/state_machine.hpp"

// ros2 topic pub -1 /state std_msgs/msg/String "{data: STANDBY }"
const std::string POS_X = "state_machine_pos_x";
const std::string POS_Y = "state_machine_pos_y";
const std::string STATES = "state_machine_states";
const std::string ENABLE = "state_machine_enabled";
const std::string RGB = "text_rgb";
namespace rome_hud
{
    class SMStatus : public rome_hud::HudItem
    {
        
        virtual void initialize(const rclcpp::Node::SharedPtr &node) override
        {
            this->node_ = node;
            this->node_->declare_parameter<int>(POS_X, 100);
            this->node_->declare_parameter<int>(POS_Y, 100);
            this->node_->declare_parameter<bool>(ENABLE, true);
            this->node_->declare_parameter<std::vector<std::string>>(STATES, std::vector<std::string>());
            this->node_->declare_parameter<std::vector<int32_t>>(RGB, {255, 0, 0});
            this->subscription_ = this->node_->create_subscription<rome_interfaces::msg::StateMachine>(
                "/rome/state",
                10,
                std::bind(&SMStatus::handler_, this, std::placeholders::_1));

            RCLCPP_INFO(rclcpp::get_logger("hud_plugins"), "init state machine status");
        }


        virtual void render(cv::Mat frame) override
        {
            int x = this->node_->get_parameter(POS_X).as_int();
            int y = this->node_->get_parameter(POS_Y).as_int();
            auto states = this->node_->get_parameter(STATES).as_string_array();
            bool is_enabled = this->node_->get_parameter(ENABLE).as_bool();
            auto rgb = this->node_->get_parameter(RGB).as_integer_array();

            if (! is_enabled){
                return;
            }
            int status_index = std::stoi(this->status_);
            std::string status_caption = "not declared";
            if(status_index < (int)states.size())
            {
                status_caption = states.at(status_index);
            }
            

            cv::putText(frame, //target image
            status_caption,
            cv::Point(x, y), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            CV_RGB(0,0,0), //font color
            4);

            cv::putText(frame, //target image
            status_caption,
            cv::Point(x, y), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            CV_RGB(rgb[0], rgb[1], rgb[2]), //font color
            2);
        }

    private:
        
        int fid_ = 0;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<rome_interfaces::msg::StateMachine>::SharedPtr subscription_;
        std::string status_  = "1";
        
        void handler_(const rome_interfaces::msg::StateMachine &msg)
        {
            this->status_ = std::to_string(msg.current_state);
            // RCLCPP_INFO_STREAM(this->node_->get_logger(), "status: " << this->status_ << "\n");
        }
        
    };
};

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rome_hud::SMStatus, rome_hud::HudItem)