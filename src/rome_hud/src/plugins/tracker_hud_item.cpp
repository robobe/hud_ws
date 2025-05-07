#include <rclcpp/rclcpp.hpp>
#include <rome_hud/hud_item.hpp>
#include <iostream>
#include "rome_interfaces/msg/tracker_result.hpp"
#include <rclcpp/qos.hpp>
#include <map>
#include <mutex>
#include <condition_variable>

namespace rome_hud
{
    class TrackerData
    {
    public:
        TrackerData() {}
        cv::Point2i up_left_, down_right_;
        int8_t tracker_id;
        bool render;
    };

    class Tracker : public rome_hud::HudItem
    {
        virtual void initialize(const rclcpp::Node::SharedPtr &node) override
        {

            this->node_ = node;
            callback_group_subscriber1_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            auto sub1_opt = rclcpp::SubscriptionOptions();
            sub1_opt.callback_group = callback_group_subscriber1_;
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
            this->subscription_ = this->node_->create_subscription<rome_interfaces::msg::TrackerResult>(
                "/tracker_result",
                qos,
                std::bind(&Tracker::handler_, this, std::placeholders::_1),
                sub1_opt);

            // this->render_ = {
            //     {0, std::make_unique<TrackerData>()},
            //     {1, std::make_unique<TrackerData>()}
            // };
            this->render_[0] = std::make_unique<TrackerData>();
            this->render_[1] = std::make_unique<TrackerData>();
            RCLCPP_INFO(rclcpp::get_logger("hud_plugins"), "init tracker hud");
        }

        /// @brief render all active tracker bbox
        /// @param frame
        virtual void render(cv::Mat frame) override
        {
            std::unique_lock<std::mutex> lock(this->mtx);

            // Wait for a maximum of 5 seconds or until notified
            //TODO: change the wait time
            if (this->cv.wait_for(lock, std::chrono::milliseconds(111), [&]
                            { return notify; }))
            {
                // std::cout << "Worker: Received notification!\n";
                auto logger = this->node_->get_logger();
                RCLCPP_INFO(logger, "trigger");

            }
            else
            {
                auto logger = this->node_->get_logger();
                RCLCPP_INFO(logger, "timeout");
                // std::cout << "Worker: Timeout reached, proceeding without notification.\n";
            }
            this->notify = false;
            for (const auto &render : this->render_)
            {
                // get tracker color
                char tracker_id = render.first;
                auto color = this->tracker_color_mapping[tracker_id];
                if (render.second->render)
                {
                    rectangle(frame,
                              render.second->up_left_,
                              render.second->down_right_,
                              cv::Scalar(color[0], color[1], color[2]),
                              2, cv::LINE_8);
                }

                circle(frame, cv::Point2i(320, 256), 3, cv::Scalar(255, 0, 255), 3); // Using circle()function to draw the line//
            }
        }

    private:
        std::mutex mtx;
        std::condition_variable cv;
        bool notify = false;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
        std::map<char, std::unique_ptr<TrackerData>> render_; // map tracker id to allow render
        cv::Point2i up_left_, down_right_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<rome_interfaces::msg::TrackerResult>::SharedPtr subscription_;
        std::map<char, std::vector<int>> tracker_color_mapping = {
            {0, {255, 0, 0}},
            {1, {0, 0, 255}}};

        void handler_(const rome_interfaces::msg::TrackerResult &msg)
        {
            bool is_track = msg.status == rome_interfaces::msg::TrackerResult::STATUS_TRACK;
            this->render_[msg.tracker_id]->render = is_track;

            this->render_[msg.tracker_id]->up_left_ = cv::Point2i(
                int(msg.bbox[0].x),
                int(msg.bbox[0].y));

            this->render_[msg.tracker_id]->down_right_ = cv::Point2i(
                int(msg.bbox[1].x),
                int(msg.bbox[1].y));

            {
                std::lock_guard<std::mutex> lock(this->mtx);
                this->notify = true;
            }
            this->cv.notify_one(); // Notify worker
            // this->down_right_ = std::to_integer(msg.bbox[1]);
            // RCLCPP_INFO_STREAM(this->node_->get_logger(), "gate: " << this->down_right_.x << "\n");
        }
    };
};

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rome_hud::Tracker, rome_hud::HudItem)