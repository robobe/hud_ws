#pragma once
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

namespace rome_hud
{
    class HudItem
    {
    public:
        virtual void initialize(const rclcpp::Node::SharedPtr& node) = 0;
        virtual void render(cv::Mat frame) = 0;
        virtual ~HudItem() {}
    protected:
        HudItem(){}
    };
}