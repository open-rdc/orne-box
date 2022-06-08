
// MIT License
// 
// Copyright (c) 2022 Naoki Takahashi
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdexcept>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <param_io/get_param.hpp>

class Node
{
    public :
        Node();

        void spin();

    private :
        ros::NodeHandle nh,
                        private_nh;

        double publish_transform_rate,
               tf_lookup_time;

        std::string project_frame_id,
                    projected_frame_id,
                    project_base_frame_id;

        ros::Timer publish_transform_timer;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;
        tf2_ros::TransformBroadcaster tf_broadcaster;

        void publishTransform(const ros::TimerEvent &);
};

Node::Node()
    : nh(),
      private_nh("~"),
      publish_transform_rate(10),
      tf_lookup_time(0),
      project_frame_id("base_link"),
      projected_frame_id("projected_" + project_frame_id),
      project_base_frame_id("map"),
      publish_transform_timer(),
      tf_buffer(),
      tf_listener(tf_buffer),
      tf_broadcaster()
{
    param_io::getParam(
        private_nh,
        "publish_transform_rate",
        publish_transform_rate
    );
    param_io::getParam(
        private_nh,
        "project_frame_id",
        project_frame_id
    );
    param_io::getParam(
        private_nh,
        "projected_frame_id",
        projected_frame_id
    );
    param_io::getParam(
        private_nh,
        "project_base_frame_id",
        project_base_frame_id
    );

    const double publish_duration = 1 / publish_transform_rate;

    if(isnan(publish_duration))
    {
        throw std::runtime_error("1 / publish_transform_rate is NaN");
    }

    publish_transform_timer = nh.createTimer(
        ros::Duration(publish_duration),
        std::bind(
            &Node::publishTransform,
            this,
            std::placeholders::_1
        )
    );
}

void Node::spin()
{
    ros::spin();
}

void Node::publishTransform(const ros::TimerEvent &)
{
    geometry_msgs::TransformStamped target_transform,
                                    project_transform;
    try
    {
        target_transform = tf_buffer.lookupTransform(
            project_base_frame_id,
            project_frame_id,
            ros::Time(tf_lookup_time)
        );
    }
    catch(tf2::TransformException &e)
    {
        ROS_WARN_STREAM(e.what());
        return;
    }
    project_transform.header.stamp = ros::Time::now();
    project_transform.header.frame_id = target_transform.header.frame_id;
    project_transform.child_frame_id = projected_frame_id;
    project_transform.transform = target_transform.transform;
    project_transform.transform.translation.z = 0;

    tf_broadcaster.sendTransform(project_transform);
}

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "projected_transofrm_publisher");

    Node node;
    node.spin();

    return 0;
}
