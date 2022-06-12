
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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_eigen/tf2_eigen.h>
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
    geometry_msgs::TransformStamped current_transform,
                                    project_transform;
    try
    {
        current_transform = tf_buffer.lookupTransform(
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
    
    Eigen::Quaternion<double> current_quaternion{0, 0, 0, 1},
                              project_quaternion{0, 0, 0, 1};
    Eigen::Matrix<double, 1, 3> current_euler_angles;

    tf2::fromMsg(
        current_transform.transform.rotation,
        current_quaternion
    );

    current_euler_angles = current_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

    project_quaternion =
        project_quaternion * Eigen::AngleAxis<double>(
            current_euler_angles.z(), Eigen::Vector3d::UnitZ()
        );

    project_transform.header.stamp = ros::Time::now();
    project_transform.header.frame_id = current_transform.header.frame_id;
    project_transform.child_frame_id = projected_frame_id;

    project_transform.transform = current_transform.transform;
    project_transform.transform.translation.z = 0;

    project_transform.transform.rotation = tf2::toMsg(
        project_quaternion
    );

    tf_broadcaster.sendTransform(project_transform);
}

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "projected_transofrm_publisher");

    Node node;
    node.spin();

    return 0;
}
