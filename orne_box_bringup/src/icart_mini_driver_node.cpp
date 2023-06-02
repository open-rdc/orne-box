#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


#include <ypspur.h>

class Icart_mini_driver : public rclcpp::Node
{
    public:
        Icart_mini_driver(): Node("icart_mini_driver")
        {
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",1);
            js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",1);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&Icart_mini_driver::cmd_vel_cb, this, std::placeholders::_1));
        }
        void read_param();
        void reset_param();
        void bringup_ypspur();
        void joint_states();
        void odometry();
        bool loop();
        std::string odom_frame_id;
        std::string base_frame_id;
        std::string left_wheel_joint;
        std::string right_wheel_joint;
        int loop_hz;
        double liner_vel_lim,liner_accel_lim,angular_vel_lim,angular_accel_lim;
        bool odom_from_ypspur, debug_mode=false;
    private:
        geometry_msgs::msg::Twist::SharedPtr cmd_vel_ = std::make_shared<geometry_msgs::msg::Twist>();
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
        sensor_msgs::msg::JointState js;
        nav_msgs::msg::Odometry odom;
        float dt = 1.0 / loop_hz;
        double tf_time_offset_ = 0.0; 
        tf2::Vector3 z_axis_;
        
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        geometry_msgs::msg::TransformStamped odom_trans;

        //this function is cmd_vel callback and send command ypspur
        void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          cmd_vel_ =  msg;
          Spur_vel(-(msg->linear.x),msg->angular.z);
        }
        
};
    // this function is read and set ypspur_param
    void Icart_mini_driver::read_param()
    { 
      declare_parameter("odom_frame_id","odom");
      declare_parameter("base_frame_id","base_footprint");
      declare_parameter("Hz",10);
      declare_parameter("left_wheel_joint","left_wheel_joint");
      declare_parameter("right_wheel_joint","right_wheel_joint");
      declare_parameter("liner_vel_lim",1.5);
      declare_parameter("liner_accel_lim",1.5);
      declare_parameter("angular_vel_lim",3.14);
      declare_parameter("angular_accel_lim",3.14);
      declare_parameter("calculate_odom_from_ypspur",true);
      declare_parameter("debug_mode",false);
      
      get_parameter("odom_frame_id",odom_frame_id);
      get_parameter("base_frame_id",base_frame_id);
      get_parameter("left_wheel_joint",left_wheel_joint);
      get_parameter("right_wheel_joint",right_wheel_joint);
      get_parameter("liner_vel_lim",liner_vel_lim);
      get_parameter("liner_accel_lim",liner_accel_lim);
      get_parameter("angular_vel_lim",angular_vel_lim);
      get_parameter("angular_accel_lim",angular_accel_lim);
      get_parameter("Hz",loop_hz);
      get_parameter("calculate_odom_from_ypspur",odom_from_ypspur);
      get_parameter("debug_mode",debug_mode);
      RCLCPP_INFO(this->get_logger(),"Set param!!");
      
    }
    // this function is initialize various parameters
     void Icart_mini_driver::reset_param()
    {
        z_axis_.setX(0);
        z_axis_.setY(0);
        z_axis_.setZ(1);
        cmd_vel_->linear.x = 0.0;
        cmd_vel_->angular.z  = 0.0;
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, 0));
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;

        js.name.push_back(left_wheel_joint);
        js.name.push_back(right_wheel_joint);
        js.position.resize(2);
        js.velocity.resize(2);
    }
    //this function is set ypspur_param and bringup ypspur_coordinator
    void Icart_mini_driver::bringup_ypspur()
    {
        if(Spur_init()>0)
        {
          RCLCPP_INFO(this->get_logger(),"Bringup ypspur!!");
          Spur_stop();
          Spur_free();
          Spur_set_pos_GL(0,0,0);
          Spur_set_vel(liner_vel_lim);
          Spur_set_accel(liner_accel_lim);
          Spur_set_angvel(angular_vel_lim);
          Spur_set_angaccel(angular_accel_lim);
        }
        else 
        {
           RCLCPP_WARN(this->get_logger(),"Disconnected ypspur");
        }
    }

    //this function is set and pub joint_states
    void Icart_mini_driver::joint_states()
    {
        rclcpp::Time js_t = this->now();
        const rclcpp::Time current_stamp_js(js_t);
        double l_ang_pos{},r_ang_pos{},l_wheel_vel{},r_wheel_vel{};
        YP_get_wheel_ang(&l_ang_pos, &r_ang_pos);
        YP_get_wheel_vel(&l_wheel_vel, &r_wheel_vel);
        js.header.stamp = current_stamp_js;
        js.header.frame_id = "base_link";
        js.position[0] = -l_ang_pos;
        js.position[1] = -r_ang_pos;
        js.velocity[0] = l_wheel_vel;
        js.velocity[1] = r_wheel_vel;
        js_pub_->publish(js);
    }
    //this function is compute odometry and pub odometry topic , odom tf
    void Icart_mini_driver::odometry()
    {
        //odom
        double x,y,yaw,v,w;
        z_axis_.setX(0);
        z_axis_.setY(0);
        z_axis_.setZ(1);
        rclcpp::Time t = this->now();
        const rclcpp::Time current_stamp(t);
        
        //compute odom from ypspur's function
        if (odom_from_ypspur)
        {
          
          Spur_get_pos_GL(&x,&y,&yaw);
          Spur_get_vel(&v,&w);
        }
        
        //compute odom from cmd_vel
        else
        {
          v = cmd_vel_->linear.x;
          w = cmd_vel_->angular.z;
          yaw = tf2::getYaw(odom.pose.pose.orientation) + dt * w;
          x = odom.pose.pose.position.x + dt * v * cosf(yaw);
          y = odom.pose.pose.position.y + dt * v * sinf(yaw);
        }
    
        //publish odom
        odom.header.stamp = current_stamp;
        odom.header.frame_id = odom_frame_id;
        odom.child_frame_id = base_frame_id;
        odom.pose.pose.position.x = -x;
        odom.pose.pose.position.y = -y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, yaw));
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = w;
        odom_pub_->publish(odom);

        //odom_tf
        odom_trans.header.stamp = current_stamp + rclcpp::Duration::from_seconds(tf_time_offset_);
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = base_frame_id;
        odom_trans.transform.translation.x = -x;
        odom_trans.transform.translation.y = -y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(odom_trans);

        //if debug_mode
        //if (debug_mode)
    }
   
    //main loop function
    bool Icart_mini_driver::loop()
    {
      if (!YP_get_error_state())
      {
          odometry();
          joint_states();
     
      }
      else
      {
          RCLCPP_WARN(this->get_logger(),"Disconnected ypspur");
          bringup_ypspur();
          return false;
      }

     return true;
    }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
//   Icart_mini_driver icart;
  auto icart = std::make_shared<Icart_mini_driver>();

  rclcpp::WallRate looprate(icart->loop_hz);
  icart->read_param();
  icart->reset_param();
  icart->bringup_ypspur();
    
  while (rclcpp::ok())
  {
    icart->loop();
    rclcpp::spin_some(icart);
    looprate.sleep();
  }
  
  return 0;
}
