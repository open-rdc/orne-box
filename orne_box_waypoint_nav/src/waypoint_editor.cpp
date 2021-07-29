#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <std_msgs/String.h>

#include <std_srvs/Trigger.h>

#include "yaml-cpp/yaml.h"

#include <string>
#include <fstream>
#include <cmath>
#include <vector>
#include <memory>
#include <functional>

typedef struct Waypoints{
    geometry_msgs::Pose pose;
    std::string function;
}Waypoints;

using namespace visualization_msgs;

class WaypointEditor
{
private:
    std::string filename_;
    std::string map_frame_;
    std::vector<Waypoints> waypoints_;
    std::vector<std::string> function_list_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher marker_id_pub_;
    ros::Publisher marker_func_pub_;
    visualization_msgs::MarkerArray marker_id_description_;
    visualization_msgs::MarkerArray marker_func_description_;
    ros::ServiceServer save_server_;

    ros::Rate rate_;

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    interactive_markers::MenuHandler wp_menu_handler_;

public:
    WaypointEditor();
    ~WaypointEditor();

    bool read_yaml(void);
    bool save_yaml(void);

    void initMenu(void);
    void wpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void wpInsertCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void funcChangeCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void makeWpInteractiveMarker(std::string name, geometry_msgs::Point point);
    void makeWpsInteractiveMarker(void);

    InteractiveMarkerControl& makeWpControl(InteractiveMarker &msg);

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    Marker makeWpMarker(void);
    void publishMarkerIdDescription(void);
    void publishMarkerFuncDescription(void);
    bool saveWaypointCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

    void run(void);

};

WaypointEditor::WaypointEditor() : filename_(""), nh_(), pnh_("~"), rate_(2)
{
    pnh_.param("filename", filename_, std::string("sample.yaml"));
    pnh_.param("map_frame",map_frame_,std::string("map"));
    pnh_.param("function_list",function_list_,{"run","suspend"});

    if(!read_yaml()){
        ROS_ERROR("exit waypoint editor node");
        return;
    }

    server_.reset(new interactive_markers::InteractiveMarkerServer("waypoints_marker_server", "", false));
    initMenu();

    makeWpsInteractiveMarker();
    server_->applyChanges();

    marker_id_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_id_descriptions",1);
    marker_func_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_func_descriptions",1);
    save_server_ = nh_.advertiseService("save_waypoints", &WaypointEditor::saveWaypointCallback, this);
}

WaypointEditor::~WaypointEditor()
{
    server_.reset();
}

bool WaypointEditor::read_yaml(){
  ROS_INFO_STREAM("Read waypoints data from " << filename_);
// Check whether filename.yaml exist
  std::ifstream ifs(filename_);
  if(ifs){
    const YAML::Node read_result = YAML::LoadFile(filename_);
    YAML::Node wp_yaml;
    try{
      wp_yaml = read_result["waypoints"];
    }
    catch(std::exception e){
      ROS_ERROR("Your yaml format is wrong");
      return false;
    }

    geometry_msgs::Pose pose;
    std::string function;
    for(YAML::Node points : wp_yaml){
      pose.position.x = points["point"]["x"].as<double>();
      pose.position.y = points["point"]["y"].as<double>();
      pose.position.z = points["point"]["z"].as<double>();

      try{
        function = points["point"]["function"].as<std::string>();
      }
      catch(std::exception e){
        ROS_WARN("function is set by default (run) because function is not set in yaml");
        function = std::string("run");
      }
      if(function == ""){
        function = "run";
      }
      waypoints_.push_back({pose, function});
    }
    ROS_INFO_STREAM(waypoints_.size() << " waypoints is read");
    return true;
  }
  else{
    ROS_ERROR("yaml filename is wrong");
    return false;
  }
}

bool 
WaypointEditor::save_yaml(){
        std::ofstream ofs(filename_.c_str(), std::ios::out);
        if(ofs){
            ofs << "waypoints:" << std::endl;

            for(const auto& p : waypoints_){
                ofs << "    " << "- point:" << std::endl;
                ofs << "        x: " << p.pose.position.x << std::endl;
                ofs << "        y: " << p.pose.position.y << std::endl;
                ofs << "        z: " << p.pose.position.z << std::endl;
                ofs << "        function: " << p.function << std::endl;
            }

            ROS_INFO_STREAM("write success");
            ofs.close();
            return true;
        }
        else{
            ROS_ERROR("failed to write \"%s\"", filename_.c_str());
            ofs.close();
            return false;
        } 
        ofs.close();

    }
  
void
WaypointEditor::initMenu(){
    interactive_markers::MenuHandler::EntryHandle wp_delete_menu_handler = wp_menu_handler_.insert("Delete", std::bind(&WaypointEditor::wpDeleteCb, this,std::placeholders::_1));
    interactive_markers::MenuHandler::EntryHandle wp_insert_menu_handler = wp_menu_handler_.insert("Insert");

    interactive_markers::MenuHandler::EntryHandle wp_mode = wp_menu_handler_.insert(wp_insert_menu_handler, "Prev", std::bind(&WaypointEditor::wpInsertCb, this, std::placeholders::_1));
    wp_mode = wp_menu_handler_.insert(wp_insert_menu_handler, "Next", std::bind(&WaypointEditor::wpInsertCb, this, std::placeholders::_1));

    interactive_markers::MenuHandler::EntryHandle wp_function_menu_handler = wp_menu_handler_.insert("Function");

    interactive_markers::MenuHandler::EntryHandle function_mode;
    for (int i = 0; i < function_list_.size(); i++)
    {
        function_mode = wp_menu_handler_.insert(wp_function_menu_handler, function_list_[i], std::bind(&WaypointEditor::funcChangeCb, this, std::placeholders::_1));
    }

}

void 
WaypointEditor::wpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO_STREAM("delete : " << feedback->marker_name);

    int wp_num = std::stoi(feedback->marker_name);
    waypoints_.erase(waypoints_.begin() + wp_num);

    for (int i=wp_num; i<waypoints_.size(); i++) {
        geometry_msgs::Pose p;
        p.position = waypoints_[i].pose.position;
        server_->setPose(std::to_string(i), p);
    }
    server_->erase(std::to_string((int)waypoints_.size()));
    server_->applyChanges();
}

void 
WaypointEditor::wpInsertCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO_STREAM("menu_entry_id : " << feedback->menu_entry_id);

    int wp_num= std::stoi(feedback->marker_name);
    ROS_INFO_STREAM("insert : " << feedback->menu_entry_id);

    geometry_msgs::Pose p = feedback->pose;
    std::string function = function_list_[0];

    // If you change menu handler, you must modify "3". 
    if (feedback->menu_entry_id == 3){
        p.position.x -= 1.0;
        waypoints_.insert(waypoints_.begin() + wp_num, {p,function});

    // If you change menu handler, you must modify "4". 
    } else if (feedback->menu_entry_id == 4) {
        p.position.x += + 1.0;
        waypoints_.insert(waypoints_.begin() + wp_num + 1, {p,function});
    }

    for (int i=wp_num; i<waypoints_.size()-1; i++) {
        geometry_msgs::Pose p;
        p.position = waypoints_[i].pose.position;
        server_->setPose(std::to_string(i), p);
    }
    makeWpInteractiveMarker(std::to_string(waypoints_.size()-1), waypoints_[waypoints_.size()-1].pose.position);
    server_->applyChanges();
}

void
WaypointEditor::funcChangeCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    ROS_INFO("Switching to menu entry #%d", feedback->menu_entry_id);
    
    int wp_num= std::stoi(feedback->marker_name);
    // If you change menu handler, you must modify "6". 
    int func_entry_id = 6;

    for (int i = func_entry_id,j = 0; i < func_entry_id + function_list_.size(),j<function_list_.size(); i++,j++)
    {
        if (feedback->menu_entry_id == i){
            ROS_INFO("Function Mode Change: %s",function_list_[j].c_str());
            waypoints_[wp_num].function = function_list_[j];
        }
    }

    wp_menu_handler_.reApply( *server_ );
    server_->applyChanges();
}

void 
WaypointEditor::makeWpInteractiveMarker(std::string name, geometry_msgs::Point point){
    InteractiveMarker int_marker;
    int_marker.controls.clear();
    int_marker.header.frame_id = map_frame_;
    int_marker.pose.position = point;
    int_marker.scale = 1;
    int_marker.name = name;
    int_marker.description = name;

    int_marker.controls.push_back(makeWpControl(int_marker));

    server_->insert(int_marker, boost::bind(&WaypointEditor::processFeedback, this, _1));
    wp_menu_handler_.apply(*server_, name);
}

void 
WaypointEditor::makeWpsInteractiveMarker(){
    for (int i=0; i!=waypoints_.size(); i++){
        makeWpInteractiveMarker(std::to_string(i), waypoints_[i].pose.position);
    }
}

InteractiveMarkerControl&
WaypointEditor::makeWpControl(InteractiveMarker &msg) {
        InteractiveMarkerControl control;
        control.markers.clear();
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        control.always_visible = true;
        control.markers.push_back(makeWpMarker());
        msg.controls.push_back(control);

        return msg.controls.back();
    }

Marker
WaypointEditor::makeWpMarker(){
    Marker marker;
    marker.type = Marker::SPHERE;
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;
    marker.color.r = 0.08;
    marker.color.g = 0.0;
    marker.color.b = 0.8;
    marker.color.a = 0.5;

    return marker;
}

void 
WaypointEditor::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "'";

    std::ostringstream mouse_point_ss;

    switch(feedback->event_type){
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        ROS_INFO_STREAM(s.str() << ":button click" << mouse_point_ss.str() << ".");
        break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        ROS_INFO_STREAM(s.str() << ":menu item" << feedback->menu_entry_id << "clicked" << mouse_point_ss.str() << ".");
        break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_INFO_STREAM(s.str() << ":pose changed"
            << "\nposition = "
            << feedback->pose.position.x
            << "," << feedback->pose.position.y
            << "," << feedback->pose.position.z
            << "\norientation = "
            << "," << feedback->pose.orientation.x
            << "," << feedback->pose.orientation.y
            << "," << feedback->pose.orientation.z);
        break;
    }

    server_->applyChanges();

    waypoints_[std::stoi(feedback->marker_name)].pose.position = feedback->pose.position;
}

void 
WaypointEditor::publishMarkerIdDescription(){
    marker_id_description_.markers.clear();
    for(int i=0; i!=waypoints_.size(); i++){
        Marker marker;
        marker.type = Marker::TEXT_VIEW_FACING;
        marker.text = std::to_string(i);
        marker.header.frame_id = map_frame_;
        marker.header.stamp = ros::Time(0);
        std::stringstream name;
        name << "waypoint";
        marker.ns = name.str();
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.pose.position = waypoints_[i].pose.position;
        marker.pose.position.z = 4.0;
        marker.scale.z = 2.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.action = visualization_msgs::Marker::ADD;
        marker_id_description_.markers.push_back(marker);
    }
    marker_id_pub_.publish(marker_id_description_);
}

void 
WaypointEditor::publishMarkerFuncDescription(){
    marker_func_description_.markers.clear();
    for(int i=0; i!=waypoints_.size(); i++){
        Marker marker;
        marker.type = Marker::TEXT_VIEW_FACING;
        marker.text = waypoints_[i].function;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = ros::Time(0);
        std::stringstream name;
        name << "waypoint";
        marker.ns = name.str();
        marker.id = i;
        marker.lifetime = ros::Duration(0.5);
        marker.pose.position = waypoints_[i].pose.position;
        marker.pose.position.z = 2.0;
        marker.scale.z = 2.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.action = visualization_msgs::Marker::ADD;
        marker_func_description_.markers.push_back(marker);
    }
    marker_func_pub_.publish(marker_func_description_);
}

bool 
WaypointEditor::saveWaypointCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
    if (save_yaml()) {
        response.success = true;
        return true;
    } else {
        response.success = false;
        return false;
    }
}

void 
WaypointEditor::run() {
    while(ros::ok()){
        rate_.sleep();
        ros::spinOnce();
        publishMarkerIdDescription();
        publishMarkerFuncDescription();
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "waypoint_editor");
    WaypointEditor wp_editor;
    wp_editor.run();

    return 0;
}
