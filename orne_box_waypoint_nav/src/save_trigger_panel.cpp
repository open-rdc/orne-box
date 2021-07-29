#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_srvs/Trigger.h>

#include "save_trigger_panel.h"

namespace orne_box_waypoint_nav
{

SaveTriggerPanel::SaveTriggerPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  save_client_ = nh_.serviceClient<std_srvs::Trigger>("save_waypoints", false);

  save_button_ = new QPushButton("saveWaypoints");

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(save_button_);
  setLayout( layout );
  
  connect(save_button_, SIGNAL(clicked()), this, SLOT(pushSaveWaypoints()));
}

void SaveTriggerPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void SaveTriggerPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

void SaveTriggerPanel::pushSaveWaypoints() {
    ROS_INFO("Service call: save waypoints");
    
    std_srvs::Trigger trigger;
    if (save_client_.call(trigger)){
        ROS_INFO_STREAM("write success");
    } else {
        // ROS_WARN_STREAM(trigger.message);
    }
}

} // end namespace orne_box_waypoint_nav

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orne_box_waypoint_nav::SaveTriggerPanel,rviz::Panel )