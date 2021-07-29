#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_srvs/Trigger.h>

#include "state_trigger_panel.h"

namespace orne_box_waypoint_nav
{

StateTriggerPanel::StateTriggerPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  start_client_ = nh_.serviceClient<std_srvs::Trigger>("start_wp_nav", false);
  suspend_client_ = nh_.serviceClient<std_srvs::Trigger>("suspend_wp_nav", false);

  start_nav_button_ = new QPushButton("StartWaypointsNavigation");
  suspend_nav_button_ = new QPushButton("SuspendWaypointsNavigation");

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(start_nav_button_);
  layout->addWidget(suspend_nav_button_);
  setLayout( layout );
  
  connect(start_nav_button_, SIGNAL(clicked()), this, SLOT(pushStartNavigation()));
  connect(suspend_nav_button_, SIGNAL(clicked()), this, SLOT(pushSuspendNavigation()));
}

void StateTriggerPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

void StateTriggerPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

void StateTriggerPanel::pushStartNavigation() {
    ROS_INFO("Service call: start waypoints navigation");
    
    std_srvs::Trigger trigger;
    start_client_.call(trigger);
}

void StateTriggerPanel::pushSuspendNavigation() {
    ROS_INFO("Service call: suspend waypoints navigation");
    
    std_srvs::Trigger trigger;
    suspend_client_.call(trigger);
}

} // end namespace orne_box_waypoint_nav

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orne_box_waypoint_nav::StateTriggerPanel,rviz::Panel )