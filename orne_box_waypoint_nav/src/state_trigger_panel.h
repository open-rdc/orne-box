#ifndef STATE_TRIGGER_PANEL_H
#define STATE_TRIGGER_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QPushButton;

namespace orne_box_waypoint_nav
{

class StateTriggerPanel: public rviz::Panel
{
Q_OBJECT
public:
  StateTriggerPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void pushStartNavigation();
  void pushSuspendNavigation();
    
protected:
  ros::NodeHandle nh_;
  ros::ServiceClient start_client_, suspend_client_;
  QPushButton *start_nav_button_;
  QPushButton *suspend_nav_button_;

};

} // end namespace orne_box_waypoint_nav

#endif // STATE_TRIGGER_PANEL_H