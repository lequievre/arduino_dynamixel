/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef dynamixel_motor_plugin_H
#define dynamixel_motor_plugin_H

// rqt
#include <rqt_gui_cpp/plugin.h>

// Service Scan
#include <rqt_plugins/Scan.h>

// Topic GoalTorque
#include <rqt_plugins/GoalTorque.h>

// Topic PresentPsotion
#include <rqt_plugins/PresentPosition.h>

// ROS
#include "ros/ros.h"
#include "ros/service_client.h"

// Qt graphics
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QTabWidget>

namespace dynamixel_motor_plugins_ns {

	class DynamixelMotorPlugin : public rqt_gui_cpp::Plugin
	{
		Q_OBJECT

		public:

			DynamixelMotorPlugin();

			virtual void initPlugin(qt_gui_cpp::PluginContext& context);

			virtual void shutdownPlugin();

			virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
									  qt_gui_cpp::Settings& instance_settings) const;

			virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
										 const qt_gui_cpp::Settings& instance_settings);

		public slots:
		
			void scan();
            void sendGoalTorque();

		private:
		
			QTabWidget *tab_widget_;
		
			QWidget *widget_scan_, *widget_position_, *widget_global_;
			
			QVBoxLayout *vlayout_global_, *vlayout_scan_, *vlayout_position_;
			QHBoxLayout *hlayout_scan_, *hlayout_model_, *hlayout_goal_torque_, *hlayout_present_position_;
			QLabel *label_scan_, *label_model_, *label_goal_torque_, *label_present_position_;
			QLineEdit *line_scan_, *line_model_, *line_goal_torque_, *line_present_position_;
			//QHBoxLayout* button_scan_;
			
			QPushButton *button_scan_, *button_goal_torque_;
			
			ros::ServiceClient scan_service_client_;
			rqt_plugins::Scan a_scan_service_;

			rqt_plugins::GoalTorque goal_torque_msg_;

			ros::Publisher  pub_goal_torque_;
			ros::Subscriber sub_present_position_;
			
			void setupROSComponents_();
			void shutdownROSComponents_();
			void createScanWidget_();
			void createPositionWidget_();
			void presentPositionCallback_(const rqt_plugins::PresentPosition::ConstPtr& msg);

	}; // End of class

} // End of namespace

#endif
