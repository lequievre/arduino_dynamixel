#include "rqt_plugins/dynamixel_motor_plugin.h"
#include <pluginlib/class_list_macros.h>


// for debug
#include <QtCore/QDebug>

namespace dynamixel_motor_plugins_ns {

	DynamixelMotorPlugin::DynamixelMotorPlugin()
	: rqt_gui_cpp::Plugin()
	{
		setObjectName("Plugin Dynamixel Motor");
	}

	void DynamixelMotorPlugin::sendGoalTorque()
    {
		goal_torque_msg_.id = line_scan_->text().toInt();
		goal_torque_msg_.goalTorque = line_goal_torque_->text().toInt();
		qWarning() << "id = " << goal_torque_msg_.id  << "\n";
		qWarning() << "goalTorque = " << goal_torque_msg_.goalTorque << "\n";
		 pub_goal_torque_.publish(goal_torque_msg_);
        qWarning() << "End senGoalTorque \n";
    }

	void DynamixelMotorPlugin::createPositionWidget_()
	{
		vlayout_position_= new QVBoxLayout();
		vlayout_position_->setObjectName("vertical_layout_position");

		widget_position_ = new QWidget();
		widget_position_->setWindowTitle("Present Position");

		hlayout_goal_torque_ = new QHBoxLayout();
        hlayout_present_position_ = new QHBoxLayout();

		label_goal_torque_ = new QLabel();
		label_goal_torque_ ->setObjectName("goal_torque_label_");
        label_goal_torque_ ->setText("Goal Torque :");
        QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        label_goal_torque_ ->setSizePolicy(fixed_policy);

        label_present_position_ = new QLabel();
		label_present_position_  ->setObjectName("present_position_label_");
        label_present_position_ ->setText("Present Position :");
        label_present_position_ ->setSizePolicy(fixed_policy);

		line_goal_torque_ = new QLineEdit();
		line_goal_torque_->setObjectName("line_goal_torque_ ");
		line_goal_torque_->setSizePolicy(fixed_policy);
		line_goal_torque_->setText(QString::number(0));

        line_present_position_ = new QLineEdit();
		line_present_position_->setObjectName("line_present_position_ ");
		line_present_position_->setSizePolicy(fixed_policy);
		line_present_position_->setText(QString::number(0));

        button_goal_torque_ = new QPushButton("Send Goal Torque");
        connect(button_goal_torque_, SIGNAL(pressed()), this, SLOT(sendGoalTorque()));

		hlayout_goal_torque_->addWidget(label_goal_torque_);
		hlayout_goal_torque_->addWidget(line_goal_torque_);
		hlayout_goal_torque_->addWidget(button_goal_torque_);


		hlayout_present_position_->addWidget(label_present_position_);
		hlayout_present_position_->addWidget(line_present_position_);

		vlayout_position_->addLayout(hlayout_goal_torque_);
		vlayout_position_->addLayout(hlayout_present_position_);
		
		widget_position_->setLayout(vlayout_position_);

	}

    

    void DynamixelMotorPlugin::createScanWidget_()
    {
		vlayout_scan_= new QVBoxLayout();
		vlayout_scan_->setObjectName("vertical_layout_scan");
		
		// create a main widget for sliders velocities
		widget_scan_ = new QWidget();
		widget_scan_->setWindowTitle("Scan Id Motor");
		
		// create contains of widget scan
		label_scan_ = new QLabel();
		label_scan_->setObjectName("id_label_");
        label_scan_->setText("ID :");
        QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        label_scan_->setSizePolicy(fixed_policy);
        
        line_scan_ = new QLineEdit();
		line_scan_->setObjectName("line_scan_ ");
		line_scan_->setSizePolicy(fixed_policy);
		line_scan_->setText(QString::number(1));
        
        button_scan_ = new QPushButton("Scan");
        connect(button_scan_, SIGNAL(pressed()), this, SLOT(scan()));
		
		hlayout_scan_ = new QHBoxLayout();
		
		hlayout_scan_->addWidget(label_scan_);
		hlayout_scan_->addWidget(line_scan_);
		hlayout_scan_->addWidget(button_scan_);
		
		
		label_model_ = new QLabel();
		label_model_->setObjectName("label_model_");
        label_model_->setText("Model :");
        label_model_->setSizePolicy(fixed_policy);
        
        line_model_ = new QLineEdit();
		line_model_->setObjectName("line_model_ ");
		line_model_->setSizePolicy(fixed_policy);
		line_model_->setText("");
		
		hlayout_model_ = new QHBoxLayout();
		
		hlayout_model_->addWidget(label_model_);
		hlayout_model_->addWidget(line_model_);
		
		vlayout_scan_->addLayout(hlayout_scan_);
		vlayout_scan_->addLayout(hlayout_model_);
		widget_scan_->setLayout(vlayout_scan_);
	}


	void DynamixelMotorPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{ 

		setupROSComponents_();

		// create a main widget
		widget_global_ = new QWidget();
		widget_global_->setWindowTitle("Main widget");
		
		vlayout_global_ = new QVBoxLayout();
		vlayout_global_->setObjectName("vertical_layout_global");
		
		createScanWidget_();
		createPositionWidget_();
		
		tab_widget_ = new QTabWidget();
		tab_widget_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
		
		tab_widget_->addTab(widget_scan_,"Scan Motor");
		tab_widget_->addTab(widget_position_,"Position");
		
		vlayout_global_->addWidget(tab_widget_);
		vlayout_global_->setStretchFactor(tab_widget_, 1);
		
		widget_global_->setLayout(vlayout_global_);
		
		context.addWidget(widget_global_);
		
	}

	void DynamixelMotorPlugin::shutdownPlugin()
	{
		delete label_scan_;
		delete line_scan_;
		delete button_scan_;
		
		delete label_model_;
		delete line_model_;
		
		delete hlayout_model_;
		delete hlayout_scan_;
		delete vlayout_scan_;
		delete widget_scan_;
		
		delete vlayout_global_;
	}

	void DynamixelMotorPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
								qt_gui_cpp::Settings& instance_settings) const
	{
		// TODO save intrinsic configuration, usually using:
		// instance_settings.setValue(k, v)
	}

	void DynamixelMotorPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
								   const qt_gui_cpp::Settings& instance_settings)
	{
		// TODO restore intrinsic configuration, usually using:
		// v = instance_settings.value(k)
	}
	
	void DynamixelMotorPlugin::scan()
	{
		qWarning() << "Enter in scan()" << "\n";
		
		a_scan_service_.request.id = line_scan_->text().toInt();
		
		qWarning() << "Id motor = " << a_scan_service_.request.id << "\n";
		
	
		bool return_call = scan_service_client_.call(a_scan_service_);

		qWarning() << "return call = " << return_call << "\n"; 
		
		uint16_t an_modelNumber = a_scan_service_.response.modelNumber;
		
		qWarning() << "Model number = " << an_modelNumber << "\n";
		
		line_model_->setText(QString::number(an_modelNumber));
		
		qWarning() << "End of scan()";
	}

    void DynamixelMotorPlugin::presentPositionCallback_(const rqt_plugins::PresentPosition::ConstPtr& msg)
    {
			line_present_position_->setText(QString::number(msg->presentPosition));
    }
	
	void DynamixelMotorPlugin::setupROSComponents_()
	{
		qWarning() << "Enter in setupROSComponents_()" << "\n";
		scan_service_client_ = getNodeHandle().serviceClient<rqt_plugins::Scan>("/scan");

		pub_goal_torque_ = getNodeHandle().advertise<rqt_plugins::GoalTorque>("/GoalTorque", 1000);

		sub_present_position_=getNodeHandle().subscribe("/PresentPosition", 1000, &DynamixelMotorPlugin::presentPositionCallback_,this);

		qWarning() << "End of setupROSComponents_()" << "\n";
	}
	
	void DynamixelMotorPlugin::shutdownROSComponents_()
	{
		scan_service_client_.shutdown();
	}


 } // End of namespace

PLUGINLIB_DECLARE_CLASS(dynamixel_motor_plugins_ns, DynamixelMotorPlugin, dynamixel_motor_plugins_ns::DynamixelMotorPlugin, rqt_gui_cpp::Plugin)

