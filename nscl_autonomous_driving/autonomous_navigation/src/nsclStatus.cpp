#include "autonomous_navigation/nsclStatus.h"


NSCLStatus::NSCLStatus()
{
	ROS_INFO("### nscl status ###");
	totalStatus_pub = nh_.advertise<nscl_msgs::CommInfo>("/nscl/status",1);
	sonar_br_pub = nh_.advertise<sensor_msgs::Range>("/sonar_br",1);
	sonar_bl_pub = nh_.advertise<sensor_msgs::Range>("/sonar_bl",1);
	
	battery_sub = nh_.subscribe<sensor_msgs::BatteryState>("/battery_state", 1, &NSCLStatus::batteryCb, this);
	sonar_sub = nh_.subscribe<std_msgs::String>("/sonar", 1, &NSCLStatus::sonarCb, this);
	led_sub = nh_.subscribe<std_msgs::String>("/nscl/status/led", 1, &NSCLStatus::ledCb, this);
	lidar_sub = nh_.subscribe<std_msgs::String>("/nscl/status/lidar", 1, &NSCLStatus::lidarCb, this);
	pose_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &NSCLStatus::poseCb, this);
	bumper_sub = nh_.subscribe<std_msgs::String>("/nscl/status/bumper", 1, &NSCLStatus::bumperCb, this);
	vel_sub = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &NSCLStatus::velCb, this);
	resp_sub= nh_.subscribe<std_msgs::String>("/nscl/status/response", 1, &NSCLStatus::respCb, this);

  	mapSubscriber = nh_.subscribe("/map_metadata", 1000, &NSCLStatus::mapInfoCb, this);

	sonar_br_cnt = 0;
	sonar_bl_cnt = 0;
	total_sonar_br_cnt = 0;
	total_sonar_bl_cnt = 0;
}


NSCLStatus::~NSCLStatus()
{

}

void NSCLStatus::mapInfoCb(nav_msgs::MapMetaData map_data)
{
  g_map_data = map_data;
}

void NSCLStatus::publishNSCLStatus()
{
    ros::Rate loop_rate(1); //1hz
	while(ros::ok())
    { 
		//ROS_INFO_STREAM("Publish Communication infomation");		
		ros::param::get("nscl/status/battery", comminfo_msg.battery);
		ros::param::get("nscl/status/bumper", comminfo_msg.bumper);
		ros::param::get("nscl/status/sonar", comminfo_msg.sonar);
		ros::param::get("nscl/status/led", comminfo_msg.led);
		ros::param::get("nscl/status/lidar", comminfo_msg.lidar);
		ros::param::get("nscl/status/pose/x", comminfo_msg.pose_x);
		ros::param::get("nscl/status/pose/y", comminfo_msg.pose_y);
		ros::param::get("nscl/status/pose/w", comminfo_msg.pose_theta);
		ros::param::get("nscl/status/velocity/v", comminfo_msg.trans_velocity);
		ros::param::get("nscl/status/velocity/w", comminfo_msg.ang_velocity);
		ros::param::get("nscl/status/response", comminfo_msg.response);
		
		totalStatus_pub.publish(comminfo_msg);
		
        ros::spinOnce();
        loop_rate.sleep();
    }
}

geometry_msgs::Point NSCLStatus::calcPosToPixel(float x, float y)
{
	geometry_msgs::Point pt;

	// origin(0, 0) = (g_map_data.origin.position.x,  g_map_data.origin.position.y + g_map_data.height *g_map_data.resolution)
	float rounded_down = floorf(g_map_data.resolution * 10000) *0.0001;

	pt.x =  (x - g_map_data.origin.position.x) / rounded_down;
	pt.y =  g_map_data.height + ((g_map_data.origin.position.y -y) / rounded_down)  ;

	return pt;
}
void NSCLStatus::batteryCb(const sensor_msgs::BatteryStateConstPtr& str)
{ 
	//ROS_INFO_STREAM("nscl/status/battery :" << *str);
	ros::param::set("/nscl/status/battery", str->voltage);   
}
void NSCLStatus::sonarCb(const std_msgs::StringConstPtr& str)
{
	//ROS_INFO_STREAM("nscl/status/sonar :" << *str);
	ros::param::set("/nscl/status/sonar", str->data.c_str());  
	sonarSplit(str->data.c_str());
}
void NSCLStatus::sonarSplit(const char* total_sonar)
{
	total_sonar_br_cnt++;
	total_sonar_bl_cnt++;

	//ROS_INFO_STREAM("nscl/status/sonar :" << total_sonar[0] << total_sonar[6]);

	if(total_sonar[0] == '1')
	{
		sonar_bl_cnt++;
	}
	else if(total_sonar [0] == '0')
	{
		
	}

	if(total_sonar[6] == '1')
	{
		sonar_br_cnt++;
	}
	else if(total_sonar [6] == '0')
	{
		
	}

	if(sonar_bl_cnt >= 6 && total_sonar_bl_cnt >= 10)
	{
		ROS_INFO_STREAM("Detect sonar");
		sonar_bl_msg.header.stamp = ros::Time::now();
		sonar_bl_msg.header.frame_id = "sonar_bl";
		sonar_bl_msg.radiation_type = 0; // ULTRASOUND=0
		sonar_bl_msg.field_of_view = 0.436332222; 
		sonar_bl_msg.min_range = 0.1; 
		sonar_bl_msg.max_range = 0.5; 
		sonar_bl_msg.range = 0.3 ; 

		sonar_bl_cnt = 0;
		total_sonar_bl_cnt = 0;
		sonar_bl_pub.publish(sonar_bl_msg);
	}
	else if(total_sonar_bl_cnt >= 10)
	{		
		sonar_bl_msg.header.stamp = ros::Time::now();
		sonar_bl_msg.header.frame_id = "sonar_bl";
		sonar_bl_msg.radiation_type = 0; // ULTRASOUND=0
		sonar_bl_msg.field_of_view = 0.436332222; 
		sonar_bl_msg.min_range = 0.1; 
		sonar_bl_msg.max_range = 0.50; 
		sonar_bl_msg.range = 0.50; 

		sonar_bl_cnt = 0;
		total_sonar_bl_cnt = 0;
		sonar_bl_pub.publish(sonar_bl_msg);
	}

	if(sonar_br_cnt >= 6 && total_sonar_br_cnt >= 10)
	{
		sonar_br_msg.header.stamp = ros::Time::now();
		sonar_br_msg.header.frame_id = "sonar_br";
		sonar_br_msg.radiation_type = 0; // ULTRASOUND=0
		sonar_br_msg.field_of_view = 0.436332222; 
		sonar_br_msg.min_range = 0.1; 
		sonar_br_msg.max_range = 0.50; 
		sonar_br_msg.range = 0.3; 

		sonar_br_cnt = 0;
		total_sonar_br_cnt = 0;	
		sonar_br_pub.publish(sonar_br_msg);
	}
	else if(total_sonar_br_cnt >= 10)
	{	
	 	sonar_br_msg.header.stamp = ros::Time::now();
	 	sonar_br_msg.header.frame_id = "sonar_br";
	 	sonar_br_msg.radiation_type = 0; // ULTRASOUND=0
	 	sonar_br_msg.field_of_view = 0.436332222; 
	 	sonar_br_msg.min_range = 0.1; 
	 	sonar_br_msg.max_range = 0.5; 
	 	sonar_br_msg.range = 0.5; 	

		sonar_br_cnt = 0;
		total_sonar_br_cnt = 0;

		sonar_br_pub.publish(sonar_br_msg);
	}
}
void NSCLStatus::ledCb(const std_msgs::StringConstPtr& str)
{
	//ROS_INFO_STREAM("nscl/status/led :" << *str);
	ros::param::set("/nscl/status/led", str->data.c_str());  
}
void NSCLStatus::lidarCb(const std_msgs::StringConstPtr& str)
{
	//ROS_INFO_STREAM("nscl/status/lidar :" << *str);
	ros::param::set("/nscl/status/lidar", str->data);  
}
void NSCLStatus::poseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
	geometry_msgs::Quaternion quat = pose->pose.pose.orientation;	
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	current_pt = calcPosToPixel(pose->pose.pose.position.x, pose->pose.pose.position.y);	

	//ROS_INFO_STREAM("nscl/status/pose :" << *pose);
	ros::param::set("/nscl/status/pose/x", current_pt.x);  
	ros::param::set("/nscl/status/pose/y", current_pt.y);
	ros::param::set("/nscl/status/pose/w", yaw);

}
void NSCLStatus::bumperCb(const std_msgs::StringConstPtr& str)
{
	//ROS_INFO_STREAM("nscl/status/bumper :" << *str);
	ros::param::set("/nscl/status/bumper", str->data.c_str());  
}
void NSCLStatus::velCb(const geometry_msgs::TwistConstPtr& vel)
{
	//ROS_INFO_STREAM("nscl/status/velocity :" << *vel);
	ros::param::set("/nscl/status/velocity/v", vel->linear.x);  
	ros::param::set("/nscl/status/velocity/w", vel->angular.z);  

}
void NSCLStatus::respCb(const std_msgs::StringConstPtr& str)
{
	//ROS_INFO_STREAM("nscl/status/response :" << *str);
	ros::param::set("/nscl/status/response", str->data.c_str());  

}
