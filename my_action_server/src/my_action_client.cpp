#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_action_server/pathAction.h> //reference action message in this package

 // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

using namespace std;
typedef actionlib::SimpleActionClient<my_action_server::pathAction> Client;

bool g_lidar_alarm=false;
bool g_rotating_feedback=false;
ros::Subscriber g_alarm_subscriber;

class MyActionClient
{

private:
    ros::NodeHandle nh_;
    Client action_client_;
    my_action_server::pathGoal goal_; // goal message, received from client
    my_action_server::pathResult result_; // put results here, to be sent back to the client when done w/ goal
    my_action_server::pathFeedback feedback_;

public:
	MyActionClient();

	~MyActionClient(void) {
    }
    // Action Interface
    void lidarCb(const std_msgs::Bool& lidar_alarm);
    void feedbackCb(const my_action_server::pathFeedbackConstPtr& fdbk_msg);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
    void setup();
    void goalReset();
    void goalCancel();
    void goalClear();

};

	MyActionClient::MyActionClient() : 
		action_client_("path_action", true)
	{
	    ROS_INFO("waiting for server: ");
	    action_client_.waitForServer(); // wait for up to 5 seconds
	    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
	    //bool server_exists = action_client.waitForServer(); //wait foreve
	    //ros::Subscriber alarm_subscriber = nh.subscribe("lidar_alarm", 1, lidarCb);
	    ROS_INFO("connected to action server"); 

	}

	geometry_msgs::Quaternion MyActionClient::convertPlanarPhi2Quaternion(double phi) {
	    geometry_msgs::Quaternion quaternion;
	    quaternion.x = 0.0;
	    quaternion.y = 0.0;
	    quaternion.z = sin(phi / 2.0);
	    quaternion.w = cos(phi / 2.0);
	    return quaternion;
	}
	
	void MyActionClient::feedbackCb(const my_action_server::pathFeedbackConstPtr& fdbk_msg) {
	    ROS_INFO("feedback status");
	    feedback_.fdbk = fdbk_msg->fdbk;
	    g_rotating_feedback=feedback_.fdbk; //make status available to "main()"
    	if(g_lidar_alarm && !g_rotating_feedback){
    		ROS_INFO("we gonna crash");
    		goalCancel();
    		goalClear();
    		ros::spinOnce();
    	}
    	else if(!g_lidar_alarm && g_rotating_feedback){
    		ROS_INFO("you spin me right round");
    		goalCancel();
    		goalReset();
    		ros::spinOnce();
    	}
	}

	void MyActionClient::lidarCb(const std_msgs::Bool& lidar_alarm){
		g_lidar_alarm=lidar_alarm.data;
		if(lidar_alarm.data){
			ROS_INFO("cancelling goal");
	    	//action_client_.cancelGoal();
		}
	}

	void MyActionClient::setup(){
    //actionlib::SimpleActionClient<my_action_server::pathAction> action_client("path_action", true);
    	geometry_msgs::Quaternion quat;
    	

		geometry_msgs::PoseStamped pose_stamped;
	    geometry_msgs::Pose pose;
	    pose.position.x = 0.0; // say desired x-coord is 1
	    pose.position.y = 0.0;
	    pose.position.z = 0.0; // let's hope so!
	    pose.orientation.x = 0.0; //always, for motion in horizontal plane
	    pose.orientation.y = 0.0; // ditto
	    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
	    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
	    pose_stamped.pose = pose;
	    goal_.nav_path.poses.push_back(pose_stamped);
	    
	    // some more poses...
	    quat = convertPlanarPhi2Quaternion(0); // get a quaternion corresponding to this heading
	    pose_stamped.pose.orientation = quat;   
	    
	    pose_stamped.pose.position.x=3; 
	    pose_stamped.pose.position.y=0.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);
	    
	    pose_stamped.pose.position.x=8.0; 
	    pose_stamped.pose.position.y=5.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);

	    pose_stamped.pose.position.x=4.0; 
	    pose_stamped.pose.position.y=7.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);
	    
	    pose_stamped.pose.position.x=3.0; 
	    pose_stamped.pose.position.y=7.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);

	    pose_stamped.pose.position.x=3.0; 
	    pose_stamped.pose.position.y=12.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);
	    
	    pose_stamped.pose.position.x=0.0; 
	    pose_stamped.pose.position.y=12.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);

	    goal_.rotate=false;

	   	g_alarm_subscriber = nh_.subscribe("lidar_alarm",1, &MyActionClient::lidarCb, this);
	  
	    action_client_.sendGoal(goal_);
	}

	void MyActionClient::goalReset(){
    //actionlib::SimpleActionClient<my_action_server::pathAction> action_client("path_action", true);


    	geometry_msgs::Quaternion quat;
    	

		geometry_msgs::PoseStamped pose_stamped;
	    geometry_msgs::Pose pose;
	    pose.position.x = 0.0; // say desired x-coord is 1
	    pose.position.y = 0.0;
	    pose.position.z = 0.0; // let's hope so!
	    pose.orientation.x = 0.0; //always, for motion in horizontal plane
	    pose.orientation.y = 0.0; // ditto
	    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
	    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
	    pose_stamped.pose = pose;
	    goal_.nav_path.poses.clear();
	    goal_.nav_path.poses.push_back(pose_stamped);
	    
	    // some more poses...
	    quat = convertPlanarPhi2Quaternion(0); // get a quaternion corresponding to this heading
	    pose_stamped.pose.orientation = quat;   
	    
	    pose_stamped.pose.position.x=3; 
	    pose_stamped.pose.position.y=0.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);
	    
	    pose_stamped.pose.position.x=8.0; 
	    pose_stamped.pose.position.y=5.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);

	    pose_stamped.pose.position.x=4.0; 
	    pose_stamped.pose.position.y=7.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);
	    
	    pose_stamped.pose.position.x=3.0; 
	    pose_stamped.pose.position.y=7.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);

	    pose_stamped.pose.position.x=3.0; 
	    pose_stamped.pose.position.y=12.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);
	    
	    pose_stamped.pose.position.x=0.0; 
	    pose_stamped.pose.position.y=12.0; 
	    goal_.nav_path.poses.push_back(pose_stamped);

	    goal_.rotate=false;
	  
	    action_client_.sendGoal(goal_);
	}

	void MyActionClient::goalCancel(){
		 action_client_.cancelGoal();

	}

	void MyActionClient::goalClear(){
		goal_.nav_path.poses.clear();
		geometry_msgs::PoseStamped pose_stamped;
	    geometry_msgs::Pose pose;
	    pose.position.x = 0.0; // say desired x-coord is 1
	    pose.position.y = 0.0;
	    pose.position.z = 0.0; // let's hope so!
	    pose.orientation.x = 0.0; //always, for motion in horizontal plane
	    pose.orientation.y = 0.0; // ditto
	    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
	    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
	    pose_stamped.pose = pose;
	    goal_.nav_path.poses.push_back(pose_stamped);
	    goal_.rotate=true;
	    action_client_.sendGoal(goal_);
	}




int main(int argc, char **argv) {
    ros::init(argc, argv, "my_action_client_node"); // name this node 
   
    // goal to send to server
    MyActionClient my_client;
    my_client.setup();
    	    
	   ros::spin(); 
    /*
    while(ros::ok()) {
    	
    	if(!g_lidar_alarm && !g_rotating_feedback){
    		ros::spinOnce();
    	}
    	else if(g_lidar_alarm && !g_rotating_feedback){
    		ROS_INFO("we gonna crash");
    		my_client.goalCancel();
    		my_client.goalClear();
    		g_rotating_feedback=true;
    		ros::spinOnce();
    	}
    	else if(!g_lidar_alarm && g_rotating_feedback){
    		ROS_INFO("you spin me right round");
    		my_client.goalCancel();
    		my_client.goalReset();
    		g_rotating_feedback=false;
    		ros::spinOnce();
    	}
    	else{
    		ros::spinOnce();
    	}
    }
*/
    return 0;
}