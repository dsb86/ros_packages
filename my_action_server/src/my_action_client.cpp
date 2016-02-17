#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_action_server/pathAction.h> //reference action message in this package

using namespace std;

void lidarCb(const std_msgs::Bool& lidar_alarm){
	if(lidar-alarm){
		ROS_INFO("cancelling goal");
    	action_client.cancelGoal();
	}
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "my_action_client_node"); // name this node 
    
    // goal to send to server
   	my_action_server::pathGoal goal; 
    

    actionlib::SimpleActionClient<my_action_server:pathAction> action_client("path_action", true);


    geometry_msgs::Quaternion quat;
    ros::Subscriber lidar_subscriber = nh.subscribe("lidar_alarm", 1, lidarCb);


    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    
   
    ROS_INFO("connected to action server"); 

    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
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
    goal.request.nav_path.poses.push_back(pose_stamped);
    
    // some more poses...
    quat = convertPlanarPhi2Quaternion(0); // get a quaternion corresponding to this heading
    pose_stamped.pose.orientation = quat;   
    
    pose_stamped.pose.position.x=3; 
    pose_stamped.pose.position.y=0.0; 
    goal.request.nav_path.poses.push_back(pose_stamped);
    
    pose_stamped.pose.position.x=8.0; 
    pose_stamped.pose.position.y=5.0; 
    goal.request.nav_path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x=4.0; 
    pose_stamped.pose.position.y=7.0; 
    goal.request.nav_path.poses.push_back(pose_stamped);
    
    pose_stamped.pose.position.x=3.0; 
    pose_stamped.pose.position.y=7.0; 
    goal.request.nav_path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x=3.0; 
    pose_stamped.pose.position.y=12.0; 
    goal.request.nav_path.poses.push_back(pose_stamped);
    
    pose_stamped.pose.position.x=0.0; 
    pose_stamped.pose.position.y=12.0; 
    goal.request.nav_path.poses.push_back(pose_stamped);


  
    action_client.sendGoal(goal);
    ros::spin();
    

    return 0;
}