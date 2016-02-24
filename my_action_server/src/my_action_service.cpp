#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_action_server/pathAction.h>

#include <ros/ros.h>
#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>

const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 



// here are a few useful utility functions:


class MyActionServer {




private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<my_action_server::pathAction> as_;
    
    // here are some message types to communicate with our client(s)
    my_action_server::pathGoal goal_; // goal message, received from client
    my_action_server::pathResult result_; // put results here, to be sent back to the client when done w/ goal
    my_action_server::pathFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    


public:
    MyActionServer(); //define the body of the constructor outside of class definition

    ~MyActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<my_action_server::pathAction>::GoalConstPtr& goal);
    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

    void do_halt();
    void do_move(double distance);
    void do_spin(double spin_ang);
    void do_evasive_spin();
    //void do_small_spin(double spin_ang);
    void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading);
    void do_inits(ros::NodeHandle &n);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class exampleActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

MyActionServer::MyActionServer() :
   as_(nh_, "path_action", boost::bind(&MyActionServer::executeCB, this, _1),false) 

{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
    do_inits(nh_);
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void MyActionServer::executeCB(const actionlib::SimpleActionServer<my_action_server::pathAction>::GoalConstPtr& goal) {
    ROS_INFO("callback activated");
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose pose_desired;
    
      
    goal_.nav_path= goal->nav_path;
    goal_.rotate= goal->rotate;

    if(!goal_.rotate){
        
        int npts = goal_.nav_path.poses.size();
        ROS_INFO("received path request with %d poses",npts); 
        for (int i=0;i<npts;i++) { //visit each subgoal
            ros::spinOnce();
            if(as_.isPreemptRequested() || !ros::ok()){
                ROS_INFO("Doing Halt From Lin");
                as_.setPreempted();
                do_halt();
                feedback_.fdbk=false;
                as_.publishFeedback(feedback_);
                break;
            }
            else{

                

                // odd notation: drill down, access vector element, drill some more to get pose
                
                pose_desired = goal_.nav_path.poses[i].pose; //get next pose from vector of poses
                
                //WRITE THIS FNC: compute desired heading and travel distance based on current and desired poses
                get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
                ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired,
                   pose_desired.position.x,pose_desired.position.y); 
                ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
                ROS_INFO("travel distance = %f",travel_distance);         
                
                
                // a quaternion is overkill for navigation in a plane; really only need a heading angle
                // this yaw is measured CCW from x-axis
                // GET RID OF NEXT LINE AFTER FIXING get_yaw_and_dist()
                //yaw_desired = convertPlanarQuat2Phi(pose_desired.orientation); //from i'th desired pose
                
                ROS_INFO("pose %d: desired yaw = %f",i,yaw_desired);        
                yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
                spin_angle = yaw_desired - yaw_current; // spin this much
                spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
                do_spin(spin_angle); // carry out this incremental action
                // we will just assume that this action was successful--really should have sensor feedback here
                
                g_current_pose.orientation = convertPlanarPhi2Quaternion(yaw_desired); // assumes got to desired orientation precisely
                
                //FIX THE NEXT LINE, BASED ON get_yaw_and_dist()
                if(travel_distance>g_dist_tol){ 
                  do_move(travel_distance);
                  g_current_pose.position = pose_desired.position;
                }  // move forward 1m...just for illustration; SHOULD compute this from subgoal pose
            }
        }
    }
    else{
        while(1)
        {
            ros::spinOnce();
            if(as_.isPreemptRequested() || !ros::ok()){
                ROS_INFO("Doing Halt From Rot");
                feedback_.fdbk=true;
                as_.setPreempted();
                do_halt();
                as_.publishFeedback(feedback_);
                break;
            }
            else{
                do_evasive_spin();
            }
        }
        
    }


}

void MyActionServer::do_evasive_spin(){
    //feedback_.fdbk=true;
    //as_.publishFeedback(feedback_);
    do_spin(0.5);
}

double MyActionServer::sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double MyActionServer::min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double MyActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion MyActionServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void MyActionServer::do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
        if(as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("Doing Halt From Rot");
            as_.setPreempted();
            do_halt();
            feedback_.fdbk=true;
            as_.publishFeedback(feedback_);
            break;
        }
        else{
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
        }

    }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void MyActionServer::do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
        if(as_.isPreemptRequested() || !ros::ok()){
            ROS_INFO("Doing Halt From Lin");
            as_.setPreempted();
            do_halt();
            feedback_.fdbk=false;
            as_.publishFeedback(feedback_);
            break;
        }
        else{
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
        }
    }  
    do_halt();
}

void MyActionServer::do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}

//THIS FUNCTION IS NOT FILLED IN: NEED TO COMPUTE HEADING AND TRAVEL DISTANCE TO MOVE
//FROM START TO GOAL
void MyActionServer::get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 ROS_INFO("%f", current_pose.position.x);
 ROS_INFO("%f",current_pose.position.y);
 ROS_INFO("%f", goal_pose.position.x);
 ROS_INFO("%f",goal_pose.position.y);


 double x_coor_c = current_pose.position.x;
 //ROS_INFO("xc %f", x_coor_c);
 double y_coor_c = current_pose.position.y;
 //ROS_INFO("yc %f", y_coor_c);
 double x_coor_g = goal_pose.position.x;
 //ROS_INFO("xg %f", x_coor_g);
 double y_coor_g = goal_pose.position.y;
 ROS_INFO("yg %f", y_coor_g);
 
 dist = sqrt(pow((x_coor_c-x_coor_g), 2.0)+pow((y_coor_c-y_coor_g), 2.0)); //rectified
 //ROS_INFO("distance of %f", dist);
 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
 }
 else {
    double y_head=y_coor_g-y_coor_c;
    double x_head=x_coor_g-x_coor_c;
    heading = atan2(y_head,x_head); //REVISED
 }

}

void MyActionServer::do_inits(ros::NodeHandle &n) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "timer_action_server_node"); // name this node 

    ROS_INFO("instantiating the timer_action_server: ");

    MyActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

