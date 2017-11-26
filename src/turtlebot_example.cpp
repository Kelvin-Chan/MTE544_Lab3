//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 3
//
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>

using namespace Eigen;

ros::Publisher marker_pub;

#define TAGID 0

// GLOBALS for milestone generation
#define NUM_MILESTONES 200
#define MAP_X_RANGE 100
#define MAP_Y_RANGE 100
MatrixXd Milestones = MatrixXd::Zero(NUM_MILESTONES, 2);
MatrixXd OccupancyMap = MatrixXd::Zero(MAP_X_RANGE, MAP_Y_RANGE);

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}

//Example of drawing a curve
void drawCurve(int k) 
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p); 

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);   
   }

   //publish new curve
   marker_pub.publish(lines);

}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with
}


// XXX: DUMMY CODE
void generate_occupancy_grid() {
    for (int i = 0; i < 100; i++) {
        for (int j = 0; j< 50; j++) {
            OccupancyMap(i, j) = 100;
        }
    }
}

// generate milestone points and remove point on obstacles
void generate_milestones()
{
    // generate NUM_MILESTONES and only store them if they
    // do not coincide with an objects, otherwise retry

    int current_x;
    int current_y;

    for (int i = 0; i < NUM_MILESTONES; i++)
    {
        while (true) {
            current_x = rand() % MAP_X_RANGE;
            current_y = rand() % MAP_Y_RANGE;
            if (OccupancyMap(current_x, current_y) == 0) {
                break;
            }
        }
        Milestones(i, 0) = current_x;
        Milestones(i, 1) = current_y;
    }
}

void visualize_milestones()
{
    for (int i = 0; i < NUM_MILESTONES; i++)
    {
        std::cout << "Milestone X: " << Milestones(i, 0) << ",  Y: " << Milestones(i, 1) << std::endl;
    }
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
	
    //XXX: DUMMY CODE
    generate_occupancy_grid();
    generate_milestones();
    visualize_milestones();

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

	 //Draw Curves
         drawCurve(1);
         drawCurve(2);
         drawCurve(4);
    
    	//Main loop code goes here:
    	vel.linear.x = 0.1; // set linear speed
    	vel.angular.z = 0.3; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
