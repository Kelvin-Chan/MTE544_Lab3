//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 3
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

// Initial Libraries
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Additional Libraries
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <stdint.h>

// Namespace
using namespace std;
using namespace Eigen;

ros::Publisher marker_pub;

#define TAGID 0

// Configurations
#define map_height 100
#define map_width 100
#define map_size 10000
#define num_milestones 200
#define wp_radius_tol 0.25	// 0.25 m radius tolerance for waypoints

#define DEBUG_MODE 1
static const int wp_sequence[] = {1,3,1,2};

// ------------------------------------------------------------------
// Global Variables

// Map declaration
// Map is stored as RowMajor array
Matrix<int,Dynamic,Dynamic,RowMajor> grid_map;

// Milestones (Col0: X, Col1: Y)
MatrixXd Milestones = MatrixXd::Zero(num_milestones, 2);
Matrix<bool, num_milestones, num_milestones ,RowMajor> MilestoneEdges;


// Waypoints [x[m], y[m], θ[rad]]
float wp1 [] = {4.0, 0.0, 0.0};
float wp2 [] = {8.0, -4.0, 3.14};
float wp3 [] = {8.0, 0.0, -1.57};
float *wp_list[] = {wp1, wp2, wp3};

// Path planning represented as list of waypoints (x,y,theta)
queue<float> x_wp_list, y_wp_list, theta_wp_list;

// Robot Position
double X, Y, Yaw;

// Past and current target waypoint
float x_old, y_old, theta_old;
float x_target, y_target, theta_target;

// Velocity control variable
geometry_msgs::Twist vel;
// ------------------------------------------------------------------


// ------------------------------------------------------------------
// Macros
#define D2C_DISTANCE(x) double(x*25.0/100.0 - (25.0/2.0))
#define C2D_DISTANCE(x) int((x + (25.0/2.0)) * 100.0/25.0)
// ------------------------------------------------------------------

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg) {
	// This function is called when a new position message is received
	X = msg.pose.pose.position.x; // Robot X psotition
	Y = msg.pose.pose.position.y; // Robot Y psotition
 	Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	// std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}

//Example of drawing a curve
void drawCurve(int k) {
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

short sgn(int x) {
    // cout << "Called sgn" << endl;
    return x >= 0 ? 1 : -1;
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {
    // cout << "Called bresenham" << endl;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

// generate milestone points and remove point on obstacles
void generate_milestones()
{
    // generate NUM_MILESTONES and only store them if they
    // do not coincide with an objects, otherwise retry

    int current_x;
    int current_y;

    for (int i = 0; i < num_milestones; i++)
    {
        while (true) {
            current_x = rand() % map_width;
            current_y = rand() % map_height;
            if (grid_map(current_x, current_y) == 0) {
                break;
            }
        }
        Milestones(i, 0) = current_x;
        Milestones(i, 1) = current_y;
    }
}

void visualize_milestones()
{
    std::cout << "milestones = [";
    for (int i = 0; i < num_milestones; i++)
    {
        // std::cout << "Milestone X: " << Milestones(i, 0) << ",  Y: " << Milestones(i, 1) << std::endl;
        std::cout << Milestones(i, 0) << " " << Milestones(i, 1) << "; ";
    }
    std::cout << "];" << std::endl;
}


// return euclidean distance between two milestones of indices i and j
double edge_length(int i, int j)
{
    return sqrt(double(
        pow((Milestones(i,0) - Milestones(j,0)), 2) +
        pow((Milestones(i,1) - Milestones(j,1)), 2)
    ));
}

void generate_edges()
{
    // MilestoneEdges
    int i, j;

    // initialization
    for (i = 0; i < num_milestones; i++)
    {
        for (j = 0; j < num_milestones; j++)
        {
            MilestoneEdges(i,j) = true;
        }
    }

    // for comparing again occupancy grid_map
    int temp_x;
    int temp_y;

    // avoiding duplication of work
    for (i = 0; i < num_milestones; i++)
    {
        for (j = 0; j < i; j++)
        {
            // if milestones overlap ignore edge
            if ((Milestones(i, 0) == Milestones(j, 0)) and (Milestones(i, 1) == Milestones(j, 1)))
            {
                MilestoneEdges(i,j) = false;
                MilestoneEdges(j,i) = false;
                continue;
            }

            std::vector<int> bres_x_coords;
            std::vector<int> bres_y_coords;
            bresenham(
                Milestones(i, 0),
                Milestones(i, 1),
                Milestones(j, 0),
                Milestones(j, 1),
                bres_x_coords,
                bres_y_coords
            );

            while (!bres_x_coords.empty())
            {
                temp_x = bres_x_coords.back();
                temp_y = bres_y_coords.back();
                bres_x_coords.pop_back();
                bres_y_coords.pop_back();

                if (grid_map(temp_x, temp_y) != 0)
                {
                    MilestoneEdges(i,j) = false;
                    MilestoneEdges(j,i) = false;
                    break;
                }
            }
        }
    }
}

void visualize_edges()
{
    int i, j;
    std::cout << "edges = [";
    for (i = 0; i < num_milestones; i++)
    {
        for (j = 0; j < num_milestones; j++)
        {
            std::cout << int(MilestoneEdges(i,j)) << " ";
        }
        std::cout << "; ";
    }
    std::cout << "];" << std::endl;
}


void generate_path_partial(float X0, float Y0, float X1, float Y1)
{
    std::cout << "\% Routing (" << X0 << "," << Y0 << ") - (" << X1 << "," << Y1 << ")" << std::endl;
    int x0 = C2D_DISTANCE(X0);
    int y0 = C2D_DISTANCE(Y0);
    int x1 = C2D_DISTANCE(X1);
    int y1 = C2D_DISTANCE(Y1);

}

void generate_path()
{
    int num_wp = sizeof(wp_sequence)/sizeof(wp_sequence[0]);
    for (int i = 0; i < (num_wp - 1); i++)
    {
        std::cout << "\% Generating Path " << i << " For: "<< wp_sequence[i] << " - " << wp_sequence[(i+1)] << std::endl;
        generate_path_partial(
            wp_list[(wp_sequence[i]-1)][0],
            wp_list[(wp_sequence[i]-1)][1],
            wp_list[(wp_sequence[i+1]-1)][0],
            wp_list[(wp_sequence[i+1]-1)][1]
        );
    }
}

void visualize_path()
{
}


// Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg) {
    // Copy msg map data into grid map data
	copy(msg.data.data(), msg.data.data() + map_size, grid_map.data());

	generate_milestones();
    if (DEBUG_MODE)
    {
        visualize_milestones();
    }

    generate_edges();
    if (DEBUG_MODE)
    {
        visualize_edges();
    }

    generate_path();
    if (DEBUG_MODE)
    {
        visualize_path();
    }
}

// Generate velocity commands based on planned waypoints, current robot pose
 void velocity_control_update() {
	 // Distance between robot position and target waypoint
	 float dist = sqrt(pow((x_target - X), 2) + pow((y_target - Y), 2));

	 // If robot position is within waypoint radius tolerance, pop for next
	 if (dist < wp_radius_tol) {
		 // Store previous waypoint
		 x_old = x_target;
		 y_old = y_target;
		 theta_old = theta_target;

		 // Get new waypoint
		 x_target = x_wp_list.front();
		 y_target = y_wp_list.front();
		 theta_target = theta_wp_list.front();

		 // Pop element from queue
		 x_wp_list.pop();
		 y_wp_list.pop();
		 theta_wp_list.pop();
	 }

	 // Use closed-loop controller to correct robot path between two waypoints


	 // Can only command turtlebot for linear x & angular z
	 vel.linear.x = 0;
	 vel.angular.z = 0;
 }

int main(int argc, char **argv) {
	//Initialize map
	grid_map = 50*MatrixXi::Ones(map_height, map_width);

	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate


    while (ros::ok())
    {
    	loop_rate.sleep(); // Maintain the loop rate
    	ros::spinOnce();   // Check for new messages

	 	// Draw Curves
         drawCurve(1);
         drawCurve(2);
         drawCurve(4);

    	// Main loop code goes here:
    	velocity_control_update();

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
