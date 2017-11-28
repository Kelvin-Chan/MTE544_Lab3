//  ///////////////////////////////////////////////////////////
//
//	MTE 544 lab 3
//
//	Author: Kelvin Chan, Tommy Jung, Nicholas Manna, Lutfia Kathrada
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
#include <deque>
#include <algorithm>
#include <cmath>
#include <stdint.h>

// Namespace
using namespace std;
using namespace Eigen;

#define TAGID 0

// ------------------------------------------------------------------
// Map Configurations
#define map_height 100
#define map_width 100
#define map_size 10000

// Waypoint Configurations
#define num_milestones 200
#define wp_radius_tol 0.25 // 0.25 m radius tolerance for waypoints

// Controller Configurations
float K_P = 1;  // Proportional gain
float K_I = 0.05;  // Integral gain
float K_D = 0.05;  // Derivative gain
float period = 0.05;  // Discrete Time period

#define DEBUG_MODE 1

// ------------------------------------------------------------------
// Global Variables
// ------------------------------------------------------------------

// ROS Publisher and Messages
ros::Publisher velocity_publisher, marker_pub;
visualization_msgs::Marker waypoints;

// Map declaration
// Map is stored as RowMajor array
Matrix<int,Dynamic,Dynamic,RowMajor> grid_map;

// Milestone (Col0: X, Col1: Y)
MatrixXd Milestones = MatrixXd::Zero(num_milestones, 2);

// Waypoints [x[m], y[m], θ[rad]]
float wp1 [] = {4.0, 0.0, 0.0};
float wp2 [] = {8.0, -4.0, 3.14};
float wp3 [] = {8.0, 0.0, -1.57};

// Path planning represented as list of waypoints (x,y,theta)
deque<float> x_wp_list, y_wp_list, theta_wp_list;

// Current Robot Position
// This should be augmented by localization for increased performance
double X, Y, Yaw;

// Localized Robot Position
double X_l, Y_l, Yaw_l;

// Past and current target waypoint
float x_prev, y_prev, theta_prev;
float x_target, y_target, theta_target;

// Controls Variables deque
// The most recent variables are in the first index
deque<float> r, e, u, y;

// Velocity control variable
geometry_msgs::Twist vel;

// Flags
int endReached = 0;
int newMsgReceived = 0;

// ------------------------------------------------------------------

// ------------------------------------------------------------------
// Functions
// ------------------------------------------------------------------

//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg) {
  // This function is called when a new position message is received
  X = msg.pose.pose.position.x; // Robot X psotition
  Y = msg.pose.pose.position.y; // Robot Y psotition
  Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

  newMsgReceived = 1;
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
  lines.color.b = 0.2 * k;
  lines.color.a = 1.0;

  //generate curve points
  for (int i = 0; i < steps; i++) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0; //not used
    lines.points.push_back(p);

    //curve model
    x = x + 0.1;
    y = sin(0.1 * i * k);
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
void bresenham(int x0, int y0, int x1, int y1, std::vector < int > & x, std::vector < int > & y) {
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
    if (s) y0 += sgn(dy2);
    else x0 += sgn(dx2);
    if (d < 0) d += inc1;
    else {
      d += inc2;
      if (s) x0 += sgn(dx2);
      else y0 += sgn(dy2);
    }

    //Add point to vector
    x.push_back(x0);
    y.push_back(y0);
  }
}

// generate milestone points and remove point on obstacles
void generate_milestones() {
  // generate NUM_MILESTONES and only store them if they
  // do not coincide with an objects, otherwise retry

  int current_x;
  int current_y;

  for (int i = 0; i < num_milestones; i++) {
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

void visualize_milestones() {
  // for (int i = 0; i < num_milestones; i++) {
  //     std::cout << "Milestone X: " << Milestones(i, 0) << ",  Y: " << Milestones(i, 1) << std::endl;
  // }
}

// Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid & msg) {
  // Copy msg map data into grid map data
  copy(msg.data.data(), msg.data.data() + map_size, grid_map.data());
  generate_milestones();
  if (DEBUG_MODE) {
    visualize_milestones();
  }
}

// Callback function for the localization
void localization_callback(const geometry_msgs::PoseStamped & msg) {
  X_l = msg.pose.position.x;
  Y_l = msg.pose.position.y;
  Yaw_l = tf::getYaw(msg.pose.orientation);

  newMsgReceived = 1;
}

// Generate velocity commands based on planned waypoints, current robot pose
void velocity_control_update() {
  newMsgReceived = 0;

  // Return if completed
  if (endReached) {
    return;
  }

  // Distance between robot position and target waypoint
  float robot_dist = sqrt(pow((x_target - X), 2) + pow((y_target - Y), 2));

  // If robot position is within waypoint radius tolerance
  // Pop deque for next waypoint
  if (robot_dist < wp_radius_tol) {
    // Store previous waypoint
    x_prev = x_target;
    y_prev = y_target;
    theta_prev = theta_target;

    // Grab new waypoints if they exist
    if (x_wp_list.size() + y_wp_list.size() + theta_wp_list.size() > 0) {
      x_target = x_wp_list[0];
      y_target = y_wp_list[0];
      theta_target = theta_wp_list[0];

      // Pop element from queue
      x_wp_list.pop_front();
      y_wp_list.pop_front();
      theta_wp_list.pop_front();
    } else {
      // End reached, stop robot and exit function
      vel.linear.x = 0;
      vel.angular.z = 0;
      endReached = 1;
      return;
    }
  }

  // -------------------------------------------------------------------------
  // HEADING CONTROLLER
  // Path heading is the reference that the robot heading is trying to track
  // Using error between robot heading and path heading to control yaw
  // Using discretized PID control
  // -------------------------------------------------------------------------
  // Use closed-loop controller to correct robot path between two waypoints
  // Distance between previous & target waypoints [rad]
  float path_dist = sqrt(pow((x_target - x_prev), 2) + pow((y_target - y_prev), 2));

  // Heading between previous & target waypoints [rad]
  float path_heading = atan2((y_target - y_prev), (x_target - x_prev));

  // Heading between robot position & target waypoint [rad]
  float robot_heading = atan2((y_target - Y), (x_target - X));

  // Update r, y, e; keeping constant history size
  r.push_front(path_heading);
  r.pop_back();

  // Yaw offset of 90 degs for heading
  y.push_front(robot_heading + Yaw - M_PI/2);
  y.pop_back();

  e.push_front(r[0] - y[0]);
  e.pop_back();

  // Update u with difference equation in terms of K_P, K_I, K_D, T
  u.push_front(e[0] * (K_P + K_I * period / 2 + 2 * (K_D / period)) +
    e[1] * (K_I * period - 4 * K_D / period) +
    e[2] * (-K_P + K_I * period / 2 + 2 * K_D / period) + u[1]);
  u.pop_back();

  // Can only command turtlebot for linear x & angular z
  // Constant linear velocity
  vel.linear.x = 0.1;
  vel.angular.z = u[0];

  if (DEBUG_MODE) {
	  cout << "------------------------------------------------\n"
	  	<< "path_heading: " << path_heading << " [rad]\n"
	  	<< "robot_heading: " << robot_heading << " [rad]\n"
		<< "u: " << u[0] << " [rad/s]\n"
		<< "------------------------------------------------\n" << endl;
  }

  // -------------------------------------------------------------------------
}

// Visualize waypoints on RViz
void waypointVisualization() {
  waypoints.header.frame_id = "/map";
  waypoints.header.stamp = ros::Time::now();
  waypoints.ns = "points_and_lines";
  waypoints.type = visualization_msgs::Marker::POINTS;
  waypoints.action = visualization_msgs::Marker::ADD;
  waypoints.lifetime = ros::Duration();

  waypoints.id = 0;
  waypoints.color.r = 1.0;
  waypoints.color.g = 0.0;
  waypoints.color.b = 0.0;
  waypoints.color.a = 1.0;

  waypoints.scale.x = 0.5;
  waypoints.scale.y = 0.5;
  waypoints.scale.z = 0.5;

  // Build list of points to show
  for (int a = 0; a < x_wp_list.size(); a++) {
    geometry_msgs::Point p;
    p.x = x_wp_list[a];
    p.y = y_wp_list[a];
    p.z = 0;
    waypoints.points.push_back(p);
  }
}

int main(int argc, char * * argv) {
  // Initialize map
  grid_map = 50 * MatrixXi::Ones(map_height, map_width);

  // Initialize control variables
  // Second-order controller; history size of 3
  for (int a = 0; a < 3; a++) {
    r.push_back(0);
    e.push_back(0);
    u.push_back(0);
    y.push_back(0);
  }

  // Initialize the ROS framework
  ros::init(argc, argv, "main_control");
  ros::NodeHandle n;

  // Subscribe to the desired topics and assign callbacks
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  ros::Subscriber localization_sub = n.subscribe("/localization_output", 1, localization_callback);

  // Setup topics to Publish from this node
  velocity_publisher = n.advertise < geometry_msgs::Twist > ("/cmd_vel_mux/input/navi", 1);
  marker_pub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1, true);

  // Wait until path has been generated
  // <PLACEHOLDER - INSERT CODE HERE>

  // Visualize the three given waypoints
  // waypointVisualization();

  // Set control loop refresh rate to 20 Hz
  ros::Rate control_loop_rate(20); // 20Hz update rate

  // TESTING ONLY; REMOVE WHEN IMPLEMENTING
  x_prev = 0;
  y_prev = 0;
  theta_prev = 0.5;

  x_target = 5;
  y_target = 5;
  theta_target = 0.5;

  while (ros::ok()) {
    control_loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce(); // Check for new messages

    // Publish visualization markers for target waypoints
    // marker_pub.publish(waypoints);

    // Main loop code goes here:
    control_loop_rate.sleep();

    // Update velocity control when new IPS/localization msg received
    if (newMsgReceived) {
      velocity_control_update();
    }

    velocity_publisher.publish(vel); // Publish the command velocity
  }

  return 0;
}
