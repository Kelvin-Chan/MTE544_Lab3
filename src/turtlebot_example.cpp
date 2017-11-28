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
#include <Eigen/Sparse>
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
#define num_milestones 300
#define wp_radius_tol 0.25	// 0.25 m radius tolerance for waypoints

// Controller Configurations
float K_P = 1;		// Proportional gain
float K_I = 1;		// Integral gain
float K_D = 1;		// Derivative gain
float period = 0.05;		// Discrete Time period

#define DEBUG_MODE 1
#define SIMULATION 1
static const int wp_sequence[] = {1,3,1,2,3};

// ------------------------------------------------------------------
// Global Variables
// ------------------------------------------------------------------

// ROS Publisher and Messages
ros::Publisher velocity_publisher, marker_pub;
visualization_msgs::Marker waypoints;

// Map declaration
// Map is stored as RowMajor array
Matrix<int,Dynamic,Dynamic,RowMajor> grid_map;

// Milestones (Col0: X, Col1: Y)
Matrix<int, num_milestones, 2, RowMajor> Milestones;
Matrix<bool, num_milestones, num_milestones, RowMajor> MilestoneEdges;
// Matrix<double, num_milestones, num_milestones, RowMajor> EdgeDistances;
SparseMatrix<double> EdgeDistances(num_milestones, num_milestones);


// Waypoints [x[m], y[m], θ[rad]]
#ifdef SIMULATION
float wp1 [] = {3.5, 0.0, 0.0};
float wp2 [] = {7.5, 0.0, 3.14};
float wp3 [] = {7.5, -3.5, -1.57};
#else
float wp1 [] = {4.0, 0.0, 0.0};
float wp2 [] = {8.0, -4.0, 3.14};
float wp3 [] = {8.0, 0.0, -1.57};
#endif
float *wp_list[] = {wp1, wp2, wp3};

// Path planning represented as list of waypoints (x,y,theta)
deque<float> x_wp_list, y_wp_list, theta_wp_list;
deque<int> x_d_wp_list, y_d_wp_list;

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

bool initial_pose_received = false;
bool initial_map_received = false;
double initial_x;
double initial_y;
// ------------------------------------------------------------------


// ------------------------------------------------------------------
// Macros
#ifdef SIMULATION
#define C2D_DISTANCE_X(x, y) int((y + 5.0) * 100.0/10.0)
#define C2D_DISTANCE_Y(x, y) int((x + 1.0) * 100.0/10.0)
#define D2C_DISTANCE_X(x, y) double((y*10.0/100.0) - 1.0)
#define D2C_DISTANCE_Y(x, y) double((x*10.0/100.0) - 5.0)
#else
#define C2D_DISTANCE_X(x, y) 0 //int(x * 100.0/10.0)
#define C2D_DISTANCE_Y(x, y) 0 //int(y * 100.0/10.0)
#define D2C_DISTANCE_X(x, y) 0 //double(x * 10.0/100.0)
#define D2C_DISTANCE_Y(x, y) 0 //double(y * 10.0/100.0)
#endif
// ------------------------------------------------------------------
void perform_prm(); // prototype

// ------------------------------------------------------------------
// Functions
// ------------------------------------------------------------------

//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg) {
	// This function is called when a new position message is received
	X = msg.pose.pose.position.x; // Robot X psotition
	Y = msg.pose.pose.position.y; // Robot Y psotition
 	Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

    if (!initial_pose_received)
    {
        initial_pose_received = true;
        initial_x = X;
        initial_y = Y;

        if (initial_map_received)
        {
            perform_prm();
        }
    }

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
void generate_milestones() {
    // generate NUM_MILESTONES and only store them if they
    // do not coincide with an objects, otherwise retry

    int current_x;
    int current_y;

    std::cout << "\% Generating Milestones"<< std::endl;

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

    // overwrite first four positions with initial IPS and 3 Waypoints
    // XXX: Placeholder for IPS x,y
    for (int i = 1; i < 4; i++)
    {
        Milestones(i, 0) = C2D_DISTANCE_X(wp_list[i-1][0], wp_list[i-1][1]);
        Milestones(i, 1) = C2D_DISTANCE_Y(wp_list[i-1][0], wp_list[i-1][1]);
    }

    Milestones(0, 0) = C2D_DISTANCE_X(initial_x, initial_y);
    Milestones(0, 1) = C2D_DISTANCE_Y(initial_x, initial_y);

}

void visualize_milestones()
{
    std::cout << "milestones = [";
    for (int i = 0; i < num_milestones; i++)
    {
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
    std::cout << "\% Generating Edges"<< std::endl;
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

            if (MilestoneEdges(i,j))
            {
                EdgeDistances.insert(i,j) = edge_length(i,j);
                EdgeDistances.insert(j,i) = EdgeDistances.coeff(i,j);
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


// returns index of smallest double in vec
int find_vec_min(std::vector<double> &vec)
{
    int min_ind = 0;
    double min_val = vec[0];
    for (int i = 1; i < vec.size(); i++)
    {
        if (vec[i] < min_val)
        {
            min_ind = i;
            min_val = vec[i];
        }
    }
    return min_ind;
}

// look for val in vec and return its index
int find_vec_ind(std::vector<int> &vec, int val)
{
    for (int i = 0; i < vec.size(); i++)
    {
        if (vec[i] == val)
        {
            return i;
        }
    }
    return -1;
}


// Implementation of A* algorithm
// start - index of start milestone
// finish - index of finish milestone
void a_star_algorithm(int start, int finish)
{
    bool done = false;
    double dmax = edge_length(start, finish);// EdgeDistances(start,finish)

    std::vector<int>    open_node;
    std::vector<int>    open_backtrack;
    std::vector<double> open_lower_cost;
    std::vector<double> open_current_cost;

    std::vector<int>    closed_node;
    std::vector<int>    closed_backtrack;
    std::vector<double> closed_lower_cost;
    std::vector<double> closed_current_cost;

    open_node.push_back(start);
    open_backtrack.push_back(-1);
    open_lower_cost.push_back(dmax);
    open_current_cost.push_back(0.0);

    int best;
    int best_milestone;
    int i;
    double dtogo;
    double dcurr;
    int temp_open_node_index;

    // Main Algorithm
    while(!done)
    {
        if (open_node.empty())
        {
            // XXX
            return;
        }

        // find node with lowest cost node
        best = find_vec_min(open_lower_cost);

        // move node to close set
        closed_node.push_back(open_node[best]);
        closed_backtrack.push_back(open_backtrack[best]);
        closed_lower_cost.push_back(open_lower_cost[best]);
        closed_current_cost.push_back(open_current_cost[best]);

        open_node.erase(open_node.begin() + best);
        open_backtrack.erase(open_backtrack.begin() + best);
        open_lower_cost.erase(open_lower_cost.begin() + best);
        open_current_cost.erase(open_current_cost.begin() + best);

        best_milestone = closed_node.back();
        // Check for end condition
        if (best_milestone == finish)
        {
            done = 1;
            continue;
        }

        // interate over connected milestones
        for (i = 0; i < num_milestones; i++)
        {
            // check for edge
            if (MilestoneEdges(best_milestone, i))
            {
                // if milestone in closed set, continue
                if (find_vec_ind(closed_node, i) != -1)
                {
                    continue;
                }
                dtogo = edge_length(i, finish);
                dcurr = closed_current_cost.back() + EdgeDistances.coeff(best_milestone, i);

                // if connect milestone in open set, update distance
                // otherwise append it to open set
                temp_open_node_index = find_vec_ind(open_node, i);
                if (temp_open_node_index == -1)
                {
                    open_node.push_back(i);
                    open_backtrack.push_back(best_milestone);
                    open_lower_cost.push_back(dtogo+dcurr);
                    open_current_cost.push_back(dcurr);
                }
                else
                {
                    open_backtrack[temp_open_node_index] = best_milestone;
                    open_lower_cost[temp_open_node_index] = dtogo+dcurr;
                    open_current_cost[temp_open_node_index] = dcurr;
                }

            }
        }

    }

    // Backtrack and find final path
    int current_milestone_close_node_ind;
    int prev;
    int current_milestone = finish;

    while (current_milestone != -1)
    {
        current_milestone_close_node_ind = find_vec_ind(closed_node, current_milestone);
        prev = closed_backtrack[current_milestone_close_node_ind];

        x_d_wp_list.push_front(Milestones(current_milestone, 0));
        y_d_wp_list.push_front(Milestones(current_milestone, 1));

        current_milestone = prev;
    }
}


void generate_path()
{
    int num_wp = sizeof(wp_sequence)/sizeof(wp_sequence[0]);
    for (int i = (num_wp - 2); i >= 0; i--)
    {
        std::cout << "\% Generating Path " << (i+1) << " For: "<< wp_sequence[i] << " - " << wp_sequence[(i+1)] << std::endl;
        a_star_algorithm(wp_sequence[i], wp_sequence[(i+1)]);
    }

    std::cout << "\% Generating Path " << 0 << " For: IPS - " << wp_sequence[0] << std::endl;
    a_star_algorithm(0, wp_sequence[0]);

}

void visualize_path()
{
    std::cout << "path = [";
    for (int i = 0; i < x_d_wp_list.size(); i++)
    {
        std::cout << x_d_wp_list[i] << " " << y_d_wp_list[i] << "; ";
    }
    std::cout << "];" << std::endl;

    std::cout << "waypoints = [";
    for (int i = 0; i < 3; i++)
    {
        std::cout << Milestones(i+1, 0) << " " << Milestones(i+1, 1) << "; ";
    }
    std::cout << "];" << std::endl;
}


// Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg) {
    // Copy msg map data into grid map data
	copy(msg.data.data(), msg.data.data() + map_size, grid_map.data());
    initial_map_received = true;

    if (initial_pose_received)
    {
        perform_prm();
    }
}

void perform_prm()
{
	generate_milestones();
    if (DEBUG_MODE) {
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

// Callback function for the localization
void localization_callback(const geometry_msgs::PoseStamped& msg) {
	X_l = msg.pose.position.x;
	Y_l = msg.pose.position.y;
	Yaw_l = tf::getYaw(msg.pose.orientation);
}

// Generate velocity commands based on planned waypoints, current robot pose
void velocity_control_update() {
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
	 float path_heading = atan2((y_target - y_prev),(x_target - x_prev));

	 // Heading between robot position & target waypoint [rad]
	 float robot_heading = atan2((y_target - Y),(x_target - X));

	 // Update r, y, e; keeping constant history size
	 r.push_front(path_heading);
	 r.pop_back();

	 y.push_front(robot_heading);
	 y.pop_back();

	 e.push_front(r[0] - y[0]);
	 e.pop_back();

	 // Update u with difference equation in terms of K_P, K_I, K_D, T
	 u.push_front(e[0]*(K_P + K_I*period/2 + 2*(K_D/period)) +
	 							e[1]*(K_I*period - 4*K_D/period) +
								e[2]*(-K_P + K_I*period/2 + 2*K_D/period) + u[1]);
	 u.pop_back();

	 // Can only command turtlebot for linear x & angular z
	 // Constant linear velocity
	 vel.linear.x = 0.1;
	 vel.angular.z = u[0];
	  // -------------------------------------------------------------------------
 }

// Visualize waypoints on RViz
void waypointVisualization () {
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

int main(int argc, char **argv) {
	// Initialize map
	grid_map = 50*MatrixXi::Ones(map_height, map_width);

	// Initialize control variables
	// Second-order controller; history size of 3
	for (int a = 0; a < 3; a++) {
		r.push_back(0);
		e.push_back(0);
		u.push_back(0);
		y.push_back(0);
	}

	// Initialize the ROS framework
	ros::init(argc,argv,"main_control");
	ros::NodeHandle n;

	// Subscribe to the desired topics and assign callbacks
	ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
	ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
	ros::Subscriber localization_sub = n.subscribe("/localization_output", 1, localization_callback);

	// Setup topics to Publish from this node
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

	// Wait until path has been generated
	// <PLACEHOLDER - INSERT CODE HERE>

	// Visualize the three given waypoints
	// waypointVisualization();

	// Set control loop refresh rate to 20 Hz
	ros::Rate control_loop_rate(20);    // 20Hz update rate

	while (ros::ok())	{
		control_loop_rate.sleep(); // Maintain the loop rate
		ros::spinOnce();   // Check for new messages

		// Publish visualization markers for target waypoints
		// marker_pub.publish(waypoints);

		// Main loop code goes here:
		control_loop_rate.sleep();
		// velocity_control_update();

		// velocity_publisher.publish(vel); // Publish the command velocity
	}

	return 0;
}
