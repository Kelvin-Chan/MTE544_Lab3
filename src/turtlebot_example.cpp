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
#include <math.h>
#include <stdint.h>

// Namespace
using namespace std;
using namespace Eigen;

#define TAGID 0

// ------------------------------------------------------------------
#define DEBUG_MODE 1
#define SIMULATION 1

// Map Configurations
#ifdef SIMULATION
#define map_height 100
#define map_width 100
#define map_size 10000

#else
#define map_height 70
#define map_width 70
#define map_size 4900

#endif

// Waypoint Configurations
#define num_milestones 300
#define wp_radius_tol 0.25	// 0.25 m radius tolerance for waypoints

// Controller Configurations
float K_P = -1;		// Proportional gain
float K_I = 0;		// Integral gain
float K_D = 0;		// Derivative gain
float period = 0.05;		// Discrete Time period

#define DILATION_KERNEL_SIZE 2
static const int wp_sequence[] = {1,3,1,2,3};

// ------------------------------------------------------------------
// Global Variables
// ------------------------------------------------------------------

// ROS Publisher and Messages
ros::Publisher velocity_publisher, marker_pub, occ_map_pub;
visualization_msgs::Marker milestones;
visualization_msgs::Marker waypoints;

// Map declaration
// Map is stored as RowMajor array
Matrix<int,Dynamic,Dynamic,RowMajor> grid_map;
MatrixXd tmp_grid_map = MatrixXd::Zero(map_width, map_height);

// Milestones (Col0: X, Col1: Y)
Matrix<int, num_milestones, 2, RowMajor> Milestones;
SparseMatrix<double> EdgeDistances(num_milestones, num_milestones);
SparseMatrix<bool> MilestoneEdges(num_milestones, num_milestones);


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
int newMsgReceived = 0;
int markersVisualized = 0;
int wpNotGrabbed = 1;

bool initial_pose_received = false;
bool initial_map_received = false;
bool prm_generated = false;
double initial_x, initial_y;
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
    newMsgReceived = 1;

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
            MilestoneEdges.insert(i,j) = true;
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
                MilestoneEdges.coeffRef(i,j) = false;
                MilestoneEdges.coeffRef(j,i) = false;
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
                    MilestoneEdges.coeffRef(i,j) = false;
                    MilestoneEdges.coeffRef(j,i) = false;
                    break;
                }
            }

            if (MilestoneEdges.coeff(i,j))
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
            std::cout << int(MilestoneEdges.coeff(i,j)) << " ";
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
            if (MilestoneEdges.coeff(best_milestone, i))
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
        theta_wp_list.push_front(wp_list[wp_sequence[(i+1)]-1][2]);

        // pad list with -1.0 s
        while (x_d_wp_list.size() > theta_wp_list.size())
        {
            theta_wp_list.push_front(-1.0);
        }
    }


    std::cout << "\% Generating Path " << 0 << " For: IPS - " << wp_sequence[0] << std::endl;
    a_star_algorithm(0, wp_sequence[0]);

    // pad list with -1.0 s
    while (x_d_wp_list.size() > theta_wp_list.size())
    {
        theta_wp_list.push_front(-1.0);
    }

    for (int i = 0; i < x_d_wp_list.size(); i++)
    {
        x_wp_list.push_back(D2C_DISTANCE_X(x_d_wp_list[i],y_d_wp_list[i]));
        y_wp_list.push_back(D2C_DISTANCE_Y(x_d_wp_list[i],y_d_wp_list[i]));
    }
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

    // for (int i = 0; i < x_wp_list.size(); i++)
    // {
    //     std::cout << x_wp_list[i] << " " << y_wp_list[i] << " " << theta_wp_list[i] << endl;
    // }

}


// Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg) {
    occ_map_pub.publish(msg);
    // Copy msg map data into grid map data
	copy(msg.data.data(), msg.data.data() + map_size, grid_map.data());
    initial_map_received = true;


    // perform morphological dilation on map to exagerate obstacles
    int i;
    int j;
    int k;
    int l;

    for (i = 0; i < map_width; i++)
    {
        for (j = 0; j < map_height; j++)
        {
            if (grid_map(i,j) != 0)
            {
                for (k = (i-DILATION_KERNEL_SIZE); k <= (i+DILATION_KERNEL_SIZE); k++)
                {
                    for (l = (j-DILATION_KERNEL_SIZE); l <= (j+DILATION_KERNEL_SIZE); l++)
                    {
                        if ((k >=0) and (k < map_width) and (l >= 0) and (l < map_height))
                        {
                            tmp_grid_map(k,l) += 1;
                        }
                    }
                }
            }
        }
    }

    for (i = 0; i < map_width; i++)
    {
        for (j = 0; j < map_height; j++)
        {
            if (tmp_grid_map(i,j) == 0)
            {
                grid_map(i,j) = 0;
            }
            else
            {
                grid_map(i,j) = 100;
            }
        }
    }


    if (DEBUG_MODE)
    {
        std::cout << "occmap = [";
        for (j = 0; j < map_height; j++)
        {
            for (i = 0; i < map_width; i++)
            {
                std::cout << grid_map(i,j) << " ";
            }
            std::cout << "; ";
        }
        std::cout << "];" << std::endl;
    }

    if (initial_pose_received)
    {
        perform_prm();
    }
}

void perform_prm()
{
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

    prm_generated = true;
}

// Callback function for the localization
void localization_callback(const geometry_msgs::PoseStamped& msg) {
	X_l = msg.pose.position.x;
	Y_l = msg.pose.position.y;
	Yaw_l = tf::getYaw(msg.pose.orientation);
}

// Generate velocity commands based on planned waypoints, current robot pose
void velocity_control_update() {
    newMsgReceived = 0;

    // Return if completed
    if (endReached) {
        return;
    }

    // Distance between robot position and target waypoint
    float robot_dist = sqrt(pow((double) (x_target - X), 2) + pow((double) (y_target - Y), 2));

    // If robot position is within waypoint radius tolerance
    // Pop deque for next waypoint
    if (robot_dist < wp_radius_tol) {

        if (DEBUG_MODE) {
            cout << "Milestone Reached" << endl;
			cout << "Milestone List: " << x_wp_list.size() << " remaining" << endl;
        }

        // Check if target yaw is specified and not yet reached
		// Rotate to target yaw, and when tolerance is reached, next waypoint
        if (theta_target != -1 && abs(theta_target - Yaw) > 0.01) {
            if (DEBUG_MODE) {
                cout << "Adjusting yaw to waypoint target" << endl;
            }

			// Rotating to target yaw
            vel.linear.x = 0;
            vel.angular.z = K_P*(theta_target - Yaw);
            return;
        } else {
            // Store previous waypoint
            x_prev = x_target;
            y_prev = y_target;
            theta_prev = theta_target;

            // Grab new waypoints if they exist
            if (x_wp_list.size() + y_wp_list.size() + theta_wp_list.size() > 0) {
                x_target = x_wp_list[0];
                y_target = y_wp_list[0];
                theta_target = theta_wp_list[0];

                if (DEBUG_MODE) {
                    cout << "New Waypoint: (" << x_target << ", " << y_target << ")" << endl;
                }

                // Pop element from queue
                x_wp_list.pop_front();
                y_wp_list.pop_front();
                theta_wp_list.pop_front();
            } else {
                // End reached, stop robot and exit function
                if (DEBUG_MODE) {
                    cout << "Last waypoint reached" << endl;
                }
                vel.linear.x = 0;
                vel.angular.z = 0;
                endReached = 1;
                return;
            }
        }
    }

    // -------------------------------------------------------------------------
    // HEADING CONTROLLER
    // Robot heading is the reference that the robot yaw is trying to track
    // Using error between robot heading and yaw to control angular velocity
    // -------------------------------------------------------------------------
    // Use closed-loop controller to correct robot path between two waypoints
    // Distance between previous & target waypoints [rad]
    float path_dist = sqrt(pow((double) (x_target - x_prev), 2) + pow((double) (y_target - y_prev), 2));

    // Heading between previous & target waypoints [rad]
    float path_heading = atan2((double) (y_target - y_prev), (double) (x_target - x_prev));

    // Heading between robot position & target waypoint [rad]
    // float robot_heading = atan2((y_target - Y), (x_target - X));
    float robot_heading = atan2((double) (y_target - Y), (double) (x_target - X));

    // Update r, y, e; keeping constant history size
    r.push_front(robot_heading);
    r.pop_back();

    // Yaw offset of 90 degs; Yaw is CCW, Heading is CW from horizontal
    // y.push_front(robot_heading - Yaw + M_PI/2);
    y.push_front(Yaw - M_PI/2);
    y.pop_back();

    e.push_front(cos(r[0] - y[0]));
    e.pop_back();

    // Update u with difference equation in terms of K_P, K_I, K_D, T
    // u.push_front(e[0] * (K_P + K_I * period / 2 + 2 * (K_D / period)) +
    //   e[1] * (K_I * period - 4 * K_D / period) +
    //   e[2] * (-K_P + K_I * period / 2 + 2 * K_D / period) + u[1]);

    // Proportional controller
    u.push_front(K_P*e[0]);
    u.pop_back();

    // Can only command turtlebot for linear x & angular z
    // vel.linear.x = 0.1;

    // Bang-Bang controller for linear velocity
    if (abs(e[0]) < 0.01) {
        vel.linear.x = 0.5;
    } else {
        vel.linear.x = 0;
    }

    vel.angular.z = u[0];

    if (DEBUG_MODE) {
        cout << "------------------------------------------------\n"
             << "path_heading: " << path_heading << " [rad]\n"
             << "robot_heading: " << robot_heading << " [rad]\n"
             << "robot_dist: " << robot_dist << " [m]\n"
             << "e: " << e[0] << " [rad]\n"
             << "u: " << u[0] << " [rad/s]\n"
             << "Robot X: " << X << " [m]\n"
             << "Robot Y: " << Y << " [m]\n"
             << "Robot Yaw: " << Yaw << " [rad]\n"
             << "Current Target: (" << x_target << ", " << y_target << ")\n"
             << "------------------------------------------------\n" << endl;
    }

    // -------------------------------------------------------------------------
}

// Visualize waypoint on RViz
void waypointsVisualization() {
    waypoints.header.frame_id = "/map";
    waypoints.header.stamp = ros::Time::now();
    waypoints.ns = "points_and_lines";
    waypoints.type = visualization_msgs::Marker::POINTS;
    waypoints.action = visualization_msgs::Marker::ADD;
    waypoints.lifetime = ros::Duration();

    waypoints.id = 1;
    waypoints.color.r = 0.0;
    waypoints.color.g = 1.0;
    waypoints.color.b = 0.0;
    waypoints.color.a = 0.5;

    waypoints.scale.x = 0.5;
    waypoints.scale.y = 0.5;
    waypoints.scale.z = 0.5;


    geometry_msgs::Point p1;
    p1.x = wp1[0];
    p1.y = wp1[1];
    p1.z = 0;

    geometry_msgs::Point p2;
    p2.x = wp2[0];
    p2.y = wp2[1];
    p2.z = 0;

    geometry_msgs::Point p3;
    p3.x = wp3[0];
    p3.y = wp3[1];
    p3.z = 0;

    waypoints.points.push_back(p1);
    waypoints.points.push_back(p2);
    waypoints.points.push_back(p3);
}

// Visualize milestones on RViz
void milestonesVisualization() {
    milestones.header.frame_id = "/map";
    milestones.header.stamp = ros::Time::now();
    milestones.ns = "points_and_lines";
    milestones.type = visualization_msgs::Marker::LINE_STRIP;
    milestones.action = visualization_msgs::Marker::ADD;
    milestones.lifetime = ros::Duration();

    milestones.id = 0;
    milestones.color.r = 0.0;
    milestones.color.g = 0.0;
    milestones.color.b = 1.0;
    milestones.color.a = 0.5;

    milestones.scale.x = 0.2;
    milestones.scale.y = 0.2;
    milestones.scale.z = 0.2;

    // Build list of points to show
    for (int a = 0; a < x_wp_list.size(); a++) {
        geometry_msgs::Point p;
        p.x = x_wp_list[a];
        p.y = y_wp_list[a];
        p.z = 0;
        milestones.points.push_back(p);
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
    occ_map_pub = n.advertise < nav_msgs::OccupancyGrid > ("occ_grid", 1, true);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

    // Switch between /indoor_pos and /local_pose
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    // Setup topics to Publish from this node
    velocity_publisher = n.advertise < geometry_msgs::Twist > ("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise < visualization_msgs::Marker > ("visualization_marker", 1, true);

    // Set control loop refresh rate to 20 Hz
    ros::Rate control_loop_rate(20); // 20Hz update rate

    // ----------------------------------------------------
    // CONTROLS TESTING ONLY; REMOVE WHEN IMPLEMENTING
    // x_prev = 0;
    // y_prev = 0;
    // theta_prev = 0;
    //
    // // (3,3)
    // x_target = 3;
    // y_target = 3;
    // theta_target = 0.5;
    //
    // // (5, 3)
    // x_wp_list.push_back(5);
    // y_wp_list.push_back(3);
    // theta_wp_list.push_back(0.5);
    //
    // // (1, -3)
    // x_wp_list.push_back(1);
    // y_wp_list.push_back(-3);
    // theta_wp_list.push_back(0.5);
    // ----------------------------------------------------

    while (ros::ok()) {
        // control_loop_rate.sleep(); // Maintain the loop rate
        ros::spinOnce(); // Check for new messages

        if (prm_generated) {
            // Grab first iteration of prev and target points
            if (wpNotGrabbed) {
                x_prev = x_wp_list[0];
                x_target = x_wp_list[1];
                y_prev = y_wp_list[0];
                y_target = y_wp_list[1];
                theta_prev = theta_wp_list[0];
                theta_target = theta_wp_list[1];

                for (int b = 0; b < 2; b++) {
                    x_wp_list.pop_front();
                    y_wp_list.pop_front();
                    theta_wp_list.pop_front();
                }
                wpNotGrabbed = 0;
            }

            // Visualize once given milestones and waypoints when ready
            if (!markersVisualized) {
                waypointsVisualization();
                milestonesVisualization();
                markersVisualized = 1;
            }

            // Publish visualization markers for target waypoints
            marker_pub.publish(waypoints);
            marker_pub.publish(milestones);

            // Update velocity control when new IPS/localization msg received
            if (newMsgReceived) {
                velocity_control_update();
            }

            velocity_publisher.publish(vel); // Publish the command velocity

            // Main loop code goes here:
        }
        control_loop_rate.sleep();

    }

    return 0;
}
