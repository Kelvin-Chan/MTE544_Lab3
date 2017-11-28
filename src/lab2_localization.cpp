#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#define PI 3.14159265
#define LOOP_RATE 20 // loop rate in Hz
#define PERIOD_TIME 1.0/LOOP_RATE // period time in sec
#define NUM_PARTICLES 100

#define MOTION_DISTURBANCE 0.05
#define MEASUREMENT_DISTURBANCE 1.0
#define NORMPDF_CONST 0.3989422804014327

using namespace Eigen;
using namespace std;

// <0.0, NUM_PARTICLES, 3> Xp; // Particles
MatrixXd X = MatrixXd::Ones(3, NUM_PARTICLES);
MatrixXd Xp = MatrixXd::Zero(3, NUM_PARTICLES);

double vel = 0.0;
double angv = 0.0;

double Y1 = 0.0;
double Y2 = 0.0;
double Y3 = 0.0;

bool new_measurement = false;

//vector<float> cov;
//const double* cov;


/*
   TOPIC CALLBACKS
   */
void ips_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    Y1 = msg->pose.pose.position.x;
    Y2 = msg->pose.pose.position.y;
    Y3 = tf::getYaw(msg->pose.pose.orientation);
    // std::cout << "IPS pose.pose. - pose X:" << Y1 << " Y:" << Y2 << " YAW:" << Y3 << std::endl;
    new_measurement = true;
}

void odom_pose_callback(const nav_msgs::OdometryConstPtr &msg)
{

    vel = msg->twist.twist.linear.x;
    angv = msg->twist.twist.angular.z;
    // std::cout << vel << ' ' << angv << std::endl;
}


/*
   HELPER FUNCTIONS
   */

double normpdf(double X, double mu, double sigma) {
    return (1.0/(sigma*sqrt(2.0*PI)))*exp((-1.0*(X-mu)*(X-mu))/(2.0*sigma*sigma));
}

double normpdf1(double X, double mu) {
    return NORMPDF_CONST*exp(-0.5*(X-mu)*(X-mu));
}

template<typename Derived>
int binarySearch (double target, MatrixBase<Derived>& matrix, int row, int row_length) {
    int index;
    int start = 0;
    int end = row_length-1;
    int mid = start + ((end-start)/2);

    while ((end - start) > 20) {
        mid = start + ((end-start)/2);
        if (matrix(row, start) >= target) {
            return start;
        }
        if (matrix(row, mid) == target) {
            return mid;
        }
        if (matrix(row, end) == target) {
            return end;
        }
        if ((target > matrix(row, start)) && (target < matrix(row, mid))) {
            end = mid;
            continue;
        } else if ((target > matrix(row, mid)) && (target < matrix(row, end))) {
            start = mid;
            continue;
        }
    }

    index = start;
    while (true) {
        if (matrix(row, index) >= target) {
            return index;
        } else {
            index++;
        }
    }
}

// For generating randn values
double surand()
{
    return( (double) rand()/RAND_MAX ); // rand float - 0 to 1
}

double urand(double low, double high)
{
    return(low+(high-low)*surand()); // rand float - low to high
}

double genexp(double lambda)
{
    double u,x;
    u=surand();
    x=(-1/lambda)*log(u);
    return(x); // rand exp
}

double gennor()
{
    double theta,rsq,x;
    theta=urand(0,2*PI);
    rsq=genexp(0.5);
    x=sqrt(rsq)*cos(theta);
    return(x); // rand norm
}

int main(int argc, char **argv)
{
    int i;
    int j;
    MatrixXd w = MatrixXd::Zero(3, NUM_PARTICLES);
    MatrixXd W = MatrixXd::Zero(3, NUM_PARTICLES);
    MatrixXd seed_val = MatrixXd::Zero(1, NUM_PARTICLES);
    MatrixXd state_estimate = MatrixXd::Zero(3, 1);

    //Initialize the ROS framework
    ros::init(argc,argv,"localization");
    ros::NodeHandle n;

    // visualization
    visualization_msgs::Marker truePOS;
    visualization_msgs::Marker estPOS;
    visualization_msgs::Marker particles;

    truePOS.header.frame_id = "/indoor_pos";
    truePOS.header.stamp = ros::Time::now();
    truePOS.ns = "points_and_lines";
    truePOS.action = visualization_msgs::Marker::ADD;
    truePOS.pose.orientation.w = 1.0;

    estPOS.header.frame_id = "/indoor_pos";
    estPOS.header.stamp = ros::Time::now();
    estPOS.ns = "points_and_lines";
    estPOS.action = visualization_msgs::Marker::ADD;
    estPOS.pose.orientation.w = 1.0;

    truePOS.id = 0;
    truePOS.type = visualization_msgs::Marker::POINTS;
    truePOS.color.a = 1.0;
    truePOS.color.r = 1.0f;
    truePOS.scale.x = 0.2;
    truePOS.scale.y = 0.2;

    estPOS.id = 1;
    estPOS.type = visualization_msgs::Marker::POINTS;
    estPOS.color.a = 1.0;
    estPOS.color.g = 1.0f;
    estPOS.scale.x = 0.2;
    estPOS.scale.y = 0.2;

    particles.id = 2;
    particles.type = visualization_msgs::Marker::POINTS;
    particles.color.a = 1.0;
    particles.color.b = 1.0f;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber ips_pose_sub = n.subscribe("/indoor_pos", 1, ips_pose_callback);
    ros::Subscriber odom_pose_sub = n.subscribe("/odom", 1, odom_pose_callback);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher local_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_pose", 1);

    //Set the loop rate
    ros::Rate loop_rate(LOOP_RATE);    //20Hz update rate

    //Initialize Particles
    for (i = 0; i < NUM_PARTICLES; i++) {
        X(0, i) = urand(-2.5, 2.5);
        X(1, i) = urand(-2.5, 2.5);
        X(2, i) = urand(-PI, PI);
    }

    std::cout << "init complete\n";

    ros::Time time_step = ros::Time::now();
    ros::Time prev_time_step = ros::Time::now();

    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages

        prev_time_step = time_step;
        time_step = ros::Time::now();
        std::cout << (time_step-prev_time_step) << std::endl;

        //Main loop code goes here:
        for (i = 0; i < NUM_PARTICLES; i++) {
            Xp(0, i) = X(0, i) + vel*cos(X(2, i))*PERIOD_TIME + (gennor() * 0.3162);
            Xp(1, i) = X(1, i) + vel*sin(X(2, i))*PERIOD_TIME + (gennor() * 0.3162); 
            Xp(2, i) = X(2, i) + angv*PERIOD_TIME + (gennor() * 0.2236);

            w(0, i) = normpdf1(Y1, Xp(0, i));
            w(1, i) = normpdf1(Y2, Xp(1, i));
            w(2, i) = normpdf1(Y3, Xp(2, i));

            for (j = 0; j < 3; j++) {
                if (i == 0) {
                    W(j, i) = w(j, i);
                } else {
                    W(j, i) = W(j, i-1) + w(j, i);
                }
            }

        }

        state_estimate = MatrixXd::Zero(3, 1);
        for (j = 0; j < 3; j++) {
            seed_val = MatrixXd::Random(1, NUM_PARTICLES) * W(j, (NUM_PARTICLES-1));
            for (i = 0; i < NUM_PARTICLES; i++) {
                // X(j,i) = Xp(j, binarySearch(seed_val(0, i), W, j, NUM_PARTICLES));
                if (new_measurement)
                {
                    X(j,i) = Xp(j, binarySearch(surand()*W(j, (NUM_PARTICLES - 1)), W, j, NUM_PARTICLES));
                    new_measurement = false;
                }
                else
                {
                    X(j,i) = Xp(j,i);
                }
                state_estimate(j) += X(j,i);
            }
            state_estimate(j) /= double(NUM_PARTICLES);
        }

        cout << abs(Y1 - state_estimate(0)) << " - " << abs(Y2 - state_estimate(1)) << endl;

        geometry_msgs::Point pt;
        pt.x = Y1;
        pt.y = Y2;
        pt.z = 1.0;
        truePOS.points.push_back(pt);
        marker_pub.publish(truePOS);

        geometry_msgs::Point et;
        et.x = state_estimate(0);
        et.y = state_estimate(1);
        et.z = 2.0;
        estPOS.points.push_back(et);
        marker_pub.publish(estPOS);


        geometry_msgs::PoseWithCovarianceStamped lt;
        lt.pose.pose.position.x = state_estimate(0);
        lt.pose.pose.position.y = state_estimate(1);
        lt.pose.pose.orientation.w = tf::createQuaternionFromYaw(state_estimate(2)).w();
        lt.pose.pose.orientation.x = tf::createQuaternionFromYaw(state_estimate(2)).x();
        lt.pose.pose.orientation.w = tf::createQuaternionFromYaw(state_estimate(2)).y();
        lt.pose.pose.orientation.z = tf::createQuaternionFromYaw(state_estimate(2)).z();
        local_pose_pub.publish(lt);
    }

    return 0;
}
