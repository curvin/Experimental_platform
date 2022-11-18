#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;

#define hight_init 0.3f

geometry_msgs::PoseStamped attitude;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped current_pose;

geometry_msgs::Twist velocity;
bool pose_init_done = false;

int NUM_point = 0;
int poseCount = 0;
double poseArr[][3] = {
    {0, 0, hight_init}, //起飞
    {0, 0, hight_init}, //回到坐标原点
    //从这开始执行飞行任务
};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

float ABS(float num)
{
    if (num < 0)
    {
        return -num;
    }
    else
    {
        return num;
    }
}

void Quaterniond2Euler(const double x, const double y, const double z, const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    cout << "Quaterniond2Euler result is:" << endl;
    cout << "x = " << euler[2] * 57.3 << endl;
    cout << "y = " << euler[1] * 57.3 << endl;
    cout << "z = " << euler[0] * 57.3 << endl
         << endl;
}

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    cout << "Euler2Quaternion result is:" << endl;
    cout << "x = " << q.x() << endl;
    cout << "y = " << q.y() << endl;
    cout << "z = " << q.z() << endl;
    cout << "w = " << q.w() << endl
         << endl;
    return q;
}

void position_cb(const geometry_msgs::PoseStamped::ConstPtr &position_now)
{
    current_pose = *position_now;
    if (ABS(position_now->pose.position.x - poseArr[poseCount][0]) < 0.1 && ABS(position_now->pose.position.y - poseArr[poseCount][1]) < 0.1 && ABS(position_now->pose.position.z - poseArr[poseCount][2]) < 0.1)
    {
        poseCount++;
        if (poseCount < NUM_point)
        {
            pose.pose.position.x = poseArr[poseCount][0];
            pose.pose.position.y = poseArr[poseCount][1];
            pose.pose.position.z = poseArr[poseCount][2];

            ROS_INFO("Num %d Position Setted", poseCount);
            ROS_INFO("x:%f y:%f z:%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        }
        else
        {
            ROS_INFO("Mission accomplished !!!");
        }
    }
    if (!pose_init_done && position_now->pose.position.x != 0 && position_now->pose.position.y != 0 && position_now->pose.position.z != 0)
    {
        pose.pose.position.x = position_now->pose.position.x;
        pose.pose.position.y = position_now->pose.position.y;
        pose.pose.position.z = hight_init;

        poseArr[0][0] = pose.pose.position.x;
        poseArr[0][1] = pose.pose.position.y;
        poseArr[0][2] = hight_init;

        pose_init_done = true;
        ROS_INFO("position init x:%f y:%f,z:%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }

    // ROS_INFO("x:%f y:%f z:%f", position_now->pose.position.x, position_now->pose.position.y, position_now->pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_angle_control_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher attitude_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/UAV_01/pose", 1000, position_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    NUM_point = sizeof(poseArr) / sizeof(poseArr[0]);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected && pose_init_done)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV connected");

    Eigen::Quaterniond attitude_Quaternion = euler2Quaternion(0, 0, 0);

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    attitude.pose.orientation.x = attitude_Quaternion.x();
    attitude.pose.orientation.y = attitude_Quaternion.y();
    attitude.pose.orientation.z = attitude_Quaternion.z();
    attitude.pose.orientation.w = attitude_Quaternion.w();

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {

         velocity.angular.z = 8;
        velocity.linear.x = 0.2;

        velocity_pub.publish(velocity);
        // local_pos_pub.publish(pose);

        // attitude_pub.publish(attitude);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
