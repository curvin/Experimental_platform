#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <fstream>
#include <typeinfo>
#include <iomanip>

#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace Eigen;

int NUM_point = 1000;
#define hight_init 0.3f

string data_file = "src/experiments/src/attitude.txt";
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped current_pose;

bool pose_init_done = false;

float angle = 0;
int poseCount = 0;
int poseCount_old = 0;
float poseArr[500][4] = {
    {0, 0, hight_init, 0}, //起飞

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

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;

    return q;
}

void keyboard_cb(const std_msgs::Bool::ConstPtr &keyboard_now)
{

    if (keyboard_now->data)
    {
        poseCount++;
        if (poseCount < NUM_point)
        {
            pose.pose.position.x = poseArr[poseCount][0];
            pose.pose.position.y = poseArr[poseCount][1];
            pose.pose.position.z = poseArr[poseCount][2];
            angle = poseArr[poseCount][3];

            ROS_INFO("Num %d Position Setted", poseCount);
            ROS_INFO("x:%f y:%f z:%f angle:%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, angle);

            Eigen::Quaterniond angel = euler2Quaternion(0, 0, angle / 180 * 3.14);

            pose.pose.orientation.w = angel.w();
            pose.pose.orientation.x = angel.x();
            pose.pose.orientation.y = angel.y();
            pose.pose.orientation.z = angel.z();
        }
        else
        {
            ROS_INFO("Mission accomplished !!!");
        }
    }
}

void position_cb(const geometry_msgs::PoseStamped::ConstPtr &position_now)
{
    current_pose = *position_now;

    if (!pose_init_done && position_now->pose.position.x != 0 && position_now->pose.position.y != 0 && position_now->pose.position.z != 0)
    {
        pose.pose.position.x = position_now->pose.position.x;
        pose.pose.position.y = position_now->pose.position.y;
        pose.pose.position.z = hight_init;

        poseArr[0][0] = pose.pose.position.x;
        poseArr[0][1] = pose.pose.position.y;
        poseArr[0][2] = hight_init;
        poseArr[0][3] = 0;

        pose_init_done = true;
        ROS_INFO("position init x:%f y:%f,z:%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }
}

void load_point(void)
{
    ifstream infile;
    infile.open(data_file.c_str());
    for (int i = 1; i < 500; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            infile >> poseArr[i][j];
        }
    }
    infile.close();

    for (int i = 1; i < 500; i++)
    {
        if (poseArr[i][0] == 0 && poseArr[i][1] == 0 && poseArr[i][2] == 0 && poseArr[i][3] == 0)
        {
            NUM_point = i;
            break;
        }
    }

    for (int i = 0; i < NUM_point; i++)
    {
        cout << "{ ";
        for (int j = 0; j < 4; j++)
        {
            cout << std::right << setw(7) << poseArr[i][j] << ", ";
        }
        cout << "}\n";
    }
    ROS_INFO("NUM_point is %d", NUM_point);
}

int main(int argc, char **argv)
{
    std::string uav_id;
    if(argc!=2)
    {
        printf("ERROR!!!\r\n");
        printf("ERROR!!!\r\n");
        printf("ERROR!!!\r\n");
        printf("Please input uav id!!!\r\n");
        return 1;
    }
    else
    {
        uav_id=argv[1];
    }
    ros::init(argc, argv, uav_id+"_attitude_step_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(uav_id+"/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_id+"/mavros/setpoint_position/local", 10);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>(uav_id+"/mavros/local_position/pose", 10, position_cb);
    ros::Subscriber keyboard_sub = nh.subscribe<std_msgs::Bool>("uav/keyboard", 10, keyboard_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    load_point();

    // wait for FCU connection
    while (ros::ok() && !current_state.connected && pose_init_done)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV connected");

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {

        ROS_INFO("current-> x:%f y:%f z:%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        ROS_INFO("set    -> x:%f y:%f z:%f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
