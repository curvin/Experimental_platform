#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <iomanip>

using namespace std;

int NUM_point = 100;
#define hight_init 0.3f

string data_file = "src/simulation/src/position.txt";
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped current_pose;

bool pose_init_done = false;

int poseCount = 0;
float poseArr[500][3] = {
    {0, 0, hight_init}, //起飞

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
}

void load_point(void)
{
    ifstream infile;
    infile.open(data_file.c_str());
    for (int i = 1; i < 500; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            infile >> poseArr[i][j];
        }
    }
    infile.close();

    for (int i = 1; i < 500; i++)
    {
        if (poseArr[i][0] == 0 && poseArr[i][1] == 0 && poseArr[i][2] == 0)
        {
            NUM_point = i;
            break;
        }
    }

    for (int i = 0; i < NUM_point; i++)
    {
        cout << "{ ";
        for (int j = 0; j < 3; j++)
        {
            cout << std::right << setw(7) << poseArr[i][j] << ", ";
        }
        cout << "}\n";
    }
    ROS_INFO("NUM_point is %d", NUM_point);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/UAV_01/pose", 1000, position_cb);

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

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {

        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
