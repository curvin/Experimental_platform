#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

#define hight_init 0.3f

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped current_pose;

bool pose_init_done = false;

int NUM_point = 0;
int poseCount = 0;
double poseArr[][3] = {
    {0, 0, hight_init}, //起飞

    //从这开始执行飞行任务
    -0.56 - 0.875 0},
       -0.455 - 0.7 1.5
}
,
    0.07 - 0.7 1.5
}
,
    0.595 - 0.7 1.5
}
,
    1.12 - 0.7 1.5
}
,
    1.12 - 0.175 1.5
}
,
    0.595 - 0.175 1.5
}
,
    0.07 - 0.175 1.5
}
,
    0.07 0.35 1.5
}
,
    0.07 0.875 1.5
}
,
    -0.455 0.875 1.5
}
,
    -0.455 0.35 1.5
}
,
    -0.455 - 0.175 1.5
}
,
    -0.455 - 0.7 1.5
}
,
    0.07 - 0.7 1.5
}
,
    0.07 - 0.175 1.5
}
,
    0.07 0.35 1.5
}
,
    -0.455 0.35 1.5
}
,
    -0.455 0.875 1.5 0.07 0.875 1.5 0.595 0.875 1.5 1.12 0.875 1.5 1.645 0.875 1.5 1.645 1.073333333 0.6 1.645 0.35 1.5 1.12 0.35 1.5 0.595 0.35 1.5 0.595 0.875 1.5 1.12 0.875 1.5 1.645 0.875 1.5 1.645 0.35 1.5 1.12 0.35 1.5 1.12 - 0.175 1.5 1.12 - 0.7 1.5 0.595 - 0.7 1.5 0.595 - 0.175 1.5 0.595 0.35 1.5 1.12 0.35 1.5 1.645 0.35 1.5 1.645 0.875 1.5 1.12 0.875 1.5 0.595 0.875 1.5 0.07 0.875 1.5 - 0.455 0.875 1.5 - 0.455 0.35 1.5 - 0.672 0.651 0.6 0.07 0.35 1.5 0.07 - 0.175 1.5 0.07 - 0.7 1.5 - 0.455 - 0.7 1.5 - 0.455 - 0.175 1.5 0.595 0.35 1.5 1.12 0.35 1.5 1.12 - 0.175 1.5 1.12 - 0.7 1.5 0.595 - 0.7 1.5 0.595 - 0.175 1.5 0.07 0.35 1.5 - 0.455 0.35 1.5 - 0.455 0.875 1.5 0.07 0.875 1.5 0.595 0.875 1.5 1.12 0.875 1.5 1.645 0.875 1.5 1.645 0.35 1.5 1.12 0.35 1.5 0.91 0.595 0.6 0.595 0.35 1.5 0.595 - 0.175 1.5 0.07 - 0.175 1.5 0.07 - 0.7 1.5 - 0.455 - 0.7 1.5 - 0.455 - 0.175 1.5 - 0.455 0.35 1.5 0.07 0.35 1.5 0.07 0.875 1.5 - 0.455 0.875 1.5 0.595 0.875 1.5 0.595 0.35 1.5 1.12 0.35 1.5
}
;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

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

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while (ros::ok() && !current_state.armed)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV  armed,");

    while (ros::ok())
    {

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
