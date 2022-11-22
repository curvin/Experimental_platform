#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <unistd.h>
#include <termios.h>
#include <iostream>

char getch()
{
    char buf = 0;
    termios old = {0};

    fflush(stdout);

    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");

    if (read(0, &buf, 1) < 0)
        perror("read()");

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");

    return buf;
}
std_msgs::Bool key;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control_node");
    ros::NodeHandle nh;

    ros::Publisher keyboard_pub = nh.advertise<std_msgs::Bool>("uav/keyboard", 10);
    ros::Rate rate(50.0);
    while (ros::ok())
    {
        int c;
        c = getch();
        key.data = false;
        if (c == ' ')
        {
            key.data = true;

            ROS_INFO("Received command");
        }
        keyboard_pub.publish(key);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}