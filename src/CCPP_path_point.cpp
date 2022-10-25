#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <signal.h>

#include <math.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

#include <geometry_msgs/Twist.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class path_planned
{
public:
    struct goal
    {
        float x, y;
        bool visited;
    };
    vector<goal> Path;
    void add_goal(float X, float Y, bool visit);
};

void path_planned::add_goal(float X, float Y, bool visit) {
    path_planned::goal newGoal;
    newGoal.x = X;
    newGoal.y = Y;
    newGoal.visited = visit;
    Path.push_back(newGoal);
}

class CCPP_path_point
{
public:
    CCPP_path_point(ros::NodeHandle &nh);
    ~CCPP_path_point(){}
    bool new_path = false;
    float x_current, y_current;
    float x_last_current, y_last_current;
    path_planned planned_path;

private:
    ros::Publisher pubPassedPath;
    ros::Publisher pubGoalMsgs;
    ros::Subscriber subOdom;
    ros::Subscriber subCleaningPath;
    void pose_callback(const nav_msgs::Odometry &poses);
    void path_callback(const nav_msgs::Path &path);
    nav_msgs::Path passed_path;
    geometry_msgs::PoseStamped p;
    int taille_last_path = 0;

    float normeNextGoal, distance, distanceBetween;
    int count = 0;
    bool goal_reached = true;
    geometry_msgs::PoseStamped goal_msgs;
    double angle;

    Eigen::AngleAxisd rollAngle, pitchAngle, yawAngle;
    Eigen::Quaterniond quaternion;
};

CCPP_path_point::CCPP_path_point(ros::NodeHandle &nh)
{
    pubPassedPath = nh.advertise<nav_msgs::Path>("/clean_robot/passed_path", 1000);
    pubGoalMsgs = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    subOdom = nh.subscribe("odom", 1000, &CCPP_path_point::pose_callback, this);
    subCleaningPath = nh.subscribe("/path_planning_node/cleaning_plan_nodehandle/cleaning_path", 1000, &CCPP_path_point::path_callback, this);

    ros::Rate loop_rate(10);


    if(!nh.getParam("/NextGoal/tolerance_goal", normeNextGoal))
    {
        ROS_INFO("Please set your tolerance_goal.");
    }
    ROS_INFO("tolerance_goal=%f", normeNextGoal);

    while(ros::ok())
    {
        ros::spinOnce();
        if(new_path)
        {
            count = 0;
            new_path = false;
        }

        //当前处理的点
        cout << "count: " << count << "\n";
        if(!planned_path.Path.empty())
        {
            distance = sqrt(pow(x_current - planned_path.Path[count].x, 2) + pow(y_current - planned_path.Path[count].y, 2));
            distanceBetween = sqrt(pow(x_current - x_last_current, 2) + pow(y_current - y_last_current, 2));
            if( distance <= normeNextGoal)
            {
                count++;
                goal_reached = true;
            }
            else if(distanceBetween <= 1e-5)
            {
                count++;
                goal_reached = true;  
            }
            if(goal_reached)
            {
                goal_msgs.header.frame_id = "map";
                goal_msgs.header.stamp = ros::Time::now();
                goal_msgs.pose.position.x = planned_path.Path[count].x;
                goal_msgs.pose.position.y = planned_path.Path[count].y;
                goal_msgs.pose.position.z = 0;
                if(count < planned_path.Path.size())
                    angle = atan2(planned_path.Path[count+1].y - planned_path.Path[count].y, planned_path.Path[count+1].x - planned_path.Path[count].x);
                else
                    angle = atan2(planned_path.Path[0].y - planned_path.Path[count].y, planned_path.Path[0].x - planned_path.Path[count].x);
                cout << angle << "\n";
                Eigen::Vector3d eulerAngle(0, 0, angle);
                rollAngle = Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX());
                pitchAngle = Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY());
                yawAngle = Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ());
                quaternion = yawAngle * pitchAngle * rollAngle;

                goal_msgs.pose.orientation.w = quaternion.w();
                goal_msgs.pose.orientation.x = quaternion.x();
                goal_msgs.pose.orientation.y = quaternion.y();
                if(planned_path.Path[count].x < planned_path.Path[count+1].x)
                    goal_msgs.pose.orientation.z = 0;
                else
                    goal_msgs.pose.orientation.z = 2;

                cout << "New Goal: " << " " << "x = " << planned_path.Path[count].x << "y = " << planned_path.Path[count].y << "\n";

                pubGoalMsgs.publish(goal_msgs);
                goal_reached = false;
            }
            x_last_current = x_current;
            y_last_current = y_current;
            cout << "Current pose: " << x_current << " " << y_current << "\n";
            cout << "Planned_path: " << planned_path.Path[count].x << " " << planned_path.Path[count].y << "\n";
            cout << "Distance: " << distance << "\n";
        }
        loop_rate.sleep();
    }
}

void CCPP_path_point::pose_callback(const nav_msgs::Odometry &poses) {
    passed_path.header = poses.header;
    x_current = poses.pose.pose.position.x;
    y_current = poses.pose.pose.position.y;

    p.header = poses.header;
    p.pose = poses.pose.pose;
    passed_path.poses.emplace_back(p);
    pubPassedPath.publish(passed_path);
}

void  CCPP_path_point::path_callback(const nav_msgs::Path &path)
{
    //注意为了rviz显示方便 路径一直在发,但是这里只用接受一次就好,当规划的路径发生变化时候再重新装载
    if(planned_path.Path.size() == 0 || path.poses.size() != taille_last_path)
    {
        planned_path.Path.clear();
        new_path = true;
        for(int i=0; i <path.poses.size(); ++i)
        {
            planned_path.add_goal(path.poses[i].pose.position.x, path.poses[i].pose.position.y, false);
            cout << path.poses[i].pose.position.x << " " << path.poses[i].pose.position.y << "\n";
        }
        cout << "Recv path size: " << path.poses.size() << "\n";
        taille_last_path = path.poses.size();
    }
}

//class quaternion_ros
//{
//public:
//    float w, x, y, z;
//    quaternion_ros(){
//        w = 1;
//        x = 0;
//        y = 0;
//        z = 0;
//    }
//    void toQuaternion(float pitch, float roll, float yaw);
//};
//
//void quaternion_ros::toQuaternion(float pitch, float roll, float yaw) {
//
//}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "CCPP_path_point");
    ros::NodeHandle nh;
    CCPP_path_point path(nh);
    
//    MoveBaseClient ac("move_base", true);
//    ROS_INFO("waiting for move_base started");
//    ac.waitForServer();
//
//    ROS_INFO("move_base server has started.");
//
//    move_base_msgs::MoveBaseGoal;
    return 0;
}