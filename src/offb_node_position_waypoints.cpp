/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>

geometry_msgs::Point        real;
geometry_msgs::PoseArray    waypoints;
geometry_msgs::PoseStamped  goalPose;
mavros_msgs::State          currentState;

int count       = 0 ;
float tolerance = 0.1;
float errorX    = 0;
float errorY    = 0;
float errorZ    = 0;

void localPoseCallback(const geometry_msgs :: PoseStamped :: ConstPtr& msg)
{
    real.x=msg ->pose.position.x;
    real.y=msg ->pose.position.y;
    real.z=msg ->pose.position.z;

    errorX =  goalPose.pose.position.x - real.x;
    errorY =  goalPose.pose.position.y - real.y;
    errorZ =  goalPose.pose.position.z - real.z;

    if ((fabs(errorX) < tolerance) && (fabs(errorY) < tolerance) && (fabs(errorZ) < tolerance))
    {
        count++;
        if(count<waypoints.poses.size())
        {
            goalPose.pose.position.x = waypoints.poses[count].position.x;
            goalPose.pose.position.y = waypoints.poses[count].position.y;
            goalPose.pose.position.z = waypoints.poses[count].position.z;
        }

    }

}

void stateCb(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node_position");
    ros::NodeHandle nh;

    ros::Subscriber stateSub = nh.subscribe<mavros_msgs::State> ("/uav_1/mavros/state", 10, stateCb);
    ros::Subscriber localPoseSub = nh.subscribe("/uav_1/mavros/local_position/pose", 1000, localPoseCallback);
    ros::Publisher localPosPub = nh.advertise<geometry_msgs::PoseStamped> ("/uav_1/mavros/setpoint_position/local", 10);
    ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool> ("/uav_1/mavros/cmd/arming");
    ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode> ("/uav_1/mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && currentState.connected){
        ros::spinOnce();
        rate.sleep();
    }


    //read setpoints from file
    std::string str1 = ros::package::getPath("kuri_mbzirc_sim")+"/config/con_2_100_GPU_arena_ch3.txt";
    const char * filename1 = str1.c_str();

    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open File";
        fclose(file1);
    }

    double locationx,locationy,locationz,qy;
    geometry_msgs::Pose pose;


    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qy);
        pose.position.x = locationx;
        pose.position.y = locationy;
        pose.position.z = locationz;
        pose.orientation.x = qy;
        waypoints.poses.push_back(pose);

    }

    goalPose.pose.position.x = waypoints.poses[count].position.x;
    goalPose.pose.position.y = waypoints.poses[count].position.y;
    goalPose.pose.position.z = waypoints.poses[count].position.z;


    mavros_msgs::SetMode offbSetMode;
    offbSetMode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool armCmd;
    armCmd.request.value = true;

    ros::Time lastRequest = ros::Time::now();
    ros::Time statusUpdate = ros::Time::now();


    while(ros::ok()){
        if( currentState.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
            if( setModeClient.call(offbSetMode) && offbSetMode.response.success){
                ROS_INFO("Offboard enabled");
            }
            lastRequest = ros::Time::now();
        } else {
            if( !currentState.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
                if( armingClient.call(armCmd) && armCmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                lastRequest = ros::Time::now();
            }
            if(ros::Time::now() - statusUpdate > ros::Duration(1.0))
            {
                // std::cout<<"Current Mode is: "<<currentState.mode<<"\n"; fflush(stdout);
                statusUpdate = ros::Time::now();
            }
        }


        localPosPub.publish(goalPose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
