/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state1;
mavros_msgs::State current_state2;
mavros_msgs::State current_state3;

void state_cb1(const mavros_msgs::State::ConstPtr& msg){
    current_state1 = *msg;
}
void state_cb2(const mavros_msgs::State::ConstPtr& msg){
    current_state2 = *msg;
}
void state_cb3(const mavros_msgs::State::ConstPtr& msg){
    current_state3 = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node_ch3");
    ros::NodeHandle nh;
    
    //uav1
    ros::Subscriber state_sub1 = nh.subscribe<mavros_msgs::State>
            ("/uav_1/mavros/state", 10, state_cb1);
    ros::Publisher local_pos_pub1 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav_1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client1 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_1/mavros/set_mode");
	   
    //uav2	    
    ros::Subscriber state_sub2 = nh.subscribe<mavros_msgs::State>
            ("/uav_2/mavros/state", 10, state_cb2);
    ros::Publisher local_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav_2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_2/mavros/set_mode");
	    
    //uav3	    
	        ros::Subscriber state_sub3 = nh.subscribe<mavros_msgs::State>
            ("/uav_3/mavros/state", 10, state_cb3);
    ros::Publisher local_pos_pub3 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav_3/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client3 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav_3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client3 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav_3/mavros/set_mode");
	    
	    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state1.connected && current_state2.connected && current_state3.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 10;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub1.publish(pose1);
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 5;
    pose2.pose.position.z = 5;
    //send a few setpoints before starting
    for(int j = 100; ros::ok() && j > 0; --j){
        local_pos_pub2.publish(pose2);
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::PoseStamped pose3;
    pose3.pose.position.x = 5;
    pose3.pose.position.y = 0;
    pose3.pose.position.z = 2;
        //send a few setpoints before starting
    for(int k = 100; ros::ok() && k > 0; --k){
        local_pos_pub3.publish(pose3);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(  (current_state1.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))  &&
	  (current_state2.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) &&
	  (current_state3.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
	){
            if( (set_mode_client1.call(offb_set_mode) && offb_set_mode.response.success) && 
	      (set_mode_client2.call(offb_set_mode) && offb_set_mode.response.success)   &&
	      (set_mode_client3.call(offb_set_mode) && offb_set_mode.response.success)
	    ){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( (!current_state1.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) && 
	      (!current_state2.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) && 
	      (!current_state3.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) ){
                if( (arming_client1.call(arm_cmd) && arm_cmd.response.success) &&
		  (arming_client2.call(arm_cmd) && arm_cmd.response.success) &&
		  (arming_client3.call(arm_cmd) && arm_cmd.response.success)
		){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub1.publish(pose1);
	local_pos_pub2.publish(pose2);
        local_pos_pub3.publish(pose3);

	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
