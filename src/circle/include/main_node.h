
#ifndef  _MAIN_NODE_H_
#define _MAIN_NODE_H_

#include <ros/ros.h> 
#include <tf/tf.h>

#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <vector> 
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "quadrotor_msgs/PositionCommand.h"

struct sUavMsg
{
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
    nav_msgs::Odometry current_odom;
    cv::Mat cv_depth;
    cv::Mat cv_rgb;
};


class RosNode
{
public:
    RosNode(ros::NodeHandle* nodehandle); 
    void pub_set_pose(mavros_msgs::PositionTarget pose_set);
    void pub_set_att(mavros_msgs::AttitudeTarget att_set);
    void pub_img(cv::Mat img);
    sUavMsg sub_msg();
    bool arm_call(bool arm);
    bool mode_call(std::string mode);

private:
    
    ros::NodeHandle nh_; 
   
    image_transport::ImageTransport it_;
    image_transport::Publisher img_pub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;

    ros::Subscriber mavros_state_sub_; 
    ros::Subscriber current_pose_sub_;
    ros::Subscriber current_odom_sub_;
    ros::Subscriber d435_depth_sub_;
    ros::Subscriber d435_rgb_sub_;

    ros::Publisher set_pos_pub_;
    ros::Publisher set_att_pub_;
    ros::Publisher goal_point_pub_;
    
    ros::Timer agent_timer_;

    mavros_msgs::PositionTarget pose_set_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::State current_state_;

    geometry_msgs::PoseStamped current_pose_;
    nav_msgs::Odometry current_odom_;

    sensor_msgs::Image depth_;
    sensor_msgs::Image rgb_;

    cv::Mat cv_rgb_;
    cv::Mat cv_depth_;
    

    void initializeSubscribers(); 
    void initializePublishers();
    void initializeServices();
    
    //void timeagent_cb(const ros::TimerEvent& e);
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void depth_cb(const sensor_msgs::Image::ConstPtr& msg);
    void rgb_cb(const sensor_msgs::Image::ConstPtr& msg);
    void subscriberCallback(const std_msgs::Float32& message_holder); 
    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);
}; 

#endif  
