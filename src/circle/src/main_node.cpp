#include "main_node.h"

RosNode::RosNode(ros::NodeHandle* nodehandle):nh_(*nodehandle),it_(*nodehandle) { 
    // init all
    initializeSubscribers(); 
    initializePublishers();
    initializeServices();
    // init parameter
}

void RosNode::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
        mavros_state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &RosNode::state_cb, this);
    current_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &RosNode::pose_cb, this);

    current_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 10, &RosNode::odom_cb, this);
    
    d435_depth_sub_ = nh_.subscribe<sensor_msgs::Image>
            ("/camera/depth/image_raw", 2, &RosNode::depth_cb, this);
    d435_rgb_sub_ = nh_.subscribe<sensor_msgs::Image>
            ("/camera/color/image_raw", 2, &RosNode::rgb_cb, this);
    
}

void RosNode::initializeServices() {

    ROS_INFO("Initializing Services");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");

}

void RosNode::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    set_pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10, true);
    set_att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);

    img_pub_ = it_.advertise("visual/camera/image", 2);

}

void RosNode::state_cb(const mavros_msgs::State::ConstPtr& msg) {

    current_state_ = *msg;

}

void RosNode::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	current_pose_ = *msg;

}

void RosNode::odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {

    current_odom_ = *msg;

}

void RosNode::depth_cb(const sensor_msgs::Image::ConstPtr& msg) {

    depth_ = *msg;
    cv_bridge::CvImagePtr depth_ptr;
    try {
        depth_ptr = cv_bridge::toCvCopy(depth_, sensor_msgs::image_encodings::TYPE_16UC1); 
    }
    catch (cv_bridge::Exception& e) {

        ROS_ERROR("Could not convert from '%s' to '16UC1'.", depth_.encoding.c_str());
    }

    cv_depth_ = depth_ptr->image.clone();

}

void RosNode::rgb_cb(const sensor_msgs::Image::ConstPtr& msg) {

    rgb_ = *msg;
    cv_bridge::CvImagePtr rgb_ptr;
    try {
        rgb_ptr = cv_bridge::toCvCopy(rgb_, sensor_msgs::image_encodings::BGR8); 
    }
    catch (cv_bridge::Exception& e) {

        ROS_ERROR("Could not convert from '%s' to 'BGR8'.", rgb_.encoding.c_str());
    }

    cv_rgb_ = rgb_ptr->image.clone();

}

void RosNode::pub_set_pose(mavros_msgs::PositionTarget pose_set) {

    set_pos_pub_.publish(pose_set);

}

void RosNode::pub_set_att(mavros_msgs::AttitudeTarget att_set) {

    set_att_pub_.publish(att_set);

}

void RosNode::pub_img(cv::Mat img) {

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    img_pub_.publish(*msg);
}

sUavMsg RosNode::sub_msg() {

    struct sUavMsg ret;
    ret.current_pose = current_pose_;
    ret.current_odom = current_odom_;
    ret.current_state = current_state_;
    ret.cv_depth = cv_depth_;
    ret.cv_rgb = cv_rgb_;
    return ret;

}

bool RosNode::arm_call(bool arm) {

    mavros_msgs::CommandBool arm_cmd;
    if (arm) arm_cmd.request.value = true;
    else arm_cmd.request.value = false;
    arming_client_.call(arm_cmd);
    return arm_cmd.response.success;

}

bool RosNode::mode_call(std::string mode) {

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mode;

    set_mode_client_.call(offb_set_mode);
    return offb_set_mode.response.mode_sent;
    
}
void RosNode::subscriberCallback(const std_msgs::Float32& message_holder) {
}


//member function implementation for a service callback function
// bool ExampleRosClass::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
//     ROS_INFO("service callback activated");
//     response.resp = true; // boring, but valid response info
//     return true;
// }



int main(int argc, char** argv) 
{
    ros::init(argc, argv, "race"); //node name
    ros::NodeHandle nh; 

    RosNode RosNode(&nh);  
    ros::Rate loop_rate(100);//control hz
    while(ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} 

