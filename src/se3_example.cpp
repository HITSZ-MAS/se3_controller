#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <dynamic_reconfigure/server.h>

#include "se3_controller/se3_controller.hpp"
#include "se3_controller/se3_dynamic_tuneConfig.h"

class SE3_EXAMPLE{
private:
    ros::NodeHandle node_;
    ros::Publisher cmd_pub_, desire_odom_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_, desire_odom_sub_;
    ros::Timer exec_timer_;
    mavros_msgs::State state_;
    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Desired_State_t desired_state_;
    SE3_CONTROLLER se3_controller_;
    nav_msgs::Odometry desire_odom_;
    Eigen::Vector3d kp_p_, kp_v_, kp_a_, kp_q_, kp_w_, kd_p_, kd_v_, kd_a_, kd_q_, kd_w_;
    double hover_percent_;

    dynamic_reconfigure::Server<se3_controller::se3_dynamic_tuneConfig> dynamic_tune_server_;
    dynamic_reconfigure::Server<se3_controller::se3_dynamic_tuneConfig>::CallbackType dynamic_tune_cb_type_;

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        odom_data_.feed(msg);
    }

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg){
        imu_data_.feed(msg);
    }

    void StateCallback(const mavros_msgs::State::ConstPtr &msg){
        state_ = *msg;
    }

    void DesireOdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        desire_odom_ = *msg;

        desired_state_.p(0) = msg->pose.pose.position.x;
        desired_state_.p(1) = msg->pose.pose.position.y;
        desired_state_.p(2) = msg->pose.pose.position.z;

        desired_state_.v(0) = msg->twist.twist.linear.x;
        desired_state_.v(1) = msg->twist.twist.linear.y;
        desired_state_.v(2) = msg->twist.twist.linear.z;

        desired_state_.a.setZero();
        desired_state_.j.setZero();

        desired_state_.q.w() = msg->pose.pose.orientation.w;
        desired_state_.q.x() = msg->pose.pose.orientation.x;
        desired_state_.q.y() = msg->pose.pose.orientation.y;
        desired_state_.q.z() = msg->pose.pose.orientation.z;

        desired_state_.yaw = utils::fromQuaternion2yaw(desired_state_.q);
        desired_state_.yaw_rate = 0.0;
    }

    void DynamicTuneCallback(se3_controller::se3_dynamic_tuneConfig &config, uint32_t level){
        ROS_INFO("kp_p: %f %f %f\n", config.kp_px, config.kp_py, config.kp_pz);
        ROS_INFO("kp_v: %f %f %f\n", config.kp_vx, config.kp_vy, config.kp_vz);
        ROS_INFO("kp_a: %f %f %f\n", config.kp_ax, config.kp_ay, config.kp_az);
        ROS_INFO("kp_q: %f %f %f\n", config.kp_qx, config.kp_qy, config.kp_qz);
        ROS_INFO("kp_w: %f %f %f\n", config.kp_wx, config.kp_wy, config.kp_wz);

        ROS_INFO("kd_p: %f %f %f\n", config.kd_px, config.kd_py, config.kd_pz);
        ROS_INFO("kd_v: %f %f %f\n", config.kd_vx, config.kd_vy, config.kd_vz);
        ROS_INFO("kd_a: %f %f %f\n", config.kd_ax, config.kd_ay, config.kd_az);
        ROS_INFO("kd_q: %f %f %f\n", config.kd_qx, config.kd_qy, config.kd_qz);
        ROS_INFO("kd_w: %f %f %f\n", config.kd_wx, config.kd_wy, config.kd_wz);
    }

    void send_cmd(const Controller_Output_t &output, bool angle){
        mavros_msgs::AttitudeTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.body_rate.x = output.bodyrates(0);
        cmd.body_rate.y = output.bodyrates(1);
        cmd.body_rate.z = output.bodyrates(2);
        cmd.orientation.w = output.q.w();
        cmd.orientation.x = output.q.x();
        cmd.orientation.y = output.q.y();
        cmd.orientation.z = output.q.z();
        cmd.thrust = output.thrust;
        if(angle){
            cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE + mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE + mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        }else{
            cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        }
        cmd_pub_.publish(cmd);
    }

    void execFSMCallback(const ros::TimerEvent &e){
        exec_timer_.stop();

        Controller_Output_t output;
        if(se3_controller_.calControl(odom_data_, imu_data_, desired_state_, output)){
            send_cmd(output, true);
            desire_odom_pub_.publish(desire_odom_);
            if(state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true){
                se3_controller_.estimateTa(imu_data_.a);
            }
        }
        
        exec_timer_.start();
    }
public:
    SE3_EXAMPLE(/* args */){};
    ~SE3_EXAMPLE(){};
    void init(ros::NodeHandle &nh){
        cmd_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
        desire_odom_pub_ = nh.advertise<nav_msgs::Odometry>("/desire_odom_pub", 10);

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/odometry/in", 10, &SE3_EXAMPLE::OdomCallback, this);
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &SE3_EXAMPLE::IMUCallback, this);
        state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &SE3_EXAMPLE::StateCallback, this);
        desire_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/desire_odom", 10, &SE3_EXAMPLE::DesireOdomCallback, this);

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &SE3_EXAMPLE::execFSMCallback, this);

        dynamic_tune_cb_type_ = boost::bind(&SE3_EXAMPLE::DynamicTuneCallback, this, _1, _2);
        dynamic_tune_server_.setCallback(dynamic_tune_cb_type_);

        kp_p_ << 1.5, 1.5, 1.5;
        kp_v_ << 1.5, 1.5, 1.5;
        kp_a_ << 1.5, 1.5, 1.5;
        kp_q_ << 5.5, 5.5, 0.1;
        kp_w_ << 1.5, 1.5, 0.1;

        kd_p_ << 0.0, 0.0, 0.0;
        kd_v_ << 0.0, 0.0, 0.0;
        kd_a_ << 0.0, 0.0, 0.0;
        kd_q_ << 0.0, 0.0, 0.0;
        kd_w_ << 0.0, 0.0, 0.0;

        hover_percent_ = 0.7;

        desired_state_.p(0) = 0;
        desired_state_.p(1) = 0;
        desired_state_.p(2) = 1;
        desired_state_.yaw = 0.0;

        se3_controller_.init(kp_p_, kp_v_, kp_a_, kp_q_, kp_w_, kd_p_, kd_v_, kd_a_, kd_q_, kd_w_, hover_percent_);
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "example_node");
    ros::NodeHandle nh("~");

    SE3_EXAMPLE se3_example;
    se3_example.init(nh);

    ros::spin();

    return 0;
}