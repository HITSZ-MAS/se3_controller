#include <ros/ros.h>
#include <se3_controller/se3_controller.hpp>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>

class SE3_EXAMPLE{
private:
    ros::NodeHandle node_;
    ros::Publisher cmd_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_;
    ros::Timer exec_timer_;
    mavros_msgs::State state_;
    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Desired_State_t desired_state_;
    SE3_CONTROLLER se3_controller_;
    Eigen::Vector3d kp_p_, kp_v_, kp_a_, kp_q_, kp_w_, kd_p_, kd_v_, kd_a_, kd_q_, kd_w_;
    double hover_percent_;

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        odom_data_.feed(msg);
    }

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg){
        imu_data_.feed(msg);
    }

    void StateCallback(const mavros_msgs::State::ConstPtr &msg){
        state_ = *msg;
    }

    void execFSMCallback(const ros::TimerEvent &e){
        exec_timer_.stop(); // To avoid blockage

        Controller_Output_t output;
        Desired_State_t desired_state;
        desired_state.p(2) = 1;
        se3_controller_.run(odom_data_, imu_data_, desired_state, output);
        mavros_msgs::AttitudeTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.body_rate.x = output.bodyrates(0);
        cmd.body_rate.y = output.bodyrates(1);
        cmd.body_rate.z = output.bodyrates(2);
        cmd.thrust = output.thrust;
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        cmd_pub_.publish(cmd);
        if(state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true)
            se3_controller_.estimateTa(imu_data_.a);
        exec_timer_.start();
    }
public:
    SE3_EXAMPLE(/* args */){};
    ~SE3_EXAMPLE(){};
    void init(ros::NodeHandle &nh){
        cmd_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/odometry/in", 10, &SE3_EXAMPLE::OdomCallback, this);
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &SE3_EXAMPLE::IMUCallback, this);
        state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &SE3_EXAMPLE::StateCallback, this);

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &SE3_EXAMPLE::execFSMCallback, this);

        kp_p_ << 2.0, 2.0, 2.0;
        kp_v_ << 1.5, 1.5, 1.5;
        kp_a_ << 3.5, 3.5, 3.5;
        kp_q_ << 3.0, 3.0, 3.0;
        kp_w_ << 0.5, 0.5, 0.5;

        kd_p_ << 0.5, 0.5, 0.5;
        kd_v_ << 0.0, 0.0, 0.0;
        kd_a_ << 0.5, 0.5, 0.5;
        kd_q_ << 0.5, 0.5, 0.5;
        kd_w_ << 0.0, 0.0, 0.0;

        hover_percent_ = 0.7;

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