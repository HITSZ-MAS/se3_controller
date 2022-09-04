#include <ros/ros.h>
#include <se3_controller/se3_controller.hpp>
#include <airsim_ros_pkgs/AngleRateThrottle.h>
#include <airsim_ros_pkgs/PoseCmd.h>

class SE3_EXAMPLE{
private:
    ros::NodeHandle node_;
    ros::Publisher ar_cmd_pub_, pose_cmd_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_;
    ros::Timer exec_timer_;
    // mavros_msgs::State state_;
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

    void execFSMCallback(const ros::TimerEvent &e){
        exec_timer_.stop(); // To avoid blockage

        Controller_Output_t output;
        Desired_State_t desired_state;

        Eigen::Vector3d desired_pos_ned(1,1,-2);
        double desired_yaw_ned = -M_PI / 4;

        desired_state.p(0) = desired_pos_ned(1);
        desired_state.p(1) = desired_pos_ned(0);
        desired_state.p(2) = -desired_pos_ned(2);
        desired_state.yaw = -desired_yaw_ned;

        se3_controller_.calControl(odom_data_, imu_data_, desired_state, output);
        airsim_ros_pkgs::PoseCmd pose_cmd;
        Eigen::Vector3d euler = get_yaw_from_quat(output.q);
        pose_cmd.roll = euler(0);
        pose_cmd.pitch = euler(1);
        pose_cmd.yaw = euler(2);
        pose_cmd.throttle = output.thrust;

        airsim_ros_pkgs::AngleRateThrottle cmd;
        cmd.rollRate = output.bodyrates(0);
        cmd.pitchRate = output.bodyrates(1);
        cmd.yawRate = output.bodyrates(2);
        cmd.throttle = output.thrust;
        
        std::cout << std::endl << "euler:      " << euler.transpose() << std::endl;
        std::cout << "bodyrates:  " << output.bodyrates.transpose() << std::endl;
        std::cout << "thrust:     " << output.thrust << std::endl;

        pose_cmd_pub_.publish(pose_cmd);
        // ar_cmd_pub_.publish(cmd);

        se3_controller_.estimateTa(imu_data_.a);
        exec_timer_.start();
    }
public:
    SE3_EXAMPLE(/* args */){};
    ~SE3_EXAMPLE(){};
    void init(ros::NodeHandle &nh){
        ar_cmd_pub_ = nh.advertise<airsim_ros_pkgs::AngleRateThrottle>("/airsim_node/drone_1/angle_rate_throttle_frame", 10);
        pose_cmd_pub_ = nh.advertise<airsim_ros_pkgs::PoseCmd>("/airsim_node/drone_1/pose_cmd_body_frame", 10);

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned", 10, &SE3_EXAMPLE::OdomCallback, this);
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/airsim_node/drone_1/imu/imu", 10, &SE3_EXAMPLE::IMUCallback, this);

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &SE3_EXAMPLE::execFSMCallback, this);

        kp_p_ << 1.5, 1.5, 1.5;
        kp_v_ << 1.5, 1.5, 1.5;
        kp_a_ << 1.5, 1.5, 1.5;
        kp_q_ << 1.5, 1.5, 1.5;
        kp_w_ << 0.0, 0.0, 0.0;

        kd_p_ << 0.0, 0.0, 0.0;
        kd_v_ << 0.0, 0.0, 0.0;
        kd_a_ << 0.0, 0.0, 0.0;
        kd_q_ << 0.0, 0.0, 0.0;
        kd_w_ << 0.0, 0.0, 0.0;

        hover_percent_ = 0.593593;

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