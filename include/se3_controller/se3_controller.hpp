#pragma once
#include <queue>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

// #define TRACK_TRAJ
// #define ACHIEVE_POINT

struct Odom_Data_t{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Quaterniond q;
	Eigen::Vector3d w;

	nav_msgs::Odometry msg;
	ros::Time rcv_stamp;
	bool recv_new_msg;

	Odom_Data_t(){
		recv_new_msg = false;
	}
	void feed(nav_msgs::OdometryConstPtr pMsg){
		msg = *pMsg;
		rcv_stamp = ros::Time::now();
		recv_new_msg = true;

		p(0) = msg.pose.pose.position.x;
		p(1) = msg.pose.pose.position.y;
		p(2) = msg.pose.pose.position.z;

		v(0) = msg.twist.twist.linear.x;
		v(1) = msg.twist.twist.linear.y;
		v(2) = msg.twist.twist.linear.z;

		q.w() = msg.pose.pose.orientation.w;
		q.x() = msg.pose.pose.orientation.x;
		q.y() = msg.pose.pose.orientation.y;
		q.z() = msg.pose.pose.orientation.z;

		w(0) = msg.twist.twist.angular.x;
		w(1) = msg.twist.twist.angular.y;
		w(2) = msg.twist.twist.angular.z;
	}
};

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){
		p = Eigen::Vector3d::Zero();
		v = Eigen::Vector3d::Zero();
		a = Eigen::Vector3d::Zero();
		j = Eigen::Vector3d::Zero();
		q.w() = 1;
		q.x() = 0;
		q.y() = 0;
		q.z() = 0;
		yaw = 0;
		yaw_rate = 0;
	}

	Desired_State_t(Odom_Data_t odom){
		p = odom.p;
		v = Eigen::Vector3d::Zero();
		a = Eigen::Vector3d::Zero();
		j = Eigen::Vector3d::Zero();
		q = odom.q.normalized();
		yaw = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
		yaw_rate = 0;
	}
};

struct Controller_Output_t
{
	// Eigen::Vector3d v;

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;
};

struct Imu_Data_t{
	Eigen::Quaterniond q;
	Eigen::Vector3d w;
	Eigen::Vector3d a;

	sensor_msgs::Imu msg;
	ros::Time rcv_stamp;
	bool recv_new_msg;

	Imu_Data_t(){
		recv_new_msg = false;
	}
	void feed(sensor_msgs::ImuConstPtr pMsg){
		msg = *pMsg;
		rcv_stamp = ros::Time::now();
		recv_new_msg = true;

		w(0) = msg.angular_velocity.x;
		w(1) = msg.angular_velocity.y;
		w(2) = msg.angular_velocity.z;

		a(0) = msg.linear_acceleration.x;
		a(1) = msg.linear_acceleration.y;
		a(2) = msg.linear_acceleration.z;

		q.x() = msg.orientation.x;
		q.y() = msg.orientation.y;
		q.z() = msg.orientation.z;
		q.w() = msg.orientation.w;

		// check the frequency
		// static int one_min_count = 9999;
		// static ros::Time last_clear_count_time = ros::Time(0.0);
		// if ( (now - last_clear_count_time).toSec() > 1.0 ){
		// 	if ( one_min_count < 100 )
		// 		ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
		// 	one_min_count = 0;
		// 	last_clear_count_time = now;
		// }
		// one_min_count ++;
	}
};

class SE3_CONTROLLER
{
private:
	Eigen::Vector3d Kp_p_, Kp_v_, Kp_a_, Kp_q_, Kp_w_, Kd_p_, Kd_v_, Kd_a_, Kd_q_, Kd_w_;
	bool have_last_err_;

	double hover_percent_;
	double T_a_; // normalization constant
	double P_ = 1e6;
	const double rho_ = 0.998; // confidence
	const double gravity_ = 9.81;
	Eigen::Vector3d grav_vec_, last_err_p_, last_err_v_, last_err_a_, last_err_q_, last_err_w_;
	std::queue<std::pair<ros::Time, double>> timed_thrust_;

	void differential_flatness(Desired_State_t desired_state, Odom_Data_t &desired_odom,  Eigen::Vector3d acc, Odom_Data_t odom_data, Imu_Data_t imu_data){
		
		desired_odom.p = desired_state.p;
		desired_odom.v = desired_state.v;

		// #ifdef ACHIEVE_POINT
		// double roll,pitch,yaw,yaw_imu;
		// double yaw_odom = fromQuaternion2yaw(odom_data.q);
		// double sin = std::sin(yaw_odom);
		// double cos = std::cos(yaw_odom);
		// roll = (acc(0) * sin - acc(1) * cos ) / gravity_;
		// pitch = (acc(0) * cos + acc(1) * sin ) / gravity_;
		// // yaw = fromQuaternion2yaw(des.q);
		// // yaw_imu = fromQuaternion2yaw(imu.q);
		// // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
		// //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
		// //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
		// Eigen::Quaterniond q = Eigen::AngleAxisd(desired_state.yaw,Eigen::Vector3d::UnitZ())
		// 	* Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
		// 	* Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
		// // q = imu_data.q * odom_data.q.inverse() * q;
		// desired_odom.q = q;
		// desired_odom.w = odom_data.w;
		// #endif
		// TODO check
		// #ifdef TRACK_TRAJ
		Eigen::Vector3d xc(cos(desired_state.yaw), sin(desired_state.yaw), 0);
		Eigen::Vector3d yc(-sin(desired_state.yaw), cos(desired_state.yaw), 0);
		Eigen::Vector3d alpha = desired_state.a + grav_vec_;
		Eigen::Vector3d xb = yc.cross(alpha);
		xb.normalize();
		Eigen::Vector3d yb = alpha.cross(xb);
		yb.normalize();
		Eigen::Vector3d zb = xb.cross(yb);
		Eigen::Matrix3d rotM;
		rotM << xb, yb, zb;
		desired_odom.q = Eigen::Quaterniond(rotM);

		double a_zb = zb.dot(alpha);
		desired_odom.w(0) = -yb.dot(desired_state.j) / a_zb;
		desired_odom.w(1) = xb.dot(desired_state.j) / a_zb;
		desired_odom.w(2) = desired_state.yaw_rate * xc.dot(xb) + yc.dot(zb) * desired_odom.w(1);
		desired_odom.w(2) /= (yc.cross(zb)).norm();
		// #endif
	}

	double fromQuaternion2yaw(Eigen::Quaterniond q){
		return atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
	}

	void limitErr(Eigen::Vector3d &err, double low, double upper){
		err(0) = std::max(std::min(err(0), upper), low);
		err(1) = std::max(std::min(err(1), upper), low);
		err(2) = std::max(std::min(err(2), upper), low);
	}

public:
	SE3_CONTROLLER(){};
    ~SE3_CONTROLLER(){};

	void init(Eigen::Vector3d kp_p, Eigen::Vector3d kp_v, Eigen::Vector3d kp_a, Eigen::Vector3d kp_q, Eigen::Vector3d kp_w, Eigen::Vector3d kd_p, Eigen::Vector3d kd_v, Eigen::Vector3d kd_a, Eigen::Vector3d kd_q, Eigen::Vector3d kd_w, double hover_percent){
		Kp_p_ = kp_p;
		Kp_v_ = kp_v;
		Kp_a_ = kp_a;
		Kp_q_ = kp_q;
		Kp_w_ = kp_w;
		Kd_p_ = kd_p;
		Kd_v_ = kd_v;
		Kd_a_ = kd_a;
		Kd_q_ = kd_q;
		Kd_w_ = kd_w;
		hover_percent_ = hover_percent;
		T_a_ = gravity_ / hover_percent_;
		grav_vec_ << 0.0, 0.0, gravity_;

		last_err_p_ = Eigen::Vector3d::Zero();
		last_err_v_ = Eigen::Vector3d::Zero();
		last_err_a_ = Eigen::Vector3d::Zero();
		last_err_q_ = Eigen::Vector3d::Zero();
		last_err_w_ = Eigen::Vector3d::Zero();

		have_last_err_ = false;
	}

	void run(Odom_Data_t odom_data, Imu_Data_t imu_data, Desired_State_t desired_state, Controller_Output_t &output){
		if((ros::Time::now() - odom_data.rcv_stamp).toSec() > 0.1){
			std::cout << "odom not rcv" << std::endl;
			return;
		}
		Eigen::Vector3d err_p = odom_data.p - desired_state.p;
		limitErr(err_p, -1.0, 1.0);
		Eigen::Vector3d err_v = odom_data.v - desired_state.v;
		limitErr(err_v, -1.0, 1.0);
		if(have_last_err_ == false){
			last_err_p_ = err_p;
			last_err_v_ = err_v;
		}
		Eigen::Vector3d d_err_p = err_p - last_err_p_;
		limitErr(d_err_p, -1.0, 1.0);
		Eigen::Vector3d d_err_v = err_v - last_err_v_;
		limitErr(d_err_v, -1.0, 1.0);
		Eigen::Vector3d acc = desired_state.a - Kp_p_.asDiagonal() * err_p - Kp_v_.asDiagonal() * err_v - Kd_p_.asDiagonal() * d_err_p - Kd_v_.asDiagonal() * d_err_v + grav_vec_;
		desired_state.a = acc - grav_vec_;
		// std::cout << std::endl << "err_p: " << err_p.transpose() << std::endl;
		// std::cout << std::endl << "err_v: " << err_v.transpose() << std::endl;
		Eigen::Vector3d err_a = acc - grav_vec_ - desired_state.a;
		limitErr(err_a, -1.0, 1.0);
		if(have_last_err_ == false){
			last_err_a_ = err_a;
		}
		Eigen::Vector3d d_err_a = err_a - last_err_a_;
		limitErr(d_err_a, -1.0, 1.0);
		desired_state.j = desired_state.j - Kp_a_.asDiagonal() * err_a - Kd_a_.asDiagonal() * d_err_a;

		last_err_p_ = err_p;
		last_err_v_ = err_v;
		last_err_a_ = err_a;
		
		double thr = acc.transpose() * (odom_data.q * Eigen::Vector3d::UnitZ());
		output.thrust = thr / T_a_;
		// std::cout << std::endl << "T_a: " << T_a_ << std::endl;
		// std::cout << "acc: " << acc.transpose() << std::endl;
		
		Odom_Data_t desired_odom;
		differential_flatness(desired_state, desired_odom, acc, odom_data, imu_data);
		
		Eigen::Vector3d err_q = (odom_data.q * (desired_odom.q.inverse())).vec();
		limitErr(err_q, -1.0, 1.0);
		// Eigen::Vector3d err_w = odom_data.w - odom_data.q.matrix().transpose() * desired_odom.q.matrix() * desired_odom.w;
		Eigen::Vector3d err_w = odom_data.w - desired_odom.w;
		limitErr(err_w, -1.0, 1.0);
		if(have_last_err_ == false){
			have_last_err_ = true;
			last_err_q_ = err_q;
			last_err_w_ = err_w;
		}
		Eigen::Vector3d d_err_q = err_q - last_err_q_;
		limitErr(d_err_q, -1.0, 1.0);
		Eigen::Vector3d d_err_w = err_w - last_err_w_;
		limitErr(d_err_w, -1.0, 1.0);
		output.bodyrates = desired_odom.w - Kp_q_.asDiagonal() * err_q - Kp_w_.asDiagonal() * err_w - Kd_q_.asDiagonal() * d_err_q - Kd_w_.asDiagonal() * d_err_w;
		// std::cout << "thrust: " << output.thrust << std::endl;
		// std::cout << "bodyrates: " << output.bodyrates.transpose() << std::endl;
		last_err_q_ = err_q;
		last_err_w_ = err_w;

		timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), output.thrust));
		while (timed_thrust_.size() > 100)
			timed_thrust_.pop();
	}

	bool estimateTa(const Eigen::Vector3d &est_a){
		ros::Time t_now = ros::Time::now();
		while (timed_thrust_.size() >= 1)
		{
			// Choose data before 35~45ms ago
			std::pair<ros::Time, double> t_t = timed_thrust_.front();
			double time_passed = (t_now - t_t.first).toSec();
			if (time_passed > 0.045){ // 45ms
				// printf("continue, time_passed=%f\n", time_passed);
				timed_thrust_.pop();
				continue;
			}
			if (time_passed < 0.035){ // 35ms
				// printf("skip, time_passed=%f\n", time_passed);
				return false;
			}

			/***********************************************************/
			/* Recursive least squares algorithm with vanishing memory */
			/***********************************************************/
			double thr = t_t.second;
			timed_thrust_.pop();
			
			/***********************************/
			/* Model: est_a(2) = thr1acc_ * thr */
			/***********************************/
			double gamma = 1 / (rho_ + thr * P_ * thr);
			double K = gamma * P_ * thr;
			T_a_ = T_a_ + K * (est_a(2) - thr * T_a_);
			P_ = (1 - K * thr) * P_ / rho_;
			// std::cout << std::endl << "imu: " << est_a.transpose() << std::endl;
			//printf("%6.3f,%6.3f,%6.3f,%6.3f\n", T_a_, gamma, K, P_);
			//fflush(stdout);

			return true;
		}
		return false;
	}
};
