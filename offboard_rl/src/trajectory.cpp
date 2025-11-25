#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <offboard_rl/utils.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class Trajectory : public rclcpp::Node
{
public:
    Trajectory() : Node("trajectory")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        local_position_subscription_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos,
            std::bind(&Trajectory::vehicle_local_position_callback, this, std::placeholders::_1));

        attitude_subscription_ = this->create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude",
            qos,
            std::bind(&Trajectory::vehicle_attitude_callback, this, std::placeholders::_1));

        offboard_control_mode_publisher_ =
            this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        trajectory_setpoint_publisher_ =
            this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        vehicle_command_publisher_ =
            this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        timer_offboard_ = this->create_wall_timer(
            100ms, std::bind(&Trajectory::activate_offboard, this));

        timer_trajectory_publish_ = this->create_wall_timer(
            20ms, std::bind(&Trajectory::publish_trajectory_setpoint, this));

        waypoints_ = {
            {  0.0,   0.0,  -6.0,   0.79 },
            {  5.0,   5.0,  -6.0,   0.10 },
            { 10.0,   5.5,  -6.0,  -0.61 },
            { 15.0,   2.0,  -6.0,  -2.36 },
            { 10.0,  -3.0,  -6.0,  -3.04 },
            {  5.0,  -3.5,  -6.0,   2.94 },
            {  0.0,   1.0,  -6.0,  -2.41 },
            { -5.0,  -3.5,  -8.0,  -2.60 },
            { -10.0, -3.0,  -8.0,   3.00 },
            { -15.0,  2.0,  -8.0,   2.63 },
            { -10.0,  5.5,  -8.0,  0.540 },
            { -5.0,   5.0,  -8.0,  -0.10 },
            {  0.0,   0.0,  -8.0,  -0.79 }
        };

        set_point_received = true; 

        T = 5.0;
        trajectory_computed = false;

        RCLCPP_INFO(this->get_logger(), "Trajectory node initialized.");
    }

private:
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_subscription_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;

    std::vector<Eigen::Vector4d> waypoints_;
    size_t current_wp_index{0};
    bool segment_active{false};

    bool set_point_received{false};
    bool offboard_active{false};
    bool trajectory_computed{false};

    Eigen::Vector<double, 6> x; 

    double T{5.0}, t{0.0};  
    Eigen::Vector4d pos_i, pos_f;

    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};
    double offboard_counter{0.0};

    const double v_pass_ = 1.0;

    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = *msg;
    }

    void vehicle_attitude_callback(const VehicleAttitude::SharedPtr msg)
    {
        current_attitude_ = *msg;
    }

    void activate_offboard()
    {
        if (!set_point_received) {
            return;
        }

        if (offboard_counter == 10) {
            RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD and arming");

            VehicleCommand mode_cmd{};
            mode_cmd.param1 = 1;
            mode_cmd.param2 = 6;
            mode_cmd.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            mode_cmd.target_system = 1;
            mode_cmd.target_component = 1;
            mode_cmd.source_system = 1;
            mode_cmd.source_component = 1;
            mode_cmd.from_external = true;
            mode_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(mode_cmd);

            VehicleCommand arm_cmd{};
            arm_cmd.param1 = 1.0;  
            arm_cmd.param2 = 0.0;
            arm_cmd.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            arm_cmd.target_system = 1;
            arm_cmd.target_component = 1;
            arm_cmd.source_system = 1;
            arm_cmd.source_component = 1;
            arm_cmd.from_external = true;
            arm_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_publisher_->publish(arm_cmd);

            pos_i(0) = current_position_.x;
            pos_i(1) = current_position_.y;
            pos_i(2) = current_position_.z;

            auto rpy = utilities::quatToRpy(
                Eigen::Vector4d(current_attitude_.q[0],
                                current_attitude_.q[1],
                                current_attitude_.q[2],
                                current_attitude_.q[3]));
            pos_i(3) = rpy[2]; 

            pos_f = waypoints_[current_wp_index];

            updateSegmentTime();

            offboard_active = true;
            segment_active = true;

            RCLCPP_INFO(this->get_logger(), "Offboard active, starting trajectory.");
        }

        OffboardControlMode off_msg{};
        off_msg.position = true;
        off_msg.velocity = false;
        off_msg.acceleration = false;
        off_msg.attitude = false;
        off_msg.body_rate = false;
        off_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(off_msg);

        if (offboard_counter < 11) {
            offboard_counter++;
        }
    }

    void publish_trajectory_setpoint()
    {
        if (!offboard_active || !segment_active) {
            return;
        }

        double dt = 1.0 / 50.0; 

        TrajectorySetpoint sp = compute_trajectory_setpoint(t);
        trajectory_setpoint_publisher_->publish(sp);

        t += dt;

        if (t > T) {
            current_wp_index++;

            if (current_wp_index >= waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "Trajectory completed, holding final pose.");
                segment_active = false;
                return;
            }

            pos_i = pos_f;
            pos_f = waypoints_[current_wp_index];

            updateSegmentTime();

            t = 0.0;
            trajectory_computed = false;

            RCLCPP_INFO(this->get_logger(), "Starting segment %zu / %zu",
                        current_wp_index, waypoints_.size() - 1);
        }
    }

    void updateSegmentTime()
    {
        Eigen::Vector3d pi(pos_i(0), pos_i(1), pos_i(2));
        Eigen::Vector3d pf(pos_f(0), pos_f(1), pos_f(2));

        double dist = (pf - pi).norm();
        double yaw_diff = std::abs(utilities::angleError(pos_f(3), pos_i(3))); 

        double T_dist = dist / v_pass_;

        double max_yaw_rate = 1.0;
        double T_yaw = yaw_diff / max_yaw_rate;

        T = std::max(T_dist, T_yaw);

        if (T < 5.0) {
            T = 5.0;
        }
    }


    TrajectorySetpoint compute_trajectory_setpoint(double t)
    {
        Eigen::Vector4d e = pos_f - pos_i;
        e(3) = utilities::angleError(pos_f(3), pos_i(3)); 
        double s_f = e.norm(); 

        double v_in_s = 0.0;
        double v_out_s = 0.0;

        if (current_wp_index == 0) {
            v_in_s = 0.0;
        } else {
            v_in_s = v_pass_;
        }

        if (current_wp_index == waypoints_.size() - 1) {
            v_out_s = 0.0;
        } else {
            v_out_s = v_pass_;
        }

        if (!trajectory_computed) {
            Eigen::VectorXd b(6);
            Eigen::Matrix<double, 6, 6> A;

            b << 0.0,
                 v_in_s,
                 0.0,
                 s_f,
                 v_out_s,
                 0.0;

            A << 0,       0,       0,      0,  0, 1,
                 0,       0,       0,      0,  1, 0,
                 0,       0,       0,      2,  0, 0,
                 std::pow(T,5), std::pow(T,4), std::pow(T,3), std::pow(T,2), T, 1,
                 5*std::pow(T,4), 4*std::pow(T,3), 3*std::pow(T,2), 2*T, 1, 0,
                 20*std::pow(T,3), 12*std::pow(T,2), 6*T, 2, 0, 0;

            x = A.inverse() * b;
            trajectory_computed = true;
        }

		double s, s_d, s_dd;

		Eigen::Vector4d ref_traj_pos, ref_traj_vel, ref_traj_acc;

		s   = x(0) * std::pow(t, 5.0)
			+ x(1) * std::pow(t, 4.0)
			+ x(2) * std::pow(t, 3.0)
			+ x(3) * std::pow(t, 2.0)
			+ x(4) * t
			+ x(5);

		s_d = 5.0  * x(0) * std::pow(t, 4.0)
			+ 4.0  * x(1) * std::pow(t, 3.0)
			+ 3.0  * x(2) * std::pow(t, 2.0)
			+ 2.0  * x(3) * t
			+        x(4);

		s_dd = 20.0 * x(0) * std::pow(t, 3.0)
			+ 12.0 * x(1) * std::pow(t, 2.0)
			+  6.0 * x(2) * t
			+        x(3);

		ref_traj_pos = pos_i + s*e/s_f;
    	ref_traj_vel = s_d*e/s_f;
        ref_traj_acc = s_dd*e/s_f;

		TrajectorySetpoint msg{};
		msg.position = {float(ref_traj_pos(0)), float(ref_traj_pos(1)), float(ref_traj_pos(2))};
		msg.velocity = {float(ref_traj_vel(0)), float(ref_traj_vel(1)), float(ref_traj_vel(2))};
		msg.acceleration = {float(ref_traj_acc(0)), float(ref_traj_acc(1)), float(ref_traj_acc(2))};
		msg.yaw = float(ref_traj_pos(3)); // [-PI:PI]
        msg.yawspeed = float(ref_traj_vel(3));
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        return msg;
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting multi-segment trajectory OFFBOARD node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Trajectory>());
    rclcpp::shutdown();
    return 0;
}
