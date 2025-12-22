#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "champ_msgs/msg/contacts_stamped.hpp" 
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <vector>
#include <deque>

class AdvancedGaitNode : public rclcpp::Node
{
public:
    AdvancedGaitNode() : Node("simple_gait_node")
    {
        // Parameters
        this->declare_parameter("step_height", 0.15);
        this->declare_parameter("stair_step_height", 0.18);
        this->declare_parameter("step_frequency", 3.0);
        this->declare_parameter("nominal_height", -0.28);
        this->declare_parameter("imu_stabilization", false);
        this->declare_parameter("step_contacts", true);     
        this->declare_parameter("pitch_gain", 0.5);         
        this->declare_parameter("imu_deadband", 0.02);      

        load_parameters();

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&AdvancedGaitNode::on_params_changed, this, std::placeholders::_1));

        // Subscriptions and Publishers
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&AdvancedGaitNode::vel_callback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&AdvancedGaitNode::imu_callback, this, std::placeholders::_1));

        contact_sub_ = this->create_subscription<champ_msgs::msg::ContactsStamped>(
            "/foot_contacts", 10, std::bind(&AdvancedGaitNode::contact_callback, this, std::placeholders::_1));

        target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("leg_targets_xyz", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("foot_trajectory_markers", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&AdvancedGaitNode::control_loop, this));

        // Initialization
        contact_states_ = {true, true, true, true};
        is_latch_active_ = {false, false, false, false};
        latched_z_heights_ = {nominal_height_, nominal_height_, nominal_height_, nominal_height_};
        pitch_error_ = 0.0;
        leg_histories_.resize(4);
    }

private:
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) { current_vel_ = *msg; }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        double p = std::asin(2.0 * (qw * qy - qz * qx));
        pitch_error_ = (std::abs(p) > imu_deadband_) ? p : 0.0;
    }

    void contact_callback(const champ_msgs::msg::ContactsStamped::SharedPtr msg)
    {
        if(msg->contacts.size() >= 4) {
            for(int i=0; i<4; i++) contact_states_[i] = msg->contacts[i];
        }
    }

    void control_loop()
    {
        static double phase = 0.0;
        
        bool is_stair_mode = (current_vel_.linear.z > 0.45 && 
                              std::abs(current_vel_.linear.x) < 0.05 && 
                              std::abs(current_vel_.linear.y) < 0.05);

        double active_freq = is_stair_mode ? 1.5 : step_frequency_;
        double active_step_h = is_stair_mode ? stair_step_height_ : step_height_;
        double active_fwd_vel = is_stair_mode ? 0.12 : current_vel_.linear.x; 

        bool is_stepping = (std::abs(active_fwd_vel) > 0.01 || 
                            std::abs(current_vel_.linear.y) > 0.01 || 
                            std::abs(current_vel_.angular.z) > 0.01);

        if (is_stepping) {
            phase += 0.02 * (2 * M_PI * active_freq);
        } else {
            phase = 0.0;
        }

        std_msgs::msg::Float64MultiArray targets_msg;
        targets_msg.data.resize(12);
        visualization_msgs::msg::MarkerArray marker_array;

        for (int i = 0; i < 4; i++) {
            double leg_phase = fmod(phase + ((i == 1 || i == 2) ? M_PI : 0.0), 2 * M_PI);
            double sin_p = std::sin(leg_phase);
            
            double swing_height = 0.0;
            if (sin_p > 0) { 
                swing_height = sin_p * active_step_h;
                is_latch_active_[i] = false; 
            }

            double x_target = std::cos(leg_phase) * active_fwd_vel * 0.15; 
            double y_target = std::cos(leg_phase) * current_vel_.linear.y * 0.10; 
            double turn_x = (i == 0 || i == 2) ? -current_vel_.angular.z * 0.05 : current_vel_.angular.z * 0.05;
            x_target += turn_x;

            double stabilization_offset = 0.0;
            if (imu_stabilization_ || is_stair_mode) {
                double pitch_factor = (i < 2) ? 1.0 : -1.0;
                stabilization_offset += pitch_factor * pitch_error_ * pitch_gain_;
            }

            double z_final = nominal_height_ + swing_height + stabilization_offset;

	    if (step_contacts_ && sin_p <= 0) {
		    if (!is_latch_active_[i]) {
			if (contact_states_[i]) {
			    latched_z_heights_[i] = z_final;
			    is_latch_active_[i] = true;
			    // AŞAĞIDAKİ LOGU EKLE:
			    RCLCPP_INFO(this->get_logger(), "Ayak %d temas etti! Yukseklik kilitlendi: %.3f", i, z_final);
			}
		    } else { 
			z_final = latched_z_heights_[i]; 
		    }
		}

            double y_final = ((i == 1 || i == 3) ? -0.10 : 0.10) + y_target;

            targets_msg.data[i * 3 + 0] = x_target;
            targets_msg.data[i * 3 + 1] = y_final;
            targets_msg.data[i * 3 + 2] = z_final;

            // --- Marker Visualization ---
            double hip_x_offset = (i < 2) ? 0.193 : -0.193; 
            geometry_msgs::msg::Point current_p;
            current_p.x = hip_x_offset + x_target;
            current_p.y = y_final;
            current_p.z = z_final;
            
            leg_histories_[i].push_back(current_p);
            if(leg_histories_[i].size() > 50) leg_histories_[i].pop_front();

            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "base_link";
            line_marker.header.stamp = this->now();
            line_marker.ns = "trajectories";
            line_marker.id = i;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.01;
            line_marker.color.a = 1.0; 
            
            if(i==0){ line_marker.color.r = 1.0; }
            else if(i==1){ line_marker.color.g = 1.0; }
            else if(i==2){ line_marker.color.b = 1.0; }
            else { line_marker.color.r = 1.0; line_marker.color.g = 1.0; line_marker.color.b = 1.0; }

            for(const auto& pt : leg_histories_[i]) {
                line_marker.points.push_back(pt);
            }
            marker_array.markers.push_back(line_marker);
        }

        target_pub_->publish(targets_msg);
        marker_pub_->publish(marker_array);
    }

    void load_parameters() {
        nominal_height_ = this->get_parameter("nominal_height").as_double();
        step_height_ = this->get_parameter("step_height").as_double();
        stair_step_height_ = this->get_parameter("stair_step_height").as_double();
        step_frequency_ = this->get_parameter("step_frequency").as_double();
        imu_stabilization_ = this->get_parameter("imu_stabilization").as_bool();
        step_contacts_ = this->get_parameter("step_contacts").as_bool();
        pitch_gain_ = this->get_parameter("pitch_gain").as_double();
        imu_deadband_ = this->get_parameter("imu_deadband").as_double();
    }

    rcl_interfaces::msg::SetParametersResult on_params_changed(const std::vector<rclcpp::Parameter> &params) {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        for (auto &p : params) {
            if (p.get_name() == "step_height") step_height_ = p.as_double();
            if (p.get_name() == "stair_step_height") stair_step_height_ = p.as_double();
            if (p.get_name() == "step_frequency") step_frequency_ = p.as_double();
            if (p.get_name() == "imu_stabilization") imu_stabilization_ = p.as_bool();
            if (p.get_name() == "step_contacts") step_contacts_ = p.as_bool();
            if (p.get_name() == "pitch_gain") pitch_gain_ = p.as_double();
            if (p.get_name() == "imu_deadband") imu_deadband_ = p.as_double();
        }
        return result;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<champ_msgs::msg::ContactsStamped>::SharedPtr contact_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    geometry_msgs::msg::Twist current_vel_;
    std::vector<bool> contact_states_, is_latch_active_;
    std::vector<double> latched_z_heights_;
    std::vector<std::deque<geometry_msgs::msg::Point>> leg_histories_;
    double pitch_error_, nominal_height_, step_height_, stair_step_height_, step_frequency_, pitch_gain_, imu_deadband_;
    bool imu_stabilization_, step_contacts_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdvancedGaitNode>());
    rclcpp::shutdown();
    return 0;
}
