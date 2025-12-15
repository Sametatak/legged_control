#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <cmath>
#include <vector>
#include <string>

// Bu Node, koordinatları alır ve motor açılarını hesaplar.
class SimpleIKNode : public rclcpp::Node
{
public:
    SimpleIKNode() : Node("simple_ik_node")
    {
        // Hedef koordinatları dinle
        target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "leg_targets_xyz", 10, std::bind(&SimpleIKNode::targetCallback, this, std::placeholders::_1));

        // Motorlara komut gönder
        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_group_position_controller/command", 10);

        // --- UNITREE GO2 FİZİKSEL ÖLÇÜLERİ ---
        l1_ = 0.0955; // Hip (yana açılma offseti)
        l2_ = 0.213;  // Upper Leg (Üst bacak)
        l3_ = 0.213;  // Lower Leg (Alt bacak)
        
        // --- BAŞLANGIÇ YÜKSEKLİĞİ ---
        initial_z_ = -0.28; 
        
        // Nominal Y (IK hesaplamasinda referans icin)
        nominal_y_ = 0.10; 

        // --- JOINT İSİMLERİ ---
        joint_names_ = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        };

        // Robot basladiginda ayaga kalkmasi icin timer
        startup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SimpleIKNode::publishStandPose, this));
            
        RCLCPP_INFO(this->get_logger(), "Holonomik IK Node baslatildi. L1: %.4f, L2: %.4f, L3: %.4f", l1_, l2_, l3_);
    }

private:
    void computeIK(double x, double y, double z, double q[3], int leg_index)
    {
        // x, y, z: Ayagin kalca merkezine (hip joint ekseni) gore konumu.
        // Sag bacaklar icin Y degeri negatiftir.
        
        // 1. HIP JOINT (q[0]) - Yana acilma
        // YZ duzlemindeki hipotenus mesafesi
        double lyz = std::sqrt(y*y + z*z);
        // l1 (kalca linki) cikinca kalan bacak duzlemi mesafesi
        // Geometrik kisit: lyz, l1'den kucuk olamaz (fiziksel imkansiz)
        if (lyz < l1_) lyz = l1_ + 0.001;
        
        double l_eff = std::sqrt(lyz*lyz - l1_*l1_);
        
        // Kalca acisi hesaplama
        // Sol bacaklar (0, 2): y pozitif. Sag bacaklar (1, 3): y negatif.
        // Formül: q0 = atan2(y, -z) - atan2(l1, l_eff) [Sol icin]
        // Sag icin isaretler degisir.
        
        double alpha_y = std::atan2(y, -z); // Hedef noktanin yere gore acisi
        double offset_angle = std::atan2(l1_, l_eff); // l1 kaynakli geometrik ofset

        if (leg_index == 0 || leg_index == 2) { // Sol Bacaklar
             q[0] = alpha_y - offset_angle;
        } else { // Sag Bacaklar
             // Sagda Y negatiftir, l1 de simetrik dusunulurse + aci bacak icine - aci disina
             q[0] = alpha_y + offset_angle;
        }

        // 2. UPPER & LOWER LEG (q[1], q[2]) - Ileri geri ve diz
        // Bu hesaplama artik "efektif bacak boyu" (l_eff) uzerinden yapilmali.
        // X-Z duzlemi yerine artik X - Leff duzlemindeyiz (bacak duzlemi dondugu icin)
        
        double r = std::sqrt(x*x + l_eff*l_eff);

        // Erişim sınırı kontrolü
        double max_reach = l2_ + l3_ - 0.01;
        if (r > max_reach) r = max_reach;
        if (r < 0.1) r = 0.1;

        // Cosine Rule ile Diz Açısı (Lower Leg)
        double cos_knee = (l2_*l2_ + l3_*l3_ - r*r) / (2 * l2_ * l3_);
        if(cos_knee > 1.0) cos_knee = 1.0;
        if(cos_knee < -1.0) cos_knee = -1.0;
        
        double alpha_knee = std::acos(cos_knee);

        // Diz Yonu Duzeltmesi (Unitree Go2 icin)
        q[2] = -1.0 * (M_PI - alpha_knee); 

        // Upper Leg (Üst Bacak) Açısı
        double cos_thigh = (l2_*l2_ + r*r - l3_*l3_) / (2 * l2_ * r);
        if(cos_thigh > 1.0) cos_thigh = 1.0;
        if(cos_thigh < -1.0) cos_thigh = -1.0;
        
        double alpha_thigh = std::acos(cos_thigh);
        
        // Hedef noktanin (x, -l_eff) duzlemindeki acisi
        // Not: z yerine -l_eff kullaniyoruz cunku bacak duzleminde dikey eksen artik l_eff (asagi dogru)
        double angle_to_ground = std::atan2(x, l_eff); 

        q[1] = (angle_to_ground + alpha_thigh);
    }

    void publishStandPose()
    {
        if((this->now() - last_msg_time_).seconds() < 0.5) return;

        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = this->now();
        traj_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        for (int i = 0; i < 4; i++) {
            double q[3];
            // Hedef: x=0, z=initial_z. Y ise nominal genislikte durmali.
            double stand_y = (i == 1 || i == 3) ? -nominal_y_ : nominal_y_;
            
            computeIK(0.0, stand_y, initial_z_, q, i);

            point.positions[i*3 + 0] = q[0];
            point.positions[i*3 + 1] = q[1];
            point.positions[i*3 + 2] = q[2];
        }
        
        point.time_from_start = rclcpp::Duration::from_seconds(0.5); 
        traj_msg.points.push_back(point);
        joint_pub_->publish(traj_msg);
    }

    void targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        last_msg_time_ = this->now();
        if (msg->data.size() != 12) return;

        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = this->now();
        traj_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        for (int i = 0; i < 4; i++) {
            double x = msg->data[i*3 + 0];
            double y = msg->data[i*3 + 1];
            double z = msg->data[i*3 + 2]; 

            double q[3];
            computeIK(x, y, z, q, i);

            point.positions[i*3 + 0] = q[0]; 
            point.positions[i*3 + 1] = q[1]; 
            point.positions[i*3 + 2] = q[2]; 
        }

        point.time_from_start = rclcpp::Duration::from_seconds(0.02); 
        traj_msg.points.push_back(point);
        joint_pub_->publish(traj_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::Time last_msg_time_;
    
    double l1_, l2_, l3_, initial_z_, nominal_y_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleIKNode>());
    rclcpp::shutdown();
    return 0;
}
