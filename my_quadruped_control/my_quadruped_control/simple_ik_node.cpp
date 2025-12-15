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

        // Motorlara komut gönder (Gazebo veya gerçek robot kontrolcüsü için standart topic)
        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "joint_group_position_controller/command", 10);

        // Robotun Uzuv Uzunlukları (METRE CİNSİNDEN - BUNLARI KENDİ ROBOTUNA GÖRE DÜZENLE)
        l1_ = 0.05; // Hip (Kalça) offset (yana doğru)
        l2_ = 0.20; // Upper Leg (Üst bacak)
        l3_ = 0.20; // Lower Leg (Alt bacak)

        // Joint isimleri (URDF dosyanla uyumlu olmalı)
        joint_names_ = {
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint", // lh = lb (left hind)
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"  // rh = rb (right hind)
        };
    }

private:
    // Basit Geometrik Ters Kinematik Fonksiyonu
    // x, y, z: Ayağın kalça eklemine (shoulder) göre konumu
    // q: Hesaplanan açılar (hip, upper, lower)
    // is_right_leg: Sağ bacaklarda koordinat sistemi simetrisi için
    void computeIK(double x, double y, double z, double q[3], bool is_right_leg)
    {
        // 1. Hip Roll (Kalça Yana Açılma)
        // Sağ ve sol bacakta y ekseni farklı çalışabilir, basit model:
        double dy = y; 
        double dz = z;
        double l1_eff = is_right_leg ? -l1_ : l1_; 
        
        // Yan düzlemdeki hipotenüs
        double lyz = std::sqrt(dy*dy + dz*dz); 
        // l1_ çıktığında kalan bacak düzlemi uzunluğu
        double l_eff = std::sqrt(lyz*lyz - l1_*l1_);

        q[0] = 0.0; // Basitleştirme: Şu anlık yana açılmayı 0 kabul edelim veya basit atan2
        // Not: Gerçek bir robotta burası atan2(z,y) ve geometrik ofset hesapları gerektirir.
        
        // Şimdilik sadece ileri-geri (Pitch) harekete odaklanalım (Daha stabil test için):
        // X-Z düzleminde 2 Linkli IK (Law of Cosines)
        
        double r = std::sqrt(x*x + z*z); // Kalçadan ayağa olan mesafe
        
        // Cosine Rule
        // c^2 = a^2 + b^2 - 2ab*cos(C)
        // Burada r bizim c'miz (ama L1 yüzünden biraz kısalabilir, şimdilik ihmal)

        if (r > (l2_ + l3_)) r = l2_ + l3_ - 0.001; // Hedef uzanılamayacak kadar uzaktaysa sınırla

        // Diz açısı (Knee) - İç açı
        double alpha_knee = std::acos((l2_*l2_ + l3_*l3_ - r*r) / (2 * l2_ * l3_));
        q[2] = -(M_PI - alpha_knee); // Robotlarda genelde diz ters döner

        // Kalça açısı (Thigh)
        double alpha_thigh = std::acos((l2_*l2_ + r*r - l3_*l3_) / (2 * l2_ * r));
        double angle_to_ground = std::atan2(x, -z); // Z ekseni aşağı negatif olduğu için -z
        q[1] = angle_to_ground - alpha_thigh; // Veya + duruma göre
    }

    void targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 12) return;

        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = this->now();
        traj_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        // 4 Bacak için IK hesapla
        for (int i = 0; i < 4; i++) {
            double x = msg->data[i*3 + 0];
            double y = msg->data[i*3 + 1];
            double z = msg->data[i*3 + 2];

            double q[3];
            // 1 ve 3 numaralı bacaklar (RF, RB) sağ taraftır
            bool is_right = (i == 1 || i == 3);

            computeIK(x, y, z, q, is_right);

            point.positions[i*3 + 0] = q[0]; // Hip
            point.positions[i*3 + 1] = q[1]; // Upper
            point.positions[i*3 + 2] = q[2]; // Lower
        }

        point.time_from_start = rclcpp::Duration::from_seconds(0.02); // 20ms'de git
        traj_msg.points.push_back(point);

        joint_pub_->publish(traj_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
    
    double l1_, l2_, l3_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleIKNode>());
    rclcpp::shutdown();
    return 0;
}
