#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <vector>

// Bu Node, cmd_vel dinler ve her ayak için X,Y,Z hedef koordinatlarını yayınlar.
class SimpleGaitNode : public rclcpp::Node
{
public:
    SimpleGaitNode() : Node("simple_gait_node")
    {
        // cmd_vel dinleyicisi
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&SimpleGaitNode::velCallback, this, std::placeholders::_1));

        // Ayak hedeflerini yayınlayan publisher (Sıra: LF, RF, LB, RB -> x,y,z)
        // Toplam 12 veri: [lf_x, lf_y, lf_z, rf_x, rf_y, rf_z, ...]
        target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("leg_targets_xyz", 10);

        // 50 Hz döngü
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SimpleGaitNode::controlLoop, this));

        // Varsayılan Ayakta Durma Pozisyonu (Robotun kendi bacak ölçülerine göre ayarlanmalı)
        // Hip eksenine göre: X=0, Y=0 (veya hafif yana), Z=-0.30 (aşağı)
        nominal_height_ = -0.30; 
        step_height_ = 0.05; // Adım atarken ayağın ne kadar kalkacağı
        step_length_ = 0.10; // Adım uzunluğu çarpanı
    }

private:
    void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_vel_ = *msg;
    }

    void controlLoop()
    {
        // Zaman sayacı (Gait fazını hesaplamak için)
        static double phase = 0.0;
        double frequency = 2.0; // Adım atma hızı (Hz)

        // Eğer hız çok düşükse fazı durdur (yerinde sayma)
        if (std::abs(current_vel_.linear.x) > 0.01 || std::abs(current_vel_.angular.z) > 0.01) {
            phase += 0.02 * (2 * M_PI * frequency); // dt * omega
        }

        std_msgs::msg::Float64MultiArray targets_msg;
        targets_msg.data.resize(12);

        // 4 Bacak için döngü (0:LF, 1:RF, 2:LB, 3:RB)
        // Trot yürüyüşü: Çapraz bacaklar senkronize (0-3 ve 1-2)
        for (int i = 0; i < 4; i++) {
            double leg_phase = phase;
            // Çapraz bacaklara PI kadar faz farkı ekle (Biri havadayken diğeri yerde)
            if (i == 1 || i == 2) {
                leg_phase += M_PI;
            }

            double x_target = 0.0;
            double z_target = nominal_height_;

            // Basit Sinüs Yörüngesi
            // X ekseni: İleri geri hareket (hıza bağlı)
            x_target = -std::cos(leg_phase) * current_vel_.linear.x * step_length_;

            // Z ekseni: Sadece fazın yarısında ayağı kaldır (Swing phase)
            // sin(phase) > 0 ise ayak havada, değilse yerde
            if (std::sin(leg_phase) > 0) {
                z_target += std::sin(leg_phase) * step_height_;
            }

            // Dönüş (Angular Z) için basit bir Y ofseti eklenebilir (Burada sade tuttum)
            double y_target = 0.0; 
            // Sağ bacaklar (RF, RB) için y ekseni negatiftir (mekanik yapıya göre değişir)
            if (i == 1 || i == 3) y_target = -0.10; // Sağ taraf ofseti
            else y_target = 0.10; // Sol taraf ofseti

            // Veriyi doldur
            targets_msg.data[i * 3 + 0] = x_target;
            targets_msg.data[i * 3 + 1] = y_target;
            targets_msg.data[i * 3 + 2] = z_target;
        }

        target_pub_->publish(targets_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist current_vel_;

    double nominal_height_;
    double step_height_;
    double step_length_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGaitNode>());
    rclcpp::shutdown();
    return 0;
}
