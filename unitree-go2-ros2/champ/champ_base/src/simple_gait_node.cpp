#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <vector>

// Bu Node, cmd_vel dinler ve her ayak için X,Y,Z hedef koordinatlarını yayınlar.
// Guncelleme: Holonomik (Omnidirectional) yuruyus eklendi.
class SimpleGaitNode : public rclcpp::Node
{
public:
    SimpleGaitNode() : Node("simple_gait_node")
    {
        // cmd_vel dinleyicisi
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&SimpleGaitNode::velCallback, this, std::placeholders::_1));

        // Ayak hedeflerini yayınlayan publisher (Sıra: LF, RF, LB, RB -> x,y,z)
        target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("leg_targets_xyz", 10);

        // 50 Hz döngü
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SimpleGaitNode::controlLoop, this));

        // Varsayılan Ayakta Durma Pozisyonu (Unitree Go2 icin)
        nominal_height_ = -0.28; 
        step_height_ = 0.05; // Adım yüksekliği
        step_length_ = 0.10; // Adım uzunluğu çarpanı (X ve Y icin)
        
        // Ayaklarin govde merkezine gore nominal Y konumlari (govde genisligi/2 + hip offset)
        // Go2 icin yaklasik degerler:
        nominal_y_offset_ = 0.10; // Sol bacaklar icin pozitif, saglar icin negatif kullanacagiz
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

        // Hız komutu varsa (ileri/geri, sag/sol veya donus) sayaci ilerlet
        bool is_moving = (std::abs(current_vel_.linear.x) > 0.01 || 
                          std::abs(current_vel_.linear.y) > 0.01 || 
                          std::abs(current_vel_.angular.z) > 0.01);

        if (is_moving) {
            phase += 0.02 * (2 * M_PI * frequency);
        } else {
            // Hareket yoksa fazi sifirla veya durdugu yerde birak
            // Resetlemek ayagin havada kalmasini onler (basit bir cozum)
            phase = 0.0; 
        }

        std_msgs::msg::Float64MultiArray targets_msg;
        targets_msg.data.resize(12);

        // 4 Bacak için döngü (0:LF, 1:RF, 2:LB, 3:RB)
        for (int i = 0; i < 4; i++) {
            double leg_phase = phase;
            // Trot yürüyüşü: Çapraz bacaklar senkronize (0-3 ve 1-2)
            // 1(RF) ve 2(LB) bacaklarına PI faz farkı ekle
            if (i == 1 || i == 2) {
                leg_phase += M_PI;
            }

            // --- YORUNGE HESAPLAMA ---
            
            // 1. Z Ekseni (Yerden Kaldirma - Swing)
            double z_target = nominal_height_;
            if (is_moving && std::sin(leg_phase) > 0) {
                z_target += std::sin(leg_phase) * step_height_;
            }

            // 2. X Ekseni (Ileri - Geri)
            // Kosinus dalgasi: Ayak havadayken ileri, yerdeyken geri gider.
            // Robotun gitmesi gereken yonun tersine yeri itiyoruz.
            double x_target = -std::cos(leg_phase) * current_vel_.linear.x * step_length_;

            // 3. Y Ekseni (Sag - Sol - Holonomik)
            // Nominal durus pozisyonunu belirle
            double y_nominal = (i == 1 || i == 3) ? -nominal_y_offset_ : nominal_y_offset_;
            
            // Yana yurume icin X ile ayni mantik:
            // Kosinus dalgasi ile ayagi yana acip kapatiyoruz.
            // Eger linear.y pozitifse (sola git), ayaklar yerdeyken saga dogru itmeli.
            double y_motion = -std::cos(leg_phase) * current_vel_.linear.y * step_length_;
            
            // --- Donus (Yaw) Hareketi (Basit Yaklasim) ---
            // Donus icin ayaklar tegetsel hareket etmeli.
            // X hareketi Y konumundan, Y hareketi X konumundan etkilenir.
            // Basitlestirilmis: Sol tarafi ileri, sag tarafi geri itince saga doner.
            double turn_offset_x = 0.0;
            if (i == 0 || i == 2) turn_offset_x = -current_vel_.angular.z * 0.05; // Sol taraf
            else                  turn_offset_x =  current_vel_.angular.z * 0.05; // Sag taraf
            
            x_target += turn_offset_x;

            // Veriyi doldur
            targets_msg.data[i * 3 + 0] = x_target;
            targets_msg.data[i * 3 + 1] = y_nominal + y_motion; // Nominal ofsetin uzerine hareketi ekle
            targets_msg.data[i * 3 + 2] = z_target;
        }

        target_pub_->publish(targets_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist current_vel_;

    double nominal_height_;
    double nominal_y_offset_;
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
