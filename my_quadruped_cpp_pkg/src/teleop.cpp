#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <map>

// Terminalden karakter okumak için yardımcı fonksiyon
int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class CustomTeleopNode : public rclcpp::Node {
public:
    CustomTeleopNode() : Node("custom_teleop_node") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        std::cout << "---------------------------------------------" << std::endl;
        std::cout << "İTÜ Kontrol - Quadruped Custom Teleop" << std::endl;
        std::cout << "---------------------------------------------" << std::endl;
        std::cout << "W/S: İleri/Geri | A/D: Dönüş" << std::endl;
        std::cout << "1: Mod 1 (Normal) | 2: Mod 2 (Merdiven) | 3: Mod 3 (Gövde)" << std::endl;
        std::cout << "BOŞLUK: Durdur | CTRL+C: Çıkış" << std::endl;
        std::cout << "---------------------------------------------" << std::endl;

        run_teleop();
    }

private:
    void run_teleop() {
        geometry_msgs::msg::Twist twist;
        double speed = 0.2; // m/s
        double turn = 0.5;  // rad/s

        while (rclcpp::ok()) {
            int key = getch();

            // Tuş Kombinasyonları
            if (key == 'w' || key == 'W') {
                twist.linear.x = speed;
            } else if (key == 's' || key == 'S') {
                twist.linear.x = -speed;
            } else if (key == 'a' || key == 'A') {
                twist.angular.z = turn;
            } else if (key == 'd' || key == 'D') {
                twist.angular.z = -turn;
            } else if (key == '1') {
                twist.linear.z = 1.0; // Mod 1 tetikleyici
                std::cout << ">> MOD: Normal Yürüyüş Aktif" << std::endl;
            } else if (key == '2') {
                twist.linear.z = 0.5; // Mod 2 tetikleyici
                std::cout << ">> MOD: Merdiven Dizisi Başlatıldı" << std::endl;
            } else if (key == '3') {
                twist.linear.z = -0.5; // Mod 3 tetikleyici
                std::cout << ">> MOD: Gövde Kaydırma Aktif" << std::endl;
            } else if (key == ' ') {
                twist = geometry_msgs::msg::Twist(); // Her şeyi sıfırla
                std::cout << ">> DURDURULDU" << std::endl;
            } else if (key == 3) { // CTRL+C
                break;
            }

            pub_->publish(twist);
            
            // Hareket tuşlarını yayınladıktan sonra hızı sıfırla (momentumsuz kontrol)
            // Mod tuşları (linear.z) basılı kalabilir.
            if (key == 'w' || key == 's' || key == 'a' || key == 'd') {
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                pub_->publish(twist);
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomTeleopNode>();
    rclcpp::shutdown();
    return 0;
}
