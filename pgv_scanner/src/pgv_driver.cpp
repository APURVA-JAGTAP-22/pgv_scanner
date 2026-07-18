#include <rclcpp/rclcpp.hpp>               //core ros2 function
#include <std_msgs/msg/int32.hpp>          //msg type for Tag ID and quality
#include <std_msgs/msg/bool.hpp>           //msg type for tag detection status
#include <geometry_msgs/msg/pose2_d.hpp>   //msg type for x,y,theta 

#include <sys/socket.h>              //linux networking (sockets)
#include <arpa/inet.h>               //IP address conversion
#include <unistd.h>                 //standrad linux utilities
#include <cstring>
#include <stdexcept>
#include <vector>

#define PGV_IP   "192.168.1.30"
#define PGV_PORT 50020
#define LEN      21     //sensor send 21 byte

static const uint8_t POS_REQ[2] = {0xC8, 0x37};      //position register which tells current location.

// Receive exact bytes
std::vector<uint8_t> recv_exact(int sock, int n)
{
    std::vector<uint8_t> data;
    data.reserve(n);

    while ((int)data.size() < n) {
        uint8_t buffer[256];
        int r = recv(sock, buffer, n - data.size(), 0);
        if (r <= 0) throw std::runtime_error("PGV disconnected");
        data.insert(data.end(), buffer, buffer + r);
    }
    return data;
}

// Signed conversion
int32_t sgn(int32_t v, int b)
{
    if (v >= (1 << (b - 1)))
        return v - (1 << b);
    return v;
}

class PGVNode : public rclcpp::Node
{
public:
    PGVNode() : Node("pgv_driver"), sock_(-1)
    {
        pose_pub_ = create_publisher<geometry_msgs::msg::Pose2D>("pgv_pose", 10);
        tag_pub_  = create_publisher<std_msgs::msg::Int32>("pgv_tag_id", 10);
        ok_pub_   = create_publisher<std_msgs::msg::Bool>("pgv_tag_detected", 10);
        qual_pub_ = create_publisher<std_msgs::msg::Int32>("pgv_quality", 10);

        sock_ = socket(AF_INET, SOCK_STREAM, 0);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(PGV_PORT);
        inet_pton(AF_INET, PGV_IP, &addr.sin_addr);

        if (connect(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "PGV connection failed");
            sock_ = -1;
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),         //20      ( 1000 / interval in ms --> for 50 Hz )      100 Hz max limit
            std::bind(&PGVNode::read_pgv, this)
        );

        RCLCPP_INFO(get_logger(), "PGV Driver Started");
    }

private:
    void read_pgv()
    {
        if (sock_ < 0) return;

        try {
            send(sock_, POS_REQ, sizeof(POS_REQ), 0);
            auto d = recv_exact(sock_, LEN);

            uint8_t status = d[0];
            bool tag_ok = (d[1] >> 6) & 1;

            double x = sgn(((d[2]&7)<<21)|((d[3]&127)<<14)|((d[4]&127)<<7)|(d[5]&127),24)/10.0;
            double y = sgn(((d[6]&127)<<7)|(d[7]&127),14)/10.0;
            if (status & 0x02) y *= -1;

            double theta = sgn(((d[10]&127)<<7)|(d[11]&127),14)/10.0;

            geometry_msgs::msg::Pose2D pose;
            pose.x = x;
            pose.y = y;
            pose.theta = theta;
            pose_pub_->publish(pose);

            std_msgs::msg::Int32 tag;
            tag.data = d[17];
            tag_pub_->publish(tag);

            std_msgs::msg::Bool ok;
            ok.data = tag_ok;
            ok_pub_->publish(ok);

            std_msgs::msg::Int32 q;
            q.data = d[18];
            qual_pub_->publish(q);

        } catch (...) {
            RCLCPP_WARN(get_logger(), "PGV read error");
        }
    }

    int sock_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tag_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ok_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr qual_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGVNode>());
    rclcpp::shutdown();
    return 0;
}
