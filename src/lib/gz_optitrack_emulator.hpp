#ifndef GZ_OPTITRACK_EMULATOR_HPP
#define GZ_OPTITRACK_EMULATOR_HPP

#include <iostream>
#include <array>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/odometry.pb.h> // gz::msgs::Odometry

#include "gz_optitrack_ros2_emulator/msg/mocap.hpp"

class GazeboMocapEmulator : public rclcpp::Node
{
public:
    explicit GazeboMocapEmulator(const rclcpp::NodeOptions & options);
    using ModeList = std::vector<std::string>;
    using TopicPair = std::pair<std::string, std::string>;
    // used to construct a publisher map
    using MocapType = gz_optitrack_ros2_emulator::msg::Mocap;
    using MocapPub = rclcpp::Publisher<MocapType>::SharedPtr;
    using GzMsgType = gz::msgs::Odometry;
private:
    static constexpr const char* modelListName = "gz_model_list";
    static constexpr uint32_t qos = 10u; // quality of service in the publisher and subscriber
    // load model names
    void LoadParameters();
    // construct the gz_topic_name and ros_topic_name according to the model name
    void ConstructTopics();
    void SetupNode();
    void GzCallBack(const std::string &model, const GzMsgType &msg);
    ModeList modelList;
    std::vector<TopicPair> topicPair;
    gz::transport::Node gzNode;
    std::map<std::string, MocapPub> publisherMap;
};

// using namespace Eigen;
// class Mocap_emulator{
//     public:
//         Mocap_emulator(const std::string& pub_topic_name,
//                        const std::string& sub_topic_name,
//                        std::shared_ptr<rclcpp::Node> node,
//                        size_t queue_size);
//         ~Mocap_emulator();
//     private:
//         void PublishData();
//         void SubscribeFromGazebo(const gz::msgs::Odometry& msg);
//         rclcpp::Publisher<optitrack_broadcast::msg::Mocap>::SharedPtr pubmocap_;// publishing the recieved data
//         rclcpp::Subscription<gz::msgs::Odometry>::SharedPtr      subgazebo_;// receiving
//         optitrack_broadcast::msg::Mocap                             MessageMocap_;
//         gz::msgs::Odometry                                    Drone_state_;
//         Matrix3f R_IB;
//         Vector4f quaternion;
//         Vector3f omega_b;// angular velocity in body-fixed frame
//         Vector3f omega_i;// angular velocity in inertial frame
// };
#endif
