#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Dense>
#include <time.h>

class TFPub : public rclcpp::Node
{
public:
    TFPub()
        : Node("tf2_publisher")
    {
        // 相机相对于车辆的旋转矩阵
        // （相机朝前水平放置）
        Eigen::Matrix3d R_camera_vehicle;
        R_camera_vehicle << 0.0, -1.0, 0.0,
            0.0, 0.0, -1.0,
            1.0, 0.0, 0.0;

        // 相机相对于车辆的平移向量
        // (相机放置在底盘中心前方25cm处)
        Eigen::Vector3d t_camera_vehicle_(0.0, 0.0, -0.25);

        // 订阅odom话题
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/baton/loop/odometry", 10, std::bind(&TFPub::odomCallback, this, std::placeholders::_1));
        // 发布转换后的位姿
        tran_pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/car_pose", 10);
        // 创建定时器回调
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TFPub::timerCallback, this));

        // 构建外参矩阵 T_cam_to_vehicle
        Eigen::Affine3d T_cam_to_vehicle = Eigen::Affine3d::Identity();
        T_cam_to_vehicle.translation() = t_camera_vehicle_;
        T_cam_to_vehicle.linear() = R_camera_vehicle;
        T_cam_to_vehicle_ = T_cam_to_vehicle;

        // 输出外参矩阵
        std::cout << "T_cam_to_vehicle:\n"
                  << T_cam_to_vehicle.matrix() << std::endl;

        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 初始化静态TF广播器
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

        // 发布静态TF变换 odom -> map
        geometry_msgs::msg::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = this->get_clock()->now();
        static_transformStamped.header.frame_id = "map";
        static_transformStamped.child_frame_id = "odom";
        // 假设 odom 初始位置在 map 中的原点
        static_transformStamped.transform.translation.x = 0.0;
        static_transformStamped.transform.translation.y = 0.0;
        static_transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0); // 假设没有旋转
        static_transformStamped.transform.rotation.x = quat.x();
        static_transformStamped.transform.rotation.y = quat.y();
        static_transformStamped.transform.rotation.z = quat.z();
        static_transformStamped.transform.rotation.w = quat.w();
        static_tf_broadcaster_->sendTransform(static_transformStamped);

        // 初始化current_pose_
        current_pose_.position.x = 0.0;
        current_pose_.position.y = 0.0;
        current_pose_.position.z = 0.0;
        current_pose_.orientation.x = 0.0;
        current_pose_.orientation.y = 0.0;
        current_pose_.orientation.z = 0.0;
        current_pose_.orientation.w = 1.0;

        std::cout << "========start========" << std::endl;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 将相机位姿转换到车辆旋转中心
        current_pose_ = transformPose(msg->pose.pose);
    }

    void timerCallback()
    {
        // 发布TF广播
        geometry_msgs::msg::TransformStamped base_to_odom;
        base_to_odom.header.stamp = this->get_clock()->now();
        base_to_odom.header.frame_id = "odom";
        base_to_odom.child_frame_id = "base_link";

        base_to_odom.transform.translation.x = current_pose_.position.x;
        base_to_odom.transform.translation.y = current_pose_.position.y;
        base_to_odom.transform.translation.z = current_pose_.position.z;

        base_to_odom.transform.rotation.x = current_pose_.orientation.x;
        base_to_odom.transform.rotation.y = current_pose_.orientation.y;
        base_to_odom.transform.rotation.z = current_pose_.orientation.z;
        base_to_odom.transform.rotation.w = current_pose_.orientation.w;

        tf_broadcaster_->sendTransform(base_to_odom);
    }

private:
    geometry_msgs::msg::Pose transformPose(const geometry_msgs::msg::Pose &pose)
    {
        // 将geometry_msgs::Pose转换为Eigen::Affine3d
        Eigen::Affine3d pose_eigen;
        pose_eigen.translation() << pose.position.x, pose.position.y, pose.position.z;
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        pose_eigen.linear() = q.toRotationMatrix();

        // 应用变换
        Eigen::Affine3d transformed_pose_eigen = pose_eigen * T_cam_to_vehicle_;

        // 将Eigen::Affine3d转换回geometry_msgs::Pose
        geometry_msgs::msg::Pose transformed_pose;
        transformed_pose.position.x = transformed_pose_eigen.translation().x();
        transformed_pose.position.y = transformed_pose_eigen.translation().y();
        transformed_pose.position.z = transformed_pose_eigen.translation().z();
        Eigen::Quaterniond transformed_q(transformed_pose_eigen.linear());
        transformed_q.normalize();
        transformed_pose.orientation.w = transformed_q.w();
        transformed_pose.orientation.x = transformed_q.x();
        transformed_pose.orientation.y = transformed_q.y();
        transformed_pose.orientation.z = transformed_q.z();

        nav_msgs::msg::Odometry transformed_pose_msg;
        transformed_pose_msg.header.stamp = this->get_clock()->now();
        transformed_pose_msg.header.frame_id = "odom";
        transformed_pose_msg.pose.pose = transformed_pose;
        tran_pose_pub_->publish(transformed_pose_msg);

        return transformed_pose;
    }

    Eigen::Affine3d T_cam_to_vehicle_; // 外参矩阵
    geometry_msgs::msg::Pose current_pose_; // 当前位姿
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tran_pose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFPub>());
    rclcpp::shutdown();
    return 0;
}