#include <rclcpp/rclcpp.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

class PoseInitializerClientNode : public rclcpp::Node
{
    public:
    PoseInitializerClientNode() : Node("pose_initializer_client")
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_;
        pose_.header.frame_id = "map";
        pose_.header.stamp = rclcpp::Time();
        pose_.pose.pose.position.x = 0.0;
        pose_.pose.pose.position.y = 0.0;
        pose_.pose.pose.position.z = 0.0;
        pose_.pose.pose.orientation.x = 0.0;
        pose_.pose.pose.orientation.y = 0.0;
        pose_.pose.pose.orientation.z = 0.0;
        pose_.pose.pose.orientation.w = 0.0;
        for (int i = 0; i < 35; i++) {
            pose_.pose.covariance[i] = 0.0;
        }

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose3d_pub", 10);
        pose_pub_->publish(pose_);
        thread1_ = std::thread(std::bind(&PoseInitializerClientNode::callbackTriggerService, this));
    }

    void callbackTriggerService()
    {
        auto client_ekf_trigger_ = this->create_client<std_srvs::srv::SetBool>("/localization/twist_estimator/ekf_trigger_node_mod");
                
        while (!client_ekf_trigger_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the EKF-Trigger to be up");
        }
        
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        auto future_ekf = client_ekf_trigger_->async_send_request(req);

        try
        {
            auto res = future_ekf.get();
            RCLCPP_INFO(this->get_logger(), "EKF status is %d", res->success);
            trigger_ekf = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "EKF setup failed");
        }
    }

    private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    std::thread thread1_;
    bool trigger_ekf = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseInitializerClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}