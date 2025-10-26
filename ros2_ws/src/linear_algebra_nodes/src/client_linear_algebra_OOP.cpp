#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_msgs/srv/magic_interface.hpp"
#include "linear_algebra_msgs/msg/magic_vector.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <rclcpp/executor.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;


/**
 * @class ClientPublisherNode
 * @brief ROS 2 node that acts as both a service client and publisher
 * 
 * This node:
 * - Reads matrix A and vector d from YAML configuration
 * - Sends service request to compute linear transformation
 * - Publishes recovered vector x to topic
 */
class ClientPublisherNode : public rclcpp::Node 
{
  public:
    ClientPublisherNode(): rclcpp::Node("client_publisher_node") 
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ClientPublisherNode...");

        // Initialize service client
        client_ = this->create_client<linear_algebra_msgs::srv::MagicInterface>("do_magic");

        // Initialize publisher
        publisher_ = this->create_publisher<linear_algebra_msgs::msg::MagicVector>("x_vector", 10);

        // Initialize member variables
        initialize_variables();

        
        yaml2request();

        while(!client_->wait_for_service(2s)){
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }

        auto response = client_->async_send_request(request, std::bind(&ClientPublisherNode::responseCallback, this, _1));    


        timer_ = this->create_wall_timer(500ms, std::bind(&ClientPublisherNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Contruction of ClientPublisherNode : COMPLETED!");
        rclcpp::sleep_for(2s);

    }

  private:
    rclcpp::Client<linear_algebra_msgs::srv::MagicInterface>::SharedPtr client_;
    rclcpp::Publisher<linear_algebra_msgs::msg::MagicVector>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    linear_algebra_msgs::srv::MagicInterface::Request::SharedPtr request;
    linear_algebra_msgs::msg::MagicVector::SharedPtr message;
    Eigen::Matrix3d Q;
    Eigen::Vector3d d_prime;
    Eigen::Vector3d x_transformed;
    Eigen::Vector3d x_recovered;


    bool response_received_{false};                    //LLM



    void initialize_variables(){
        try
        {
            // Initialize member variables
            message = std::make_shared<linear_algebra_msgs::msg::MagicVector>();
            Q = Eigen::Matrix3d::Identity();
            d_prime = Eigen::Vector3d::Zero();
            x_transformed = Eigen::Vector3d::Zero();
            x_recovered = Eigen::Vector3d::Zero();
           RCLCPP_INFO(this->get_logger(), "The variables initialized successfully!");
            rclcpp::sleep_for(1s);
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "The variables failed to initialize.");
            rclcpp::sleep_for(1s);
        }
    }


    void timerCallback(){
        if (!response_received_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service response...");
            return;
        }

        try
        {
            message->magic_vector.x = this->x_recovered(0);            
            message->magic_vector.y = this->x_recovered(1);
            message->magic_vector.z = this->x_recovered(2);

            publisher_->publish(*message);
            RCLCPP_INFO(this->get_logger(), "Send x to topic: [x = %f, y = %f, z = %f]", message->magic_vector.x, message->magic_vector.y, message->magic_vector.z);
            rclcpp::sleep_for(1s);
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Msg failed to reach the topic.");
            rclcpp::sleep_for(1s);
        }
    }

    void yaml2request()
    {
        try
        {
            // std::string yaml_path = "/home/andreas/ProgRobots/ros2_ws/src/linear_algebra_nodes/config/data.yaml";
            std::string yaml_path = ament_index_cpp::get_package_share_directory("linear_algebra_nodes") + "/config/data.yaml";
            YAML::Node config = YAML::LoadFile(yaml_path);

            request = std::make_shared<linear_algebra_msgs::srv::MagicInterface::Request>();
            // --- Load points (matrix A) ---
            for (const auto& point_node : config["a"])
            {
                geometry_msgs::msg::Point p;
                p.x = point_node["x"].as<double>();
                p.y = point_node["y"].as<double>();
                p.z = point_node["z"].as<double>();
                request->a.push_back(p);
            }
                // --- Load vector d ---
            for (const auto& val_node : config["d"])
            {
                request->d.push_back(val_node.as<double>());
            }
            RCLCPP_INFO(this->get_logger(), "YAML file loaded successfully!   The first element of A is: %f.  The first element of d is: %f", request->a[0].x, request->d[0]);
            rclcpp::sleep_for(1s);
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "YAML file failed to load.");
            rclcpp::sleep_for(1s);
        }

        

    }

    void responseCallback(rclcpp::Client<linear_algebra_msgs::srv::MagicInterface>::SharedFuture future)
    {
        if (!future.valid())
        {
            RCLCPP_ERROR(this->get_logger(), "Service Failure");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "THEY MADE IT! We just got the response....");

            for (int i {0} ; i<3 ; i++){
            Q(i,0) = future.get()->rotation_matrix[i].x;
            Q(i,1) = future.get()->rotation_matrix[i].y;
            Q(i,2) = future.get()->rotation_matrix[i].z;
            }
            d_prime(0) = future.get()->d_prime.x;
            d_prime(1) = future.get()->d_prime.y;
            d_prime(2) = future.get()->d_prime.z;

            x_transformed(0) = future.get()->x_transformed.x ;  
            x_transformed(1) = future.get()->x_transformed.y; 
            x_transformed(2) = future.get()->x_transformed.z;    

            x_recovered = Q.transpose() * (x_transformed - d_prime);


            response_received_ = true;                    //LLM
        }

    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientPublisherNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}