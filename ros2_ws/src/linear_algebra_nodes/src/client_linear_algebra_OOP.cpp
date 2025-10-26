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

        // Load data from YAML and create service request
        if (!yaml2request()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML data. Shutting down.");
            rclcpp::shutdown();
            return;
        }
        
        // Wait for service to be available
        if (!wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Send service request
        send_service_request();
        
        // Start publishing timer
        timer_ = this->create_wall_timer(500ms, std::bind(&ClientPublisherNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "ClientPublisherNode initialized successfully!");

    }

  private:
    // ROS 2 interfaces
    rclcpp::Client<linear_algebra_msgs::srv::MagicInterface>::SharedPtr client_;
    rclcpp::Publisher<linear_algebra_msgs::msg::MagicVector>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data storage
    linear_algebra_msgs::srv::MagicInterface::Request::SharedPtr request_;
    linear_algebra_msgs::msg::MagicVector::SharedPtr message_;
    
    // Linear algebra matrices and vectors
    Eigen::Matrix3d rotation_matrix_;
    Eigen::Vector3d displacement_vector_;
    Eigen::Vector3d transformed_vector_;
    Eigen::Vector3d recovered_vector_;


    bool response_received_{false};                


    /**
     * @brief Initialize member variables with default values
     */
    void initialize_variables()
    {
        try
        {
            // Initialize member variables
            message_ = std::make_shared<linear_algebra_msgs::msg::MagicVector>();
            rotation_matrix_ = Eigen::Matrix3d::Identity();
            displacement_vector_ = Eigen::Vector3d::Zero();
            transformed_vector_ = Eigen::Vector3d::Zero();
            recovered_vector_ = Eigen::Vector3d::Zero();
            RCLCPP_INFO(this->get_logger(), "Member variables initialized successfully!");
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Member variables failed to initialize.");
        }
    }

    /**
     * @brief Wait for the service to become available
     * @param timeout Maximum time to wait for service
     * @return true if service is available, false otherwise
     */
    bool wait_for_service(std::chrono::seconds timeout)
    {
        const auto start_time = std::chrono::steady_clock::now();
        
        while (rclcpp::ok() && 
               std::chrono::steady_clock::now() - start_time < timeout) {
            
            if (client_->wait_for_service(5s)) {
                RCLCPP_INFO(this->get_logger(), "Service is available");
                return true;
            }
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for service 'do_magic' to become available...");
        }
        
        return false;
    }    

    /**
     * @brief Send service request to compute linear transformation
     */
    void send_service_request()
    {
        RCLCPP_INFO(this->get_logger(), "Sending service request...");
        
        auto future = client_->async_send_request(request_,
            std::bind(&ClientPublisherNode::response_callback, this, _1));
            
        RCLCPP_DEBUG(this->get_logger(), "Service request sent successfully");
    }


    /**
     * @brief Timer callback for publishing recovered vector
     */    
    void timerCallback(){
        if (!response_received_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for service response before publishing...");
            return;
        }

        try
        {
            // Update message with recovered vector
            message_->magic_vector.x = this->recovered_vector_(0);            
            message_->magic_vector.y = this->recovered_vector_(1);
            message_->magic_vector.z = this->recovered_vector_(2);

            // Publish message
            publisher_->publish(*message_);

            RCLCPP_INFO(this->get_logger(), "Published recovered vector: [x: %.3f, y: %.3f, z: %.3f]", message_->magic_vector.x, message_->magic_vector.y, message_->magic_vector.z);
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in timer callback.");
        }
    }

    /**
     * @brief Load data from YAML file and create service request
     * @return true if successful, false otherwise
     */    
    bool yaml2request()
    {
        try
        {
            // Get package-relative path to YAML file
            std::string yaml_path = ament_index_cpp::get_package_share_directory("linear_algebra_nodes") + "/config/data.yaml";
            RCLCPP_DEBUG(this->get_logger(), "Loading YAML file from: %s", yaml_path.c_str());

            YAML::Node config = YAML::LoadFile(yaml_path);
            request_ = std::make_shared<linear_algebra_msgs::srv::MagicInterface::Request>();

            // Load matrix A (points)
            if (!config["a"] || !config["a"].IsSequence()) {
                RCLCPP_ERROR(this->get_logger(), "Invalid or missing 'a' section in YAML");
                return false;
            }

            for (const auto& point_node : config["a"])
            {
                geometry_msgs::msg::Point point;
                point.x = point_node["x"].as<double>();
                point.y = point_node["y"].as<double>();
                point.z = point_node["z"].as<double>();
                request_->a.push_back(point);
            }

            // Load vector d
            if (!config["d"] || !config["d"].IsSequence()) {
                RCLCPP_ERROR(this->get_logger(), "Invalid or missing 'd' section in YAML");
                return false;
            }

            for (const auto& val_node : config["d"])
            {
                request_->d.push_back(val_node.as<double>());
            }

            if (request_->a.size() != request_->d.size())
            {
                RCLCPP_ERROR(this->get_logger(), 
                "Inconcistent matrices! Matrix A has %zu rows, Vector d has %zu elements",
                request_->a.size(), request_->d.size());
                throw;
            }

            RCLCPP_INFO(this->get_logger(), 
                "YAML data loaded: Matrix A has %zu rows, Vector d has %zu elements",
                request_->a.size(), request_->d.size());

            RCLCPP_DEBUG(this->get_logger(), 
                "First element of A: [%.3f, %.3f, %.3f], First element of d: %.3f",
                request_->a[0].x, request_->a[0].y, request_->a[0].z, request_->d[0]);

            return true;
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "YAML file failed to load.");
            return false;
        }

        

    }


    /**
     * @brief Callback for service response
     * @param future Future containing service response
     */
    void response_callback(rclcpp::Client<linear_algebra_msgs::srv::MagicInterface>::SharedFuture future)
    {

        try {
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Service response received successfully");

            // Extract rotation matrix from response
            for (int i {0} ; i<3 ; i++){
            rotation_matrix_(i,0) = response->rotation_matrix[i].x;
            rotation_matrix_(i,1) = response->rotation_matrix[i].y;
            rotation_matrix_(i,2) = response->rotation_matrix[i].z;
            }

            // Extract displacement vector
            displacement_vector_(0) = response->d_prime.x;
            displacement_vector_(1) = response->d_prime.y;
            displacement_vector_(2) = response->d_prime.z;

            // Extract transformed vector
            transformed_vector_(0) = response->x_transformed.x ;  
            transformed_vector_(1) = response->x_transformed.y; 
            transformed_vector_(2) = response->x_transformed.z;    

            // Recover original vector: x = Q^T * (x' - d')
            recovered_vector_ = rotation_matrix_.transpose() * (transformed_vector_ - displacement_vector_);

            RCLCPP_INFO(this->get_logger(), 
                "Vector recovered: [%.3f, %.3f, %.3f]", 
                recovered_vector_.x(), recovered_vector_.y(), recovered_vector_.z());

            response_received_ = true;             
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "Service Failure");
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