#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_msgs/srv/magic_interface.hpp"
#include "linear_algebra_msgs/msg/magic_vector.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <rclcpp/executor.hpp>


#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @class ServerSubscriberNode
 * @brief ROS 2 node that acts as both a service server and subscriber
 * 
 * This node:
 * - Provides service for linear algebra computations
 * - Computes least squares solutions and coordinate transformations
 * - Subscribes to vector messages for processing
 */
class ServerSubscriberNode : public rclcpp::Node 
{
public:

    ServerSubscriberNode() : rclcpp::Node("server_subscriber_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ServerSubscriberNode...");
        
        // Initialize service
        service_ = this->create_service<linear_algebra_msgs::srv::MagicInterface>("do_magic", std::bind(&ServerSubscriberNode::service_callback, this, _1, _2));

        // Initialize subscriber
        subscriber_ = this->create_subscription<linear_algebra_msgs::msg::MagicVector>("x_vector", 10, std::bind(&ServerSubscriberNode::subscriptionCallback, this, _1));

        // Start worker thread for message processing
        worker_thread_ = std::thread(&ServerSubscriberNode::worker_function, this);                                          //LLM

            
        RCLCPP_INFO(this->get_logger(), "ServerSubscriberNode initialized successfully. Service 'do_magic' is available.");

    }

    ~ServerSubscriberNode()
    {
        RCLCPP_DEBUG(this->get_logger(), "Shutting down ServerSubscriberNode...");              //LLM
        
        // Signal worker thread to stop
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_requested_ = true;
        }
        condition_variable_.notify_all();
        
        // Wait for worker thread to finish
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        
        RCLCPP_DEBUG(this->get_logger(), "ServerSubscriberNode shutdown complete");
    }


private:

    // ROS 2 interfaces
    rclcpp::Service<linear_algebra_msgs::srv::MagicInterface>::SharedPtr service_;
    rclcpp::Subscription<linear_algebra_msgs::msg::MagicVector>::SharedPtr subscriber_;

    // Linear algebra data
    Eigen::MatrixXd matrix_A_;
    Eigen::VectorXd vector_d_;
    Eigen::Vector3d solution_vector_;
    Eigen::Matrix3d rotation_matrix_;
    Eigen::Vector3d transformed_vector_;
    Eigen::Vector3d displacement_vector_;

    // Threading and synchronization
    std::thread worker_thread_;
    std::mutex mutex_;
    std::condition_variable condition_variable_;
    linear_algebra_msgs::msg::MagicVector last_message_;
    std::atomic<bool> new_message_available_{false};
    std::atomic<bool> shutdown_requested_{false};


    /**
     * @brief Service callback for linear algebra computations
     * @param request Service request containing matrix A and vector vector_d_
     * @param response Service response containing transformed data
     */
    void service_callback(const linear_algebra_msgs::srv::MagicInterface::Request::SharedPtr request,
                            linear_algebra_msgs::srv::MagicInterface::Response::SharedPtr response)
    {

        RCLCPP_INFO(this->get_logger(), 
            "Received service request: Matrix A (%zu points), Vector d (%zu elements)",
            request->a.size(), request->d.size());

        try
        {
            // Convert request to Eigen matrices
            if (!request2variables(request)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to process request data");
                return;
            }

            // Generate random rotation matrix and displacement vector            
            random_Q_dprime();

            // Compute least squares solution: solution_vector_ = matrix_A_ \ vector_d_
            solution_vector_ = matrix_A_.colPivHouseholderQr().solve(vector_d_);

            RCLCPP_INFO(this->get_logger(),
                "Least squares solution: [%.6f, %.6f, %.6f]",
                solution_vector_.x(), solution_vector_.y(), solution_vector_.z());

            // Apply transformation: x' = rotation_matrix_ * solution_vector_ + displacement_vector_
            transformed_vector_ = rotation_matrix_ * solution_vector_ + displacement_vector_;
            
            // Convert results to response
            variables2response(response);

            RCLCPP_INFO(this->get_logger(),
                "Transformation complete. Sent x': [%.6f, %.6f, %.6f]",
                response->x_transformed.x, response->x_transformed.y, response->x_transformed.z);
        }
        catch (...) {
            RCLCPP_ERROR(this->get_logger(),
                "Service computation failed.");
        }
    }

    /**
     * @brief Subscription callback for vector messages
     * @param msg Received message containing magic vector
     */
    void subscriptionCallback(linear_algebra_msgs::msg::MagicVector::SharedPtr msg)
    {
        try {
            std::lock_guard<std::mutex> lock(mutex_);                                         //LLM
            last_message_ = *msg;
            new_message_available_ = true;
            condition_variable_.notify_one();

            RCLCPP_DEBUG(this->get_logger(),
                "Received vector: [%.3f, %.3f, %.3f]",
                msg->magic_vector.x, msg->magic_vector.y, msg->magic_vector.z);
        }
        catch (...) {
            RCLCPP_ERROR(this->get_logger(),
                "Error in subscription callback.");
        }
    }


    /**
     * @brief Convert service request to Eigen matrices
     * @param request Service request containing input data
     * @return true if conversion successful, false otherwise
     */
    bool request2variables(const linear_algebra_msgs::srv::MagicInterface::Request::SharedPtr request)
    {
        try
        {
            // Resize and populate matrix A
            matrix_A_.resize(request->a.size(), 3);
            for (size_t i = 0; i < request->a.size(); i++) 
            {
                matrix_A_(i, 0) = request->a[i].x;
                matrix_A_(i, 1) = request->a[i].y;
                matrix_A_(i, 2) = request->a[i].z;
            }

            // Resize and populate vector d
            vector_d_.resize(request->d.size());
            for (size_t i = 0; i < request->d.size(); i++) 
            {
                vector_d_(i) = request->d[i];
            }

            RCLCPP_INFO(this->get_logger(), "Converted request to Matrix A and Vector.");
            return true;
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting request to variables.");
            return false;
        }
    }

    /**
     * @brief Generate random rotation matrix and displacement vector
     */
    void random_Q_dprime()
    {
        try
        {
            // Generate random matrix and compute QR decomposition
            Eigen::Matrix3d mat = Eigen::Matrix3d::Random();
            Eigen::HouseholderQR<Eigen::Matrix3d> qr(mat);
            rotation_matrix_ = qr.householderQ();

            // Ensure it's a proper rotation (det = +1)
            if (rotation_matrix_.determinant() < 0) 
            {
                rotation_matrix_.col(0) *= -1;
            }

            // Generate random displacement vector
            displacement_vector_ = Eigen::Vector3d::Random();

            RCLCPP_INFO(this->get_logger(), "Random matrices generation: COMPLETED.");
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "Random matrices generation: FAILED.");
            throw;
        }
    }

    /**
     * @brief Convert computation results to service response
     * @param response Service response to populate
     */    
    void variables2response(linear_algebra_msgs::srv::MagicInterface::Response::SharedPtr response   )
    {
        try
        {
            // Populate rotation matrix (3x3)
            for (int i {0} ; i<3 ; i++)
            {
            response->rotation_matrix[i].x = rotation_matrix_(i, 0);
            response->rotation_matrix[i].y = rotation_matrix_(i, 1);
            response->rotation_matrix[i].z = rotation_matrix_(i, 2);
            }

            // Populate displacement vector
            response->d_prime.x = displacement_vector_(0);
            response->d_prime.y = displacement_vector_(1);
            response->d_prime.z = displacement_vector_(2);

            // Populate transformed vector
            response->x_transformed.x = transformed_vector_(0);  
            response->x_transformed.y = transformed_vector_(1);  
            response->x_transformed.z = transformed_vector_(2); 

            RCLCPP_INFO(this->get_logger(), "Converted variables to service response.");
        }
        catch(...)
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting variables to response.");
            throw;
        }
    }

    /**
     * @brief Worker thread function for processing subscribed messages
     */
    void worker_function()
    {
        RCLCPP_DEBUG(this->get_logger(), "Worker thread started");
        
        while (rclcpp::ok() && !shutdown_requested_) {
            std::unique_lock<std::mutex> lock(mutex_);
            
            // Wait for new messages or shutdown signal
            condition_variable_.wait(lock, [this]() {
                return new_message_available_ || shutdown_requested_;
            });
            
            if (shutdown_requested_) {
                break;
            }
            
            if (new_message_available_) {
                // Process the received message
                linear_algebra_msgs::msg::MagicVector message = last_message_;
                new_message_available_ = false;
                
                lock.unlock(); // Release lock during processing
                
                // Process the message (example: log it)
                RCLCPP_INFO(this->get_logger(),
                    "Worker processing vector: [%.6f, %.6f, %.6f]",
                    message.magic_vector.x, message.magic_vector.y, message.magic_vector.z);
                
                // Here you could add more complex processing logic
                // For example: store in buffer, perform computations, etc.
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Worker thread finished");
    }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServerSubscriberNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}