#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_msgs/srv/magic_interface.hpp"
#include "linear_algebra_msgs/msg/magic_vector.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <rclcpp/executor.hpp>


#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


class ServerSubscriberNode : public rclcpp::Node 
{
    public:

        ServerSubscriberNode() : rclcpp::Node("server_subscriber_node")
        {
            service_ = this->create_service<linear_algebra_msgs::srv::MagicInterface>("do_magic", std::bind(&ServerSubscriberNode::serviceCallback, this, _1, _2));

            subscriber_ = this->create_subscription<linear_algebra_msgs::msg::MagicVector>("x_vector", 10, std::bind(&ServerSubscriberNode::subscriptionCallback, this, _1));


            worker_thread_ = std::thread(&ServerSubscriberNode::workerFunction, this);                                          //LLM

                
            RCLCPP_INFO(this->get_logger(), "Contruction of ServerSubscriberNode : COMPLETED \n");

        }

        ~ServerSubscriberNode()                                                                 //LLM
        {
            if (worker_thread_.joinable()) {
                worker_thread_.join();
        }
}

    private:

        rclcpp::Service<linear_algebra_msgs::srv::MagicInterface>::SharedPtr service_;
        rclcpp::Subscription<linear_algebra_msgs::msg::MagicVector>::SharedPtr subscriber_;
    // Direct member variables - much cleaner!
        Eigen::MatrixXd A;
        Eigen::VectorXd d;
        Eigen::Vector3d x;
        Eigen::Matrix3d Q;
        Eigen::Vector3d x_transformed;
        Eigen::Vector3d d_prime;



        std::mutex mtx_;                                                                                     //LLM
        std::condition_variable cv_;
        bool new_message_ = false;
        linear_algebra_msgs::msg::MagicVector last_msg_;
        std::thread worker_thread_;




        void serviceCallback(const linear_algebra_msgs::srv::MagicInterface::Request::SharedPtr request,
                             linear_algebra_msgs::srv::MagicInterface::Response::SharedPtr response)
        {
            request2variables(request);
            random_Q_dprime();
            //Calculation of least square vector
            x = A.colPivHouseholderQr().solve(d);

            RCLCPP_INFO(this->get_logger(), "The least square x is: [ %f , %f , %f ] ", x(0), x(1), x(2));

            x_transformed = Q * x + d_prime;
            variables2response(response);

            RCLCPP_INFO(this->get_logger(), "The x' is sent to the client [ %f , %f , %f ]. \n", response->x_transformed.x, response->x_transformed.y, response->x_transformed.z);
            rclcpp::sleep_for(2s);

        }

        void subscriptionCallback(linear_algebra_msgs::msg::MagicVector::SharedPtr msg)
        {

            std::lock_guard<std::mutex> lock(mtx_);                                         //LLM
            last_msg_ = *msg;
            new_message_ = true;
            cv_.notify_one();
        }



        void request2variables(const linear_algebra_msgs::srv::MagicInterface::Request::SharedPtr request)
        {
            try
            {
                A.resize(request->a.size(), 3);
                for (int i = 0; i < request->a.size(); i++) 
                {
                    A(i, 0) = request->a[i].x;
                    A(i, 1) = request->a[i].y;
                    A(i, 2) = request->a[i].z;
                }

                d.resize(request->d.size());
                for (int i = 0; i < request->d.size(); i++) 
                {
                    d(i) = request->d[i];
                }
                RCLCPP_INFO(this->get_logger(), "YAML file data acquisition: COMPLETED.");
                rclcpp::sleep_for(1s);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "YAML file data acquisition: FAILED.");
                rclcpp::sleep_for(1s);
            }
        }

        void random_Q_dprime()
        {
            try
            {
                Eigen::Matrix3d mat = Eigen::Matrix3d::Random();
                Eigen::HouseholderQR<Eigen::Matrix3d> qr(mat);
                Q = qr.householderQ();

                // Ensure it's a proper rotation (det = +1)
                if (Q.determinant() < 0) 
                {
                    Q.col(0) *= -1;
                }

                d_prime = Eigen::Vector3d::Random();
                RCLCPP_INFO(this->get_logger(), "Random matrices generation: COMPLETED.");
                rclcpp::sleep_for(1s);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Random matrices generation: FAILED.");
                rclcpp::sleep_for(1s);
            }
        }

        void variables2response(linear_algebra_msgs::srv::MagicInterface::Response::SharedPtr response   )
        {
            try
            {
                for (int i {0} ; i<3 ; i++)
                {
                response->rotation_matrix[i].x = Q(i, 0);
                response->rotation_matrix[i].y = Q(i, 1);
                response->rotation_matrix[i].z = Q(i, 2);
                }

                response->d_prime.x = d_prime(0);
                response->d_prime.y = d_prime(1);
                response->d_prime.z = d_prime(2);


                response->x_transformed.x = x_transformed(0);  // Column 0
                response->x_transformed.y = x_transformed(1);  // Column 0
                response->x_transformed.z = x_transformed(2);  // Column 
                RCLCPP_INFO(this->get_logger(), "Conversion from local variables to response: COMPLETED.");
                rclcpp::sleep_for(1s);
            }
            catch(...)
            {
                RCLCPP_ERROR(this->get_logger(), "Conversion from local variables to response: FAILED.");
                rclcpp::sleep_for(1s);
            }
        }

        void workerFunction() {                                                                 //LLM
            std::unique_lock<std::mutex> lock(mtx_);
            while (rclcpp::ok()) 
            {
                cv_.wait(lock, [this]() { return new_message_; });
                RCLCPP_INFO(this->get_logger(), 
                "Worker thread processing message: [ %f, %f, %f ]",
                last_msg_.magic_vector.x,
                last_msg_.magic_vector.y,
                last_msg_.magic_vector.z);
                new_message_ = false;  // Reset flag
            }
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