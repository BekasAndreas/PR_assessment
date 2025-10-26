#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>  // for Eigen::Isometry3d
#include <tf2_eigen/tf2_eigen.hpp>  // helper conversion from tf2 to Eigen
#include <rviz_visual_tools/rviz_visual_tools.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @class JointConfiguration
 * @brief ROS2 node that publishes joint states and visualizes TF transformations
 * 
 * This node performs the following main functions:
 * - Performs TF transformations and validation
 * - Visualizes coordinate frames in RViz using rviz_visual_tools
 * - Implements periodic motion for joint parameters
 */
class JointConfiguration : public rclcpp::Node
{
public:

    JointConfiguration() : rclcpp::Node("joint_configuration_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting JointConfiguration node...");
        
        // Initialize TF buffer and listener
        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);

        // Initialize joint state publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        RCLCPP_INFO(this->get_logger(), "Joint state publisher created on topic: joint_states");

        // Initialize RViz visual tools for 3D visualization
        visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
            "base_link", 
            "/custom_markers",
            this
        );

        // Configure RViz visual tools
        visual_tools_->loadMarkerPub();
        visual_tools_->enableBatchPublishing();
        visual_tools_->deleteAllMarkers();
        visual_tools_->trigger();
        RCLCPP_DEBUG(this->get_logger(), "RViz visual tools initialized and markers cleared");


        // Create main timer (33Hz ~ 30ms)
        timer_ = this->create_wall_timer(30ms, std::bind(&JointConfiguration::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Main timer created with 30ms period");

        // Initialize timing and motion parameters
        start_ = this->now().seconds();
        period_ = 10; // seconds for complete motion cycle

        // Initialize joint positions
        goal_pos = Eigen::VectorXd::Random(6)*2;    // Random goal positions
        current_pos = Eigen::VectorXd::Zero(6);     // Start from zero

        // Declare ROS parameters for all joints with initial values
        this->declare_parameter<double>("shoulder_pan_joint", current_pos[0]);
        this->declare_parameter<double>("shoulder_lift_joint", current_pos[1]);
        this->declare_parameter<double>("elbow_joint", current_pos[2]);
        this->declare_parameter<double>("wrist_1_joint", current_pos[3]);
        this->declare_parameter<double>("wrist_2_joint", current_pos[4]);
        this->declare_parameter<double>("wrist_3_joint", current_pos[5]);

        RCLCPP_INFO(this->get_logger(), "JointConfiguration node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Motion period: %.1f seconds", period_);
    }


private:
    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState msg;

    // TF2 components
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::unique_ptr<tf2_ros::Buffer> buffer_;

    // Visualization
    std::shared_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;

    // Motion control variables
    double start_;
    double period_;
    Eigen::VectorXd goal_pos;
    Eigen::VectorXd current_pos;



    /**
     * @brief Main timer callback executed every 30ms
     */
    void timerCallback()
    {
        try {
            publishJointState();
            listenToTF_Compare();
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Timer callback Failed.");
        }

    }


    /**
     * @brief Publishes joint state message and updates joint parameters
     */
    void publishJointState()
    {

        // Set message timestamp
        msg.header.stamp = this->now();  
        msg.header.frame_id = "base_link";
        
        // Set joint names
        msg.set__name({ {"shoulder_pan_joint"},
                        {"shoulder_lift_joint"},
                        {"elbow_joint"},
                        {"wrist_1_joint"},
                        {"wrist_2_joint"},
                        {"wrist_3_joint"},                     
                        });

        // Get current joint positions from parameters
        msg.set__position({ {this->get_parameter("shoulder_pan_joint").as_double()},
                            {this->get_parameter("shoulder_lift_joint").as_double()},
                            {this->get_parameter("elbow_joint").as_double()},
                            {this->get_parameter("wrist_1_joint").as_double()},
                            {this->get_parameter("wrist_2_joint").as_double()},
                            {this->get_parameter("wrist_3_joint").as_double()},
                        });


        // Update motion parameters
        periodicmotion();
                
        // Publish the joint state message
        publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Joint state published with %zu joints", msg.position.size());
    }

    
    /**
     * @brief Implements periodic motion for joint parameters using sine waves
     */
    void periodicmotion()
    {
        auto now = this->now().seconds();
        double elapsed_time = now - start_; 
        auto phase_ = (2*M_PI)/period_*elapsed_time;

        if (elapsed_time < period_)
        {
            this->set_parameter(rclcpp::Parameter("shoulder_pan_joint",   current_pos[0] + (goal_pos[0] - current_pos[0]) * sin(phase_) ));
            this->set_parameter(rclcpp::Parameter("shoulder_lift_joint",  current_pos[1] + (goal_pos[1] - current_pos[1]) * sin(phase_) ));
            this->set_parameter(rclcpp::Parameter("elbow_joint",          current_pos[2] + (goal_pos[2] - current_pos[2]) * sin(phase_) ));
            this->set_parameter(rclcpp::Parameter("wrist_1_joint",        current_pos[3] + (goal_pos[3] - current_pos[3]) * sin(phase_) ));
            this->set_parameter(rclcpp::Parameter("wrist_2_joint",        current_pos[4] + (goal_pos[4] - current_pos[4]) * sin(phase_) ));
            this->set_parameter(rclcpp::Parameter("wrist_3_joint",        current_pos[5] + (goal_pos[5] - current_pos[5]) * sin(phase_) ));
            RCLCPP_DEBUG(this->get_logger(), "Motion phase: %.3f, Elapsed time: %.1f/%.1f", phase_, elapsed_time, period_);
        }
        else
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Motion period completed. Joint positions are static.");        
        }   

    }

    /**
     * @brief Listens to TF transforms, validates chain consistency, and visualizes frames
     */
    void listenToTF_Compare()
    {
        try {

            auto Tf_world_gripper = buffer_->lookupTransform("base_link", "tool0",  tf2::TimePointZero);
            auto Tf_world_elbow = buffer_->lookupTransform("base_link", "forearm_link", tf2::TimePointZero);
            auto Tf_elbow_gripper = buffer_->lookupTransform("forearm_link", "tool0", tf2::TimePointZero);

            // Convert TF transforms to Eigen for mathematical operations
            Eigen::Isometry3d eig_world_gripper = tf2::transformToEigen(Tf_world_gripper.transform);
            Eigen::Isometry3d eig_world_elbow   = tf2::transformToEigen(Tf_world_elbow.transform);
            Eigen::Isometry3d eig_elbow_gripper = tf2::transformToEigen(Tf_elbow_gripper.transform);

            // Validate transform chain: world->gripper should equal world->elbow * elbow->gripper
            Eigen::Isometry3d eig_world_gripper_check = eig_world_elbow * eig_elbow_gripper;

            // Check transform consistency with tolerance
            double tol = 1e-6; 
            bool same_translation = (eig_world_gripper_check.translation() - eig_world_gripper.translation()).norm() < tol;
            bool same_rotation = (eig_world_gripper_check.rotation() - eig_world_gripper.rotation()).norm() < tol;  // Frobenius norm

            if (same_translation && same_rotation) {
            RCLCPP_INFO(this->get_logger(), "TF chain validation: PASSED - Transforms are consistent!");
            }
            else
            {
            RCLCPP_INFO(this->get_logger(), " chain validation: FAILED - Transform inconsistency detected!");
            RCLCPP_DEBUG(this->get_logger(), "Translation error: %.6f, Rotation error: %.6f",
                        (eig_world_gripper_check.translation() - eig_world_gripper.translation()).norm(),
                        (eig_world_gripper_check.rotation() - eig_world_gripper.rotation()).norm());            
        }

            // Create pose for gripper frame (Tf_world_gripper)
            geometry_msgs::msg::Pose gripper_pose;
            gripper_pose.position.x = Tf_world_gripper.transform.translation.x;
            gripper_pose.position.y = Tf_world_gripper.transform.translation.y;
            gripper_pose.position.z = Tf_world_gripper.transform.translation.z;
            gripper_pose.orientation = Tf_world_gripper.transform.rotation;

            // Create pose for elbow frame to display the text label
            geometry_msgs::msg::Pose elbow_pose;
            elbow_pose.position.x = Tf_elbow_gripper.transform.translation.x;
            elbow_pose.position.y = Tf_elbow_gripper.transform.translation.y;
            elbow_pose.position.z = Tf_elbow_gripper.transform.translation.z;
            elbow_pose.orientation = Tf_elbow_gripper.transform.rotation;

            // Update RViz visualization
            visual_tools_->deleteAllMarkers();
            visual_tools_->publishAxis(gripper_pose, 0.3, 0.05); 
            
            // Add text label for elbow-gripper transform
            geometry_msgs::msg::Pose text_pose = elbow_pose;
            text_pose.position.z += 0.1;  // Offset above the elbow frame
            visual_tools_->publishText(text_pose, "Tf_elbow_gripper", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
            
            visual_tools_->trigger();
            RCLCPP_DEBUG(this->get_logger(), "RViz markers updated");

        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Error in TF processing");
        return;
}


    }   


};





int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointConfiguration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}