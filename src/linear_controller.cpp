#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <string>
#include "localization_interfaces/msg/control_msgs.hpp"


using namespace std::chrono_literals;

#define ROS_NODE_NAME "linear_controller"
#define PI 3.1415

class LinearController : public rclcpp::Node
{
  public:
    LinearController()
    : Node(ROS_NODE_NAME)
    { 
      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

      // Initialize setpoint
      this->setpoint[0] = 0;
      this->setpoint[1] = 0;
      this->setpoint[2] = 0;

      // Initialize pose
      this->robot_pose[0] = 0;
      this->robot_pose[1] = 0;
      this->robot_pose[2] = 0;

      // Advertise velocity commands
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("scooby/cmd_vel", default_qos);   

      // Advertise errors
      linear_error_pub = this->create_publisher<std_msgs::msg::Float32>("scooby/linear_error", default_qos);   
      angular_error_pub = this->create_publisher<std_msgs::msg::Float32>("scooby/angular_error", default_qos);   

      // Retrieve odom messages
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "scooby/odom_diff_drive",
        sensor_qos,
        std::bind(&LinearController::odom_callback, this, std::placeholders::_1));

      // Retrieve odom messages
      control_sub_ = this->create_subscription<localization_interfaces::msg::ControlMsgs>(
        "control_message",
        sensor_qos,
        std::bind(&LinearController::control_callback, this, std::placeholders::_1));

      // Retrieve joint messages
      joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
       "scooby/joint_states",
        sensor_qos,
        std::bind(&LinearController::joint_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
      1000ms, std::bind(&LinearController::run, this));   
    }

  private:
    /// \brief Velocity command publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    /// \brief Velocity command publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr linear_error_pub;

    /// \brief Velocity command publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_error_pub;

    /// \brief Odometry subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    /// \brief Joint state subscriber
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    /// \brief Control msgs subscriber
    rclcpp::Subscription<localization_interfaces::msg::ControlMsgs>::SharedPtr control_sub_;

    /// \brief Scale linear velocity, chosen by trial and error
    double linear_k_ = 0.02;

    /// \brief Scale angular velocity, chosen by trial and error
    double angular_k_ = 0.08;

    /// \brief Define an error acceptance threshold
    double max_linear_error = 0.2;

    /// \brief Define an error acceptance threshold
    double max_angular_error = 0.05;

    /// \brief Define an error acceptance threshold
    double last_linear_error = 0.0;

    /// \brief Define an error acceptance threshold
    double last_angular_error = 0.0;

    /// \brief Define the robot wheels radius
    double radius = 0.1;

    /// \brief Define an error acceptance threshold
    std::string state = "Running";

    /// \brief Setpoint for the controller task
    std::array<double,3> setpoint;

    /// \brief Setpoint for the controller task
    std::array<double,3> robot_pose;

    /// \brief Defines the controller object
    Controller controller = Controller(0.1, 0.1, 0.1, 0.1);

    /// \brief Defines the timer object
    rclcpp::TimerBase::SharedPtr timer_;

    /// \brief Last wheel angular positions
    double last_wheel_positions[2] = {0, 0};

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    //static rclcpp::Time last_time = odom_msg->header.stamp;

    // Updates the robot pose provided by odometry
    this->setRobotPose(odom_msg->pose.pose);
    
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
  {
    double wheel_l = joint_msg->position[0] - this->last_wheel_positions[0];
    double wheel_r = joint_msg->position[1] - this->last_wheel_positions[1];
    double ds = (wheel_l + wheel_r)*this->radius/2.0;
    this->controller.updateDistance(ds);

    this->last_wheel_positions[0] = joint_msg->position[0];
    this->last_wheel_positions[1] = joint_msg->position[1];
  }

  void control_callback(const localization_interfaces::msg::ControlMsgs::SharedPtr control_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Control message received: Type= %03d,  P= %03f, I= %03f, Setpoint= %03f ", 	
            (int) control_msg->type,
            (float) control_msg->p, (float) control_msg->i, (float) control_msg->setpoint);

    // Clean integrative errors buffer
    this->state = "Waiting";
    this->controller.reset();
    this->brake();

    float deg2rad = PI/180;

    // Check command type
    if(control_msg->type == 0)
    {
      this->state = "Linear";
      this->setpoint[0] = control_msg->setpoint;
      this->setpoint[1] = 0.0;
      this->controller.setControllerParameters(control_msg->p, control_msg->i, 0.0, 0.0);

    } else if (control_msg->type == 1)
    {
      this->state = "Angular";
      this->setpoint[0] = 0.0;
      this->setpoint[1] = control_msg->setpoint*deg2rad;
      this->controller.setControllerParameters(0.0, 0.0, control_msg->p, control_msg->i);

    } else if (control_msg->type == 2)
    {
        this->brake();
    }

    // Check for setpoint changes
    controller.setSetpoint(this->setpoint);
  }

  void setRobotPose(geometry_msgs::msg::Pose pose)
  {
    // ------- Retrieve orientation in euler format
    tf2::Quaternion q(pose.orientation.x,
                      pose.orientation.y,
                      pose.orientation.z,
                      pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Update controller pose
    this->controller.setCurrentPose(pose.position.x, pose.position.y, yaw);
    // Initialize pose
      this->robot_pose[0] = pose.position.x;
      this->robot_pose[1] = pose.position.y;
      this->robot_pose[2] = yaw;
  }

  void brake()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;

    publishCommand(msg);

    //RCLCPP_INFO(this->get_logger(), "Braking the vehicle!");	
  }

  void publishCommand(geometry_msgs::msg::Twist msg)
  {
    std::cout<<"[Controller] The vel cmd is "<< msg.linear.x << std::endl;
    std::cout<<"[Controller] The vel ang cmd is "<< msg.angular.z << std::endl << std::endl;
    cmd_pub_->publish(msg);
  }

  int run()
  {
    geometry_msgs::msg::Twist msg;

    if(this->state == "Waiting")
      {
        RCLCPP_INFO(this->get_logger(), "Emergency button pushed!");
        this->brake();
        return 0;
      }

    if(this->state == "Linear")
      {
        double currentEuclideanError = controller.getEuclideanError();
        RCLCPP_INFO(this->get_logger(), "Linear error:  %05f", currentEuclideanError);

        // Publish current error
        std_msgs::msg::Float32 error_msg;
        error_msg.data = currentEuclideanError;
        linear_error_pub->publish(error_msg);   
        
        // Retrieve command msg and publish
        msg = controller.getControlSignal();
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;
        publishCommand(msg);
        return 0;
      }

    if(this->state == "Angular")
      {
        double currentAngularError = controller.getAngularError();
        RCLCPP_INFO(this->get_logger(), "Angular error:  %05f", currentAngularError);    

        // Publish current error
        //angular_error_pub->publish(currentAngularError);       

        // Retrieve command msg and publish
        msg = controller.getControlSignal();
        msg.linear.x = 0.0;
        publishCommand(msg);

        return 0;
      }

    this->brake();  
    return 0;
  }
};

/**
 * Node for utilities necessary to evaluate scooby localization performance
 */
int main(int argc, char **argv)
{
	
  // initialize ROS node
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<LinearController>());
  rclcpp::shutdown();

  return 0;
}
