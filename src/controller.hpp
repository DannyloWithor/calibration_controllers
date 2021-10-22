#ifndef DIFF_CONTROLLER_HPP_
#define DIFF_CONTROLLER_HPP_

#include <iostream>
#include <array>
#include <chrono>
#include <memory>
#include <math.h>
#include <geometry_msgs/msg/twist.hpp>

class Controller
{
    float Pv;                                       // Proportial term of the controller
    float Iv;                                       // Integrative term of the controller
    float Dv;                                       // Derivative term of the controller
    float Po;                                       // Proportial term of the controller
    float Io;                                       // Integrative term of the controller
    float Do;                                       // Derivative term of the controller
    float integrativeEuclideanError;
    float integrativeAngularError;
    std::array<double,3> setpoint;                 // Setpoint pose, in terms of [x y th] in the map frame
    std::array<double,3> robot_pose_;              // Current robot pose
    float distance;
    float angularDistance;
    float lastLinearError;
    float lastAngularError;

    public:

    /// \brief Defines the controller object, with default parameters
    Controller(float Pv, float Iv, float Dv, float Po, float Io, float Do);

    /// \brief Sets the setpoint for the control task
    void setSetpoint(std::array<double,3> setpoint);

    /// \brief Sets the robor current pose
    void setCurrentPose(double x, double y, double th);

    /// \brief Changes the controller default parameters
    void setControllerParameters(float Pv, float Iv, float Dv, float Po, float Io, float Do);
    
    /// \brief Resets setpoint information and integrative errors
    void reset();

    /// \brief Compare the current pose and the setpoint and provide a twist message
    geometry_msgs::msg::Twist getControlSignal();

    /// \brief Returns the euclidean error between the current pose and the setpoint
    float getEuclideanError();

    /// \brief Returns the angular error between the current pose and the setpoint
    float getAngularError();

    ///
    void updateDistance(double increment);

    ///
    void saturateIntegrators();
    
};

#endif //DIFF_CONTROLLER_HPP_
