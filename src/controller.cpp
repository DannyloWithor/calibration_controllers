#include "controller.hpp"
#define PI 3.1415


Controller::Controller(float Pv, float Iv, float Dv, float Po, float Io, float Do){
        // Initialize controller parameters
        this->Pv = Pv;
        this->Iv = Iv;
        this->Dv = Dv;
        this->Po = Po;
        this->Io = Io;
        this->Do = Do;
        this->integrativeEuclideanError = 0;
        this->integrativeAngularError = 0;

        // Initialize robot parameters
        this->robot_pose_[0]=0; 
        this->robot_pose_[1]=0;
        this->robot_pose_[2]=0;

        // Initialize robot parameters
        this->setpoint[0]=0; 
        this->setpoint[1]=0;
        this->setpoint[2]=0;

        // Float
        this->distance = 0.0;
        this->angularDistance = 0.0;
        this->lastLinearError = 0.0;
        this->lastAngularError = 0.0;
}

void Controller::setControllerParameters(float Pv, float Iv, float Dv, float Po, float Io, float Do)
{
    this->Pv = Pv;
    this->Iv = Iv;
    this->Dv = Dv;
    this->Po = Po;
    this->Io = Io;
    this->Do = Do;

    this->reset();
}

void Controller::setSetpoint(std::array<double,3> setpoint)
{
    this->setpoint[0] = setpoint[0];
    this->setpoint[1] = setpoint[1];

    //std::cout<<"The current setpoint is "<< this->setpoint[0] << " " << this->setpoint[1] << std::endl;
}

void Controller::setCurrentPose(double x, double y, double th)
{
    //this->distance += sqrt( (this->robot_pose_[0]-x) * (this->robot_pose_[0]-x)  + (this->robot_pose_[1]-y) * (this->robot_pose_[1]-y));
   
    double angularDiff = (th - this->robot_pose_[2]);
    // std::cout<<"Inside get angular error   "<< this->angularDistance << " " << th << std::endl;

    if(angularDiff > PI)
      angularDiff = angularDiff - 2*PI;
    if(angularDiff < -PI)
      angularDiff = angularDiff + 2*PI;

    this->angularDistance += angularDiff;
    
    this->robot_pose_[0] = x;
    this->robot_pose_[1] = y;
    this->robot_pose_[2] = th;
}

void Controller::updateDistance(double increment)
{
    this->distance += increment;
}

void Controller::reset()
{
    this->integrativeEuclideanError = 0;
    this->integrativeAngularError = 0;

    this->distance = 0.0;
    this->angularDistance = 0.0;

    this->setpoint[0] = 0.0;
    this->setpoint[1] = 0.0;
}

float Controller::getEuclideanError()
{
    //float x_diff_2 = (this->robot_pose_[0] - this->setpoint[0]) * (this->robot_pose_[0] - this->setpoint[0]);
    //float y_diff_2 = (this->robot_pose_[1] - this->setpoint[1]) * (this->robot_pose_[1] - this->setpoint[1]);
    //std::cout<<"Inside get euclidean error   "<< this->distance << " " << this->setpoint[0] << std::endl;

    return this->setpoint[0] - this->distance;
}

float Controller::getAngularError()
{
    //float x_diff_2 = (this->setpoint[0] - this->robot_pose_[0]);
    //float y_diff_2 = (this->setpoint[1] - this->robot_pose_[1]);
    //std::cout<<"Inside get angular error   "<< this->angularDistance << " " << this->setpoint[1] << std::endl;
    return this->setpoint[1] - this->angularDistance;
}

void Controller::saturateIntegrators()
{
    if(this->integrativeEuclideanError > 2.0)
        this->integrativeEuclideanError = 2.0;

    if(this->integrativeEuclideanError < -2.0)
        this->integrativeEuclideanError = -2.0;

    if(this->integrativeAngularError > 0.1)
        this->integrativeAngularError = 0.1;

    if(this->integrativeAngularError < -0.1)
        this->integrativeAngularError = -0.1;
}
geometry_msgs::msg::Twist Controller::getControlSignal()
{
    float currentEuclideanError = this->getEuclideanError();
    float currentAngularError = this->getAngularError();
    this->integrativeEuclideanError += currentEuclideanError;
    this->integrativeAngularError += currentAngularError;

    float linearDerivativeValue = currentEuclideanError - this->lastLinearError;
    float angularDerivativeValue = currentAngularError - this->lastAngularError;

    this->saturateIntegrators();

    geometry_msgs::msg::Twist msg;
    // D = 0.8;
    // P = 0.33
    // I = 0.04

    msg.linear.x = this->Pv*currentEuclideanError + this->Iv*this->integrativeEuclideanError + this->Dv*linearDerivativeValue;
    msg.angular.z = this->Po*currentAngularError + this->Io*this->integrativeAngularError + this->Dv*angularDerivativeValue;

    std::cout<<"[Controller] The current linear error is "<< currentEuclideanError << std::endl;
    std::cout<<"[Controller] The current integrative linear buffer is "<< this->integrativeEuclideanError << std::endl;
    std::cout<<"[Controller] The current angular error is "<< currentAngularError << std::endl;
    std::cout<<"[Controller] The current integrative angular buffer is "<< this->integrativeAngularError << std::endl;

    this->lastLinearError = currentEuclideanError;
    this->lastAngularError = currentAngularError;
    return  msg;
}

