#include <ros/ros.h>
#include <math.h>
#include <flyboi_ethz/CrazyfliePID.h>

#include "pid.hpp"

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:
    Controller(const ros::NodeHandle& n)
        : m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
    {
        ros::NodeHandle n1;
        m_PID_Service = n1.advertiseService("PID_Calculation", &Controller::PID_Run, this);
    }

private:
    bool PID_Run(flyboi_ethz::CrazyfliePID::Request& req, flyboi_ethz::CrazyfliePID::Response& res)
    {   
        if (req.trigger_integral_fix) {
            pidReset();
            m_pidZ.setIntegral(45000 / m_pidZ.ki());
            ROS_INFO("Reset PID Integral Values");
        }  
        float world_goal_position[3] = {
                    req.reference_x-req.position_x,
                    req.reference_y-req.position_y,
                    req.reference_z-req.position_z};

        float body_goal_position[3]={0,0,0};

        transform_world_to_body(world_goal_position, body_goal_position, 
                    req.position_roll, req.position_pitch, req.position_yaw);

        res.actuation_pitch  = m_pidX.update(0.0, body_goal_position[0]);
        res.actuation_roll   = m_pidY.update(0.0, body_goal_position[1]);
        res.actuation_thrust_pwm = m_pidZ.update(0.0, body_goal_position[2]);
        res.actuation_yawrate = m_pidYaw.update(req.position_yaw, req.reference_yaw);

        return true;
    }

    void pidReset(){
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    float transform_world_to_body(
        const float* source,
        float* target, 
        const float phi,
        const float theta,
        const float psi)
    {
        float sp    = sinf(phi);
        float cp    = cosf(phi);
        float st    = sinf(theta);
        float ct    = cosf(theta);
        float ss    = sinf(psi);
        float cs    = cosf(psi);
        target[0] = ct*cs*source[0] + ct*ss*source[1] -st*source[2];
        target[1] = (sp*st*cs-cp*ss)*source[0] + (sp*st*ss+cp*cs)*source[1] + sp*ct*source[2];
        target[2] = (cp*st*cs+sp*ss)*source[0] + (cp*st*ss-sp*cs)*source[1]+cp*ct*source[2];
    }


    

private:
    ros::ServiceServer m_PID_Service;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Crazyflie_PID_Server");
  ros::NodeHandle n("~");

  Controller controller(n);
  
  ros::spin();

  return 0;
}
