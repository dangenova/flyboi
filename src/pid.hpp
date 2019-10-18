#pragma once

#include <ros/ros.h>
#include <cmath>

/* This is a PID Class For the Crazy Flie Created by Peng */
class PID
{
public:
    PID(
        float kp,
        float kd,
        float ki,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        const std::string& name)
        : m_kp(kp)
        , m_kd(kd)
        , m_ki(ki)
        , m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_integral(0)
        , m_previousError(0)
        , m_previous_LP_Diff(0)
        , m_previousTime(ros::Time::now())
    {
    }

    void run(float kp, float kd, float ki, float minOutput, float maxOutput, float integratorMin, float integratorMax)
    {
        m_kp = kp;
        m_kd = kd;
        m_ki = ki;
        m_minOutput = minOutput;
        m_maxOutput = maxOutput;
        m_integratorMin = integratorMin;
        m_integratorMax = integratorMax;
    }

    void reset()
    {
        m_integral = 0;
        m_previousError = 0;
        m_previous_LP_Diff = 0;
        m_previousTime = ros::Time::now();
    }

    void setIntegral(float integral)
    {
        m_integral = integral;
    }

    float ki() const
    {
        return m_ki;
    }

    float update(float value, float targetValue)
    {
        float ratio = 0.6;
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        // error = target - true
        float error = targetValue - value;
        m_integral += error * dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
        float p = m_kp * error;
        float d = 0;
        float m_difference = (error - m_previousError) / dt;
        if (dt > 0)
        {
            // A low-pass filter
            m_previous_LP_Diff = ratio*m_difference + (1-ratio)*m_previous_LP_Diff;
            d = m_kd * m_previous_LP_Diff;
        }
        float i = m_ki * m_integral;
        float output = p + d + i;
        m_previousError = error;
        m_previousTime = time;
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_previousError;
    float m_previous_LP_Diff;
    ros::Time m_previousTime;
};
