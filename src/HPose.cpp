//
// Created by Felix Holz on 01.09.18.
// Copyright (c) 2018 Felix Holz All rights reserved.
//

#include <include/HPose.h>

#include "Converter.h"
#include "HPose.h"


ORB_SLAM2::HPose::HPose()
    : m_Position(0, 0, 0), m_Rotation(0, 0, 0, 0), _empty(true)
{ }

ORB_SLAM2::HPose::HPose(const cv::Mat& twc, const cv::Mat& rwc)
    : m_Position(0, 0, 0), m_Rotation(0, 0, 0, 0), _empty(false)
{
    SetPosition(twc);
    SetRotation(rwc);
}

void ORB_SLAM2::HPose::SetPosition(cv::Mat twc)
{
    m_Position[0] = twc.at<float>(0);
    m_Position[1] = twc.at<float>(1);
    m_Position[2] = twc.at<float>(2);
    _empty = false;
}

void ORB_SLAM2::HPose::SetPosition(cv::Vec3f pos)
{
    m_Position = pos;
    _empty = false;
}

void ORB_SLAM2::HPose::SetRotation(cv::Mat rwc)
{
    std::vector<float> q = Converter::toQuaternion(rwc);

    m_Rotation[0] = q[0];
    m_Rotation[1] = q[1];
    m_Rotation[2] = q[2];
    m_Rotation[3] = q[3];
    _empty = false;
}

void ORB_SLAM2::HPose::SetRotation(cv::Vec4f quaternion)
{
    m_Rotation = quaternion;
    _empty = false;
}

bool ORB_SLAM2::HPose::empty()
{
  return(_empty);
}

cv::Vec3f ORB_SLAM2::HPose::GetTranslation() const
{
  return m_Position;
}

cv::Vec4f ORB_SLAM2::HPose::GetRotation() const
{
  cv::Vec4f _Rotation;
  _Rotation[0] = m_Rotation[1];
  _Rotation[1] = m_Rotation[2];
  _Rotation[2] = m_Rotation[3];
  _Rotation[3] = m_Rotation[0];

  return _Rotation;
}

cv::Vec4f ORB_SLAM2::HPose::GetRotationQuaternion() const
{
    return m_Rotation;
}

cv::Vec3f ORB_SLAM2::HPose::GetRotationEuler(bool degrees) const
{
    //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    double roll, pitch, yaw;

    // roll (x-axis rotation)
    double sinr = +2.0 * (m_Rotation[0] * m_Rotation[1] + m_Rotation[2] * m_Rotation[3]);
    double cosr = +1.0 - 2.0 * (m_Rotation[1] * m_Rotation[1] + m_Rotation[2] * m_Rotation[2]);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (m_Rotation[0] * m_Rotation[2] - m_Rotation[3] * m_Rotation[1]);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (m_Rotation[0] * m_Rotation[3] + m_Rotation[1] * m_Rotation[2]);
    double cosy = +1.0 - 2.0 * (m_Rotation[2] * m_Rotation[2] + m_Rotation[3] * m_Rotation[3]);
    yaw = atan2(siny, cosy);

    if(degrees)
    {
        return cv::Vec3f((float) (roll * RADIANS_TO_DEGREES), (float) (pitch * RADIANS_TO_DEGREES), (float) (yaw * RADIANS_TO_DEGREES));
    }
    else
    {
        return cv::Vec3f((float) roll, (float) pitch, (float) yaw);
    }
}

cv::Vec3f ORB_SLAM2::HPose::GetPosition() const
{
    return m_Position;
}

std::ostream& ORB_SLAM2::operator<<(std::ostream &os, const ORB_SLAM2::HPose &pose)
{
    cv::Vec3f euler = pose.GetRotationEuler(true);
    //os << std::fixed << std::setprecision(3) << "X: " << pose.GetPosition()[0] << ", Y: " << pose.GetPosition()[1] << ", Z: " << pose.GetPosition()[2] << std::endl << "RotX: " << euler[0] << ", RotY: " << euler[1] << ", RotZ: " << euler[2] << std::endl;
    os << std::fixed << std::setprecision(3) << pose.GetPosition()[0] << " " << pose.GetPosition()[1] << " " << pose.GetPosition()[2] << std::endl << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;
    return os;
}