//
// Created by Felix Holz on 01.09.18.
// Copyright (c) 2018 Felix Holz All rights reserved.
//

#ifndef ORB_SLAM2_HPOSE_H
#define ORB_SLAM2_HPOSE_H

#include <iostream>
#include <vector>
#include <iomanip>

#include<opencv2/core/core.hpp>

namespace ORB_SLAM2
{
    // Human readable position and rotation
    struct HPose
    {
    public:
        HPose();
        HPose(const cv::Mat& twc, const cv::Mat& rwc);

        // New human-readable getters
        cv::Vec3f GetTranslation() const;
        cv::Vec4f GetRotation() const;

        cv::Vec4f GetRotationQuaternion() const;
        cv::Vec3f GetRotationEuler(bool degrees = false) const;

        void SetPosition(cv::Vec3f pos);
        void SetRotation(cv::Vec4f quaternion);

        void SetPosition(cv::Mat twc);
        void SetRotation(cv::Mat rwc);

        inline cv::Vec3f GetPosition() const;

        bool empty();

    private:
        cv::Vec3f m_Position;    // Position (x, y, z)
        cv::Vec4f m_Rotation;    // Rotation (Quaternion)

        const double RADIANS_TO_DEGREES = 180.0/3.141592653589793238463;
        bool _empty;
    };

    std::ostream& operator<<(std::ostream& os, const HPose& pose);

}


#endif //ORB_SLAM2_HPOSE_H
