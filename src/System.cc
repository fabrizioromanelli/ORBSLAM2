/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <algorithm>
#include <chrono>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, bool is_save_map_, bool replayer_):mSensor(sensor), is_save_map(is_save_map_), replayer(replayer_), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),
        mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mCurrCameraPose()
{
    cout << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
        cout << "Monocular" << endl;
    else if (mSensor == STEREO)
        cout << "Stereo" << endl;
    else if (mSensor == RGBD)
        cout << "RGB-D" << endl;

    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode mapfilen = fsSettings["Map.mapfile"];
    bool bReuseMap = false;
    if (!mapfilen.empty())
    {
        mapfile = (string)mapfilen;
    }
    else if (is_save_map)
    {
        cerr << "Save map is on but no file has been specified in yaml configuration." << endl;
        cerr << "Set Map.mapfile: <filename> in the yaml file" << endl;
        exit(-1);
    }

    // Load FBoW Vocabulary
    cout << endl << "Loading FBoW Vocabulary... ";

    mpFBOWVocabulary = new fbow::Vocabulary();
    mpFBOWVocabulary->readFromFile(strVocFile);

    cout << "Vocabulary loaded!" << endl << endl;

    // Create KeyFrame Database
    // Create the Map

    if (!mapfile.empty() && LoadMap(mapfile))
    {
        bReuseMap = true;
        std::cout << "Loaded Map" << std::endl;
        is_save_map = false;
    }
    else
    {
        std::cout << "Map NOT loaded" << std::endl;
        mpKeyFrameDatabase = new KeyFrameDatabase(mpFBOWVocabulary);
        mpMap = new Map();
    }

    // Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    // Initialize the Tracking thread
    // (it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpFBOWVocabulary, mpFrameDrawer, mpMapDrawer, mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, bReuseMap);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR, strSettingsFile);

    // Initialize the Loop Closing thread and launch
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpFBOWVocabulary, mSensor != MONOCULAR, strSettingsFile);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    // Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, bReuseMap);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor != STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    // Fixes to cope with replays
    // Wait until LoopCloser has effectively finished and LocalMapper
    // has been released. This allows not to get new frames from the
    // tracker (it actually freezes the images grabbing).
    if (replayer) {
      while(!mpLoopCloser->isFinishedGBA())
      {
          std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }

      while(mpLocalMapper->isStopped())
      {
          std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

// Overload of the track RGBD function
HPose System::TrackIRD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat _Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    HPose camPose;
    // Conversion from ORB to WORLD right-handed reference frame
    if (!_Tcw.empty()) {
      cv::Mat Rwc = _Tcw.rowRange(0, 3).colRange(0, 3).t();
      cv::Mat Twc = -Rwc * _Tcw.rowRange(0, 3).col(3);

      Eigen::Matrix3f orMat;
      orMat(0,0) = Rwc.at<float>(0,0);
      orMat(0,1) = Rwc.at<float>(0,1);
      orMat(0,2) = Rwc.at<float>(0,2);
      orMat(1,0) = Rwc.at<float>(1,0);
      orMat(1,1) = Rwc.at<float>(1,1);
      orMat(1,2) = Rwc.at<float>(1,2);
      orMat(2,0) = Rwc.at<float>(2,0);
      orMat(2,1) = Rwc.at<float>(2,1);
      orMat(2,2) = Rwc.at<float>(2,2);
      Eigen::Quaternionf q_orb_tmp(orMat);
      Eigen::Quaternionf _q_orb(q_orb_tmp.w(), -q_orb_tmp.z(), -q_orb_tmp.x(), -q_orb_tmp.y());

      cv::Vec3f p_orb(Twc.at<float>(2), -Twc.at<float>(0), -Twc.at<float>(1));
      cv::Vec4f q_orb(_q_orb.w(), -_q_orb.x(), _q_orb.y(), _q_orb.z());

      camPose.SetPosition(p_orb);
      camPose.SetRotation(q_orb);
    }

    return camPose;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor != MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
  // Wait for a Global Bundle Adjustment to finish
  while (mpLoopCloser->isRunningGBA()) {
    this_thread::sleep_for(chrono::milliseconds(500));
  }

  {
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
  }
}

void System::Shutdown()
{
    std::cout << "Shutting down VSLAM system" << std::endl;
    mpLocalMapper->RequestFinish();
    std::cout << "Stopping local mapper" << std::endl;
    mpLoopCloser->RequestFinish();
    std::cout << "Stopping loop closer" << std::endl;
    if (mpViewer)
    {
        mpViewer->RequestFinish();
        std::cout << "Stopping mapviewer" << std::endl;
        while(!mpViewer->isFinished())
        {
            std::cerr << "Waiting for mapviewer to terminate" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "Mapviewer terminated" << std::endl;
    }

    // Wait until all thread have effectively stopped
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cerr << "Waiting for threads to terminate" << std::endl;
    }

    if (is_save_map)
    {
        std::cout << "Saving map..." << std::endl;
        SaveMap(mapfile);
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectory(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectory(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

// Computing covariance matrix for VSLAM pose as per
// Appendix F in "Asynchronous Multi-Sensor Fusion for 3D
// Mapping and Localization", P. Geneva, K. Eckenhoff, G. Huang
cv::Mat System::GetCurrentCovarianceMatrix(float fx, float fy, cv::Mat cameraPose, bool rotationInverse)
{
  // Computing Visual odometry covariance matrix
  if (this->GetTrackingState() == Tracking::OK)
  {
    vector<MapPoint*> trackedPoints = this->GetTrackedMapPoints();
    std::vector<cv::Mat> featPos;
    for (size_t i = 0; i < trackedPoints.size(); i++)
    {
      MapPoint* pMP = trackedPoints[i];
      if(pMP)
        featPos.push_back(pMP->GetWorldPos());
    }

    cv::Mat lambda = cv::Mat::zeros(2,2,CV_32F);
    cv::Mat factor = cv::Mat::zeros(6,6,CV_32F);
    lambda.at<float>(0,0) = 1.0/(fx*fx);
    lambda.at<float>(1,1) = 1.0/(fy*fy);
    cv::Mat Rcg = cv::Mat::zeros(3,3,CV_32F); // Rotation from Camera pose to Global frame
    Rcg.at<float>(0,0) = cameraPose.at<float>(0,0);
    Rcg.at<float>(0,1) = cameraPose.at<float>(0,1);
    Rcg.at<float>(0,2) = cameraPose.at<float>(0,2);
    Rcg.at<float>(1,0) = cameraPose.at<float>(1,0);
    Rcg.at<float>(1,1) = cameraPose.at<float>(1,1);
    Rcg.at<float>(1,2) = cameraPose.at<float>(1,2);
    Rcg.at<float>(2,0) = cameraPose.at<float>(2,0);
    Rcg.at<float>(2,1) = cameraPose.at<float>(2,1);
    Rcg.at<float>(2,2) = cameraPose.at<float>(2,2);
    
    if (rotationInverse)
      Rcg = Rcg.inv();

    for (size_t j = 0; j < featPos.size(); j++)
    {
      cv::Mat H1 = cv::Mat::zeros(2,3,CV_32F);
      cv::Mat H2 = cv::Mat::zeros(3,6,CV_32F);
      cv::Mat Pcf = cv::Mat::zeros(3,1,CV_32F); // Pose of feature in respect to Camera frame
      cv::Mat Pgc = cv::Mat::zeros(3,1,CV_32F);; // Pose of camera in respect to Global frame
      float a, b;
      Pgc.at<float>(0) = cameraPose.at<float>(0,3);
      Pgc.at<float>(1) = cameraPose.at<float>(1,3);
      Pgc.at<float>(2) = cameraPose.at<float>(2,3);

      Pcf = Rcg * (featPos[j] - Pgc);

      if (Pcf.at<float>(2) == 0.0f)
      {
        a = 1000000;
        b = 1000000000000;
      }
      else
      {
        a = 1/Pcf.at<float>(2);
        b = 1/(Pcf.at<float>(2)*Pcf.at<float>(2));
      }
      H1.at<float>(0,0) = a;
      H1.at<float>(0,2) = -Pcf.at<float>(0)*b;
      H1.at<float>(1,1) = a;
      H1.at<float>(1,2) = -Pcf.at<float>(1)*b;
      H2.at<float>(0,1) = -Pcf.at<float>(2);
      H2.at<float>(0,2) = Pcf.at<float>(1);
      H2.at<float>(0,3) = -Rcg.at<float>(0,0);
      H2.at<float>(0,4) = -Rcg.at<float>(0,1);
      H2.at<float>(0,5) = -Rcg.at<float>(0,2);
      H2.at<float>(1,0) = Pcf.at<float>(2);
      H2.at<float>(1,2) = -Pcf.at<float>(0);
      H2.at<float>(1,3) = -Rcg.at<float>(1,0);
      H2.at<float>(1,4) = -Rcg.at<float>(1,1);
      H2.at<float>(1,5) = -Rcg.at<float>(1,2);
      H2.at<float>(2,0) = -Pcf.at<float>(1);
      H2.at<float>(2,1) = Pcf.at<float>(0);
      H2.at<float>(2,3) = -Rcg.at<float>(2,0);
      H2.at<float>(2,4) = -Rcg.at<float>(2,1);
      H2.at<float>(2,5) = -Rcg.at<float>(2,2);

      cv::Mat tmp = H1*H2;
      factor = factor + tmp.t() * lambda.inv() * tmp;
    }
    return(factor.inv());
  }
  else
  {
    return(cv::Mat::eye(6,6,CV_32F));
  }
}

// Returns the currently stored map: each element is a 3D-point coordinates vector
std::vector<Eigen::Vector3f> System::GetMap(bool wait_gba)
{
  // If a Global Bundle Adjustment is running, abort this or wait for it
  while (mpLoopCloser->isRunningGBA()) {
    if (wait_gba) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      return std::vector<Eigen::Vector3f>();
    }
  }

  // Other threads must not update the map while this reads it
  unique_lock<mutex> mapLock(mpMap->mMutexMapUpdate);

  // Get all valid map points
  vector<MapPoint *> mapPoints = mpMap->GetAllMapPoints();
  mapPoints.erase(
    remove_if(
      mapPoints.begin(),
      mapPoints.end(),
      [] (MapPoint * p) { return p->isBad(); }),
      mapPoints.end());

  // Fill a matrix with their coordinates ([X Y Z]w = [Z -X -Y]o)
  std::vector<Eigen::Vector3f> pointsVec;
  pointsVec.reserve(mapPoints.size());
  for (auto p : mapPoints) {
    cv::Mat pPos = p->GetWorldPos();
    pointsVec.push_back(
      Eigen::Vector3f(
        pPos.at<float>(2),
        -pPos.at<float>(0),
        -pPos.at<float>(1)));
  }

  return pointsVec;
}

void System::SaveMap(const string &filename)
{
    std::ofstream out(filename, std::ios_base::binary);
    if (!out)
    {
        cerr << "Cannot Write to Mapfile: " << mapfile << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile: " << mapfile << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << " ...done" << std::endl;
    out.close();
}

bool System::LoadMap(const string &filename)
{
    std::ifstream in(filename, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapfile << ", Create a new one" << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << mapfile << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetFBOWvocabulary(mpFBOWVocabulary);
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {
        it->SetFBOWvocabulary(mpFBOWVocabulary);
        it->ComputeFboW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " ...done" << endl;
    in.close();
    return true;
}

HPose& System::GetCurrentCameraPose()
{
    return mCurrCameraPose;
}

} //namespace ORB_SLAM
