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
#include <iomanip>
#include <cmath>
#include <boost/filesystem.hpp>

float REPROJ_ERR_THRESH=5;
namespace ORB_SLAM2
{

System::System(const string strVocFile, const string strSettingsFile, const eSensor sensor,
               const std::string & map_file, bool load_map): // map serialization addition
               mSensor(sensor), mbReset(false),mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false),
               map_file(map_file), load_map(load_map)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary." << endl;

    mpVocabulary = new ORBVocabulary();

    //try to load from the binary file
    bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile+".bin");

    if(!bVocLoad)
    {
        cerr << "Cannot find binary file for vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile+".bin" << endl;
        cerr << "Trying to open the text file. This could take a while..." << endl;
        bool bVocLoad2 = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad2)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << strVocFile << endl;
            exit(-1);
        }
        cerr << "Saving the vocabulary to binary for the next time to " << strVocFile+".bin" << endl;
        mpVocabulary->saveToBinFile(strVocFile+".bin");
    }

    cout << "Vocabulary loaded!" << endl << endl;

    // begin map serialization addition
    // load serialized map
    if (load_map && LoadMap(map_file)) {
        std::cout << "Using loaded map with " << mpMap->MapPointsInMap() << " points\n" << std::endl;
    }
    else {
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        //Create the Map
        mpMap = new Map();
    }
    // end map serialization addition

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    currently_localizing_only_ = false;
}

void System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
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

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    current_position_ = Tcw;
}

void System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
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

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    current_position_ = Tcw;
}

void System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
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

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    current_position_ = Tcw;
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
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
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

    if(mSensor==RGBD)
    {
      ofstream out;
      out.open("dataset.txt");
      out << std::fixed;

      const std::vector<std::pair<double, std::string>> &dataset = mpTracker->dataset();
      for (size_t i = 0; i < dataset.size(); ++i)
      {
        out << dataset[i].first << " " << dataset[i].second << std::endl;
      }
      out.close();
      cout << endl << "dataset saved!" << endl;
    }
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

void System::SetMinimumKeyFrames (int min_num_kf) {
  mpTracker->SetMinimumKeyFrames(min_num_kf);
}

cv::Mat System::GetCurrentPosition () {
  return current_position_;
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

cv::Mat System::DrawCurrentFrame () {
  return mpFrameDrawer->DrawFrame();
}

std::vector<MapPoint*> System::GetAllMapPoints() {
  return mpMap->GetAllMapPoints();
}


bool System::SetCallStackSize (const rlim_t kNewStackSize) {
    struct rlimit rlimit;
    int operation_result;

    operation_result = getrlimit(RLIMIT_STACK, &rlimit);
    if (operation_result != 0) {
        std::cerr << "Error getting the call stack struct" << std::endl;
        return false;
    }

    if (kNewStackSize > rlimit.rlim_max) {
        std::cerr << "Requested call stack size too large" << std::endl;
        return false;
    }

    if (rlimit.rlim_cur <= kNewStackSize) {
        rlimit.rlim_cur = kNewStackSize;
        operation_result = setrlimit(RLIMIT_STACK, &rlimit);
        if (operation_result != 0) {
            std::cerr << "Setrlimit returned result: " << operation_result << std::endl;
            return false;
        }
        return true;
    }
    return false;
}


rlim_t System::GetCurrentCallStackSize () {
    struct rlimit rlimit;
    int operation_result;

    operation_result = getrlimit(RLIMIT_STACK, &rlimit);
    if (operation_result != 0) {
        std::cerr << "Error getting the call stack struct" << std::endl;
        return 16 * 1024L * 1024L; //default
    }

    return rlimit.rlim_cur;
}


void System::ActivateLocalizationMode()
{
    currently_localizing_only_ = true;
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    currently_localizing_only_ = false;
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::EnableLocalizationOnly (bool localize_only) {
  if (localize_only != currently_localizing_only_) {
    currently_localizing_only_ = localize_only;
    if (localize_only) {
      ActivateLocalizationMode();
    } else {
      DeactivateLocalizationMode();
    }
  }

  std::cout << "Enable localization only: " << (localize_only?"true":"false") << std::endl;
}


// map serialization addition
bool System::SaveMap(const string &filename) 
{
    unique_lock<mutex>MapPointGlobal(MapPoint::mGlobalMutex);
    std::ofstream out(filename, std::ios_base::binary);
    if (!out) {
        std::cerr << "cannot write to map file: " << map_file << std::endl;
        return false;
    }

    const rlim_t kNewStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
    const rlim_t kDefaultCallStackSize = GetCurrentCallStackSize();
    if (!SetCallStackSize(kNewStackSize)) {
        std::cerr << "Error changing the call stack size; Aborting" << std::endl;
        return false;
    }

    try {
        std::cout << "saving map file: " << map_file << std::flush;
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        oa << mpMap;
        oa << mpKeyFrameDatabase;
        std::cout << " ... done" << std::endl;
        out.close();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        SetCallStackSize(kDefaultCallStackSize);
        return false;
    } catch (...) {
        std::cerr << "Unknown exeption" << std::endl;
        SetCallStackSize(kDefaultCallStackSize);
        return false;
    }

    SetCallStackSize(kDefaultCallStackSize);

    std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    std::vector<MapPoint*> mps = mpMap->GetAllMapPoints();
    std::vector< std::vector<float> > kf_depths(vpKFs.size());
    std::vector<std::vector<cv::Point2f> > kf_2Dkeypoints(vpKFs.size());
    cv::Mat intrinsics = cv::Mat::zeros(3,4,CV_32F);

    for (size_t i=0; i<mps.size(); i++)
    {
        cv::Mat worldpos = mps[i]->GetWorldPos();
        // std::vector<float> worldPosvec {worldpos.at<float>(0,0), worldpos.at<float>(1,0), worldpos.at<float>(2,0)  };
        
        for(size_t j=0; j<vpKFs.size(); j++)
        {
            KeyFrame* kf = vpKFs[j];
            MapPoint* mappoint = mps[i];
            if(!mappoint->IsInKeyFrame(kf))
                continue;

            // const std::vector<float> depths = kf-> mvDepth;
            const std::vector<cv::KeyPoint> twoD_keypoints = kf->mvKeysUn;

            const float fx = kf->fx;
            const float fy = kf->fy;
            const float cx = kf->cx;
            const float cy = kf->cy;

            intrinsics.at<float>(0,0) = fx;
            intrinsics.at<float>(1,1) = fy;
            intrinsics.at<float>(0,2) = cx;
            intrinsics.at<float>(1,2) = cy;
            intrinsics.at<float>(2,2) = 1;

            int ind = mappoint->GetIndexInKeyFrame(kf);
            cv::KeyPoint twoD_keypoint = twoD_keypoints[ind];

            cv::Mat keyframePose_cameraFrame = kf->GetPose();
            // Reproject and check accuracy
            cv::Mat to_append = cv::Mat::ones(1,1,CV_32F);
            if (worldpos.rows<4)
                worldpos.push_back(to_append);
            cv::Mat pt_cam = keyframePose_cameraFrame*worldpos;
            cv::Mat reproj_point = intrinsics*pt_cam;
            cv::Point2f pt = twoD_keypoint.pt;
            float reproj_x = reproj_point.at<float>(0,0)/reproj_point.at<float>(2,0);
            float reproj_y = reproj_point.at<float>(1,0)/reproj_point.at<float>(2,0);

            float reproj_err = sqrt((reproj_x - pt.x)*(reproj_x - pt.x) + (reproj_y - pt.y)*(reproj_y - pt.y));
            if (reproj_err>REPROJ_ERR_THRESH)
                continue;

            // Otherwise continue to saving keyframe, sparse depth, pose (camera frame) data 
            float depth = pt_cam.at<float>(2,0)/pt_cam.at<float>(3,0);
            kf_depths[j].push_back(depth);
            kf_2Dkeypoints[j].push_back(pt);            
        }
    }   

    // Save  rgb image, depth image, pose
    std::string save_path = "/home/rakshith/orb_slam_data";
        
    if(!boost::filesystem::is_directory( save_path ))
        boost::filesystem::create_directory(save_path);
    if(!boost::filesystem::is_directory( save_path+"/images" ))
        boost::filesystem::create_directory(save_path+"/images");
    if(!boost::filesystem::is_directory( save_path+"/poses" ))
        boost::filesystem::create_directory(save_path+"/poses");
    if(!boost::filesystem::is_directory( save_path+"/depths" ))
        boost::filesystem::create_directory(save_path+"/depths");
    if(!boost::filesystem::is_directory( save_path+"/raw_depths" ))
        boost::filesystem::create_directory(save_path+"/raw_depths");
    if(!boost::filesystem::is_directory( save_path+"/validity_maps" ))
        boost::filesystem::create_directory(save_path+"/validity_maps");

    // Save intrinsics
    cv::FileStorage fs(save_path+"/intrinsics.yml", cv::FileStorage::WRITE);
    fs << "camera_intrinsics" << intrinsics;
    fs.release();


    std::string sparse_depth_file, raw_depth_file, image_file, pose_file, validity_map_file;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        sparse_depth_file = save_path;
        raw_depth_file = save_path;
        image_file = save_path;
        pose_file = save_path;
        validity_map_file = save_path;
        
        // Save color image
        cv::Mat img = vpKFs[i]->rgb_image;
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR); //imwrite default is bgr
        image_file.append("/images/keyframeP.png");
        size_t l = image_file.length();
        image_file.replace(l-5,1,std::to_string(i));
        cv::imwrite(image_file, img);

        // Save raw depth Image
        cv::Mat raw_depth_img = (vpKFs[i]->raw_depth_image);  
        // cv::cvtColor(img, img, cv::COLOR_RGB2BGR); //imwrite default is bgr
        raw_depth_file.append("/raw_depths/rawDepthP.yml");
        l = raw_depth_file.length();
        raw_depth_file.replace(l-5,1,std::to_string(i));
        cv::FileStorage fsd(raw_depth_file, cv::FileStorage::WRITE);
        fsd << "Depth" <<raw_depth_img;
        fsd.release();
        // cv::imwrite(raw_depth_file, raw_depth_img);

        // Save sparse depth Image
        cv::Mat depth_img = cv::Mat::zeros(img.rows, img.cols, CV_32F);
        std::vector<cv::Point2f> twoD_pts = kf_2Dkeypoints[i]; 
        std::vector<float> depths = kf_depths[i];

        for(size_t j=0; j<twoD_pts.size(); j++)
        {
            cv::Point2f pt = twoD_pts[j];
            depth_img.at<float>(pt.y, pt.x) = depths[j]; 
        }

        sparse_depth_file.append("/depths/depthP.yml");
        l = sparse_depth_file.length();
        sparse_depth_file.replace(l-5,1,std::to_string(i));
        cv::FileStorage fssd(sparse_depth_file, cv::FileStorage::WRITE);
        fssd << "Depth" <<depth_img;
        fssd.release();
        // cv::imwrite(sparse_depth_file, depth_img);

        // Validity Map
        cv::Mat validity_map = cv::Mat::zeros(depth_img.rows, depth_img.cols, CV_8U);
        for (size_t i=0;i<depth_img.rows;i++)
            for(size_t j=0;j<depth_img.cols;j++)
            {
                if (depth_img.at<float>(i,j) > 0)
                    validity_map.at<uchar>(i,j)=1;
            }
        validity_map_file.append("/validity_maps/validity_mapP.png");
        l = validity_map_file.length();
        validity_map_file.replace(l-5,1,std::to_string(i));
        cv::imwrite(validity_map_file, validity_map);

        // Save pose in camera frame
        cv::Mat pose_camFrame = vpKFs[i]->GetPose();
        pose_file.append("/poses/poseP.yml");
        l = pose_file.length();
        pose_file.replace(l-5,1,std::to_string(i));

        cv::FileStorage fs(pose_file, cv::FileStorage::WRITE);
        fs << "camera_pose" <<pose_camFrame;
        fs.release();
    }

    return true;
}

bool System::LoadMap(const string &filename) {
    
    unique_lock<mutex>MapPointGlobal(MapPoint::mGlobalMutex);
    std::ifstream in(filename, std::ios_base::binary);
    if (!in) {
        cerr << "Cannot open map file: " << map_file << " , you need create it first!" << std::endl;
        return false;
    }

    const rlim_t kNewStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
    const rlim_t kDefaultCallStackSize = GetCurrentCallStackSize();
    if (!SetCallStackSize(kNewStackSize)) {
        std::cerr << "Error changing the call stack size; Aborting" << std::endl;
        return false;
    }

    std::cout << "Loading map file: " << map_file << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    std::cout << " ... done" << std::endl;

    std::cout << "Map reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {

        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        
        if (it->mnFrameId > mnFrameId) {
            mnFrameId = it->mnFrameId;
        }
    }

    Frame::nNextId = mnFrameId;
    
    std::cout << " ... done" << std::endl;
    in.close();

    SetCallStackSize(kDefaultCallStackSize);
    
    return true;
}

} //namespace ORB_SLAM
