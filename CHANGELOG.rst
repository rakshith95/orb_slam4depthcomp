^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package orb_slam2_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'release' into 'dubnium-devel'
  removed utils library
  See merge request ros-overlays/orb_slam_2_ros!12
* removed utils library
* Contributors: Federico Nardi, federiconardi

2.1.1 (2020-07-31)
------------------
* Merge branch 'dev-2' into 'dubnium-devel'
  Dev 2
  See merge request ros-overlays/orb_slam_2_ros!11
* storing image only on keyframe added + better map update detection
* avoiding redundant iterations
* Contributors: Federico Nardi, federiconardi

2.1.0 (2020-07-29)
------------------
* Merge branch 'map-publishing' into 'dubnium-devel'
  Map publishing
  See merge request ros-overlays/orb_slam_2_ros!10
* improved performance by using shared_ptr to cv::Mat
* bug was in cv::Mat copying, now fixed with clone()
* addressing remarkable bug
* added changes for going multithreaded
* implemented publishOccupancyGrid method
* disabling pal flags + applying beautifier
* Merge branch 'localization-handling' into 'dubnium-devel'
  Localization handling
  See merge request ros-overlays/orb_slam_2_ros!9
* cosmetics
* added pose subscriber to initialpose topic
* code clean up
* adding changes to handle mislocalization events
* getting odometry from images callback
* Contributors: Federico Nardi, federiconardi

2.0.3 (2020-06-23)
------------------
* Merge branch 'stereo-slam' into 'dubnium-devel'
  Stereo slam
  See merge request ros-overlays/orb_slam_2_ros!8
* added depth subscriber to synchronizer message_filter
* handling dataset storing in both cases: RGBD + STEREO
* applying beautifier
* added depth image subscriber
* changes to config and launch files for using IR images
* Contributors: Federico Nardi, procopiostein

2.0.2 (2020-05-25)
------------------
* Merge branch 'dev' into 'dubnium-devel'
  Dev
  See merge request ros-overlays/orb_slam_2_ros!7
* code style fixes
* make the repo publishable
* cosmetics
* publishing identity when empty position
* publishing orb pose with covariance
* fixed jiggling due to TF
* publishing tf in the future
* avoiding to kill node in order to store keyframe trajectory
* store dataset in vector of pairs
* uupdated launches and configs for integration in pal_navigation_sm
* minor fixes in launch file
* fixes to launch and config files for ari simulation
* Merge branch 'depth-registered-orb' into 'dev'
  Depth registered orb
  See merge request ros-overlays/orb_slam_2_ros!6
* Update camera params for simulation as well as topics
* Register to topic depth_registered
* pushing dynreconf file
* fixes for ARI simulation
* fixed tf correction
* .gitignore fix
* better dataset storing
* trying to store data more efficiently
* Contributors: Federico Nardi, Sara Cooper, federiconardi, procopiostein

2.0.1 (2020-02-25)
------------------
* cosmetic
* set old behavior as default
* keeping old behavior and parametrizing tf frames
* publishing custom TFs
* Contributors: Federico Nardi, Procópio Stein

2.0.0 (2020-02-24)
------------------
* Merge branch 'build-fix' into 'master'
  Build fix
  See merge request ros-overlays/orb_slam_2_ros!3
* added codeowners
* fixed build
* Add melodic docker file
* Fix Camera.bf for D435
* Merge pull request #51 from CodesHub/master
  fix maps cannot be saved multiple times
* fix maps cannot be saved multiple times
  in System::SetCallStackSize(), 'rlimit.rlim_cur == kNewStackSize' should not be consider as an erro.
* Merge pull request #50 from jessisar/master
  Fix build with gcc9
* Corrected typedef so that map value_type and allocator are the same (KeyFrameAndPose)
* Add required package for the realsense to the docker file
* Add some remarks about Docker to the README
* Delete the melodic Dockerfile
* Fix the kinetic dockerfile, add a melodic dockerfile and move them in folders
* Add docker file
* Update README
  New line after the note for the map save-load services in the FAQ section
* Update README
  Make it more clear in the README that you need to source the catkin workspace for the custom services to become available
* Fix a seqfault which occured when loading large maps
* Make the stereo node compile
* Fix two segfaults; Save and load works now for large maps!
* Implement name specific map_saving
* Fix misspelling of variable
* Add link to the PR the save-load feature is based on
* Fix boldness of service names in the README
* Fix Headline 4 in the README
* Add documentation for the save and load feature
* Map save is now offered as a service instead as a parameter; Adjusted the launch files; Made the map save more verbose in case of a crash
* Fix the crash; Adjust all the launch file
* Implement the save load feature for all three SLAM types; DDreconfigure crashes after launch
* correction of launch file
* add save and load feature
* Give the StereoNode class owenership of the subscribers and sync
* Use the num_channels for the payload too
* Make the installation of dependencies more clear; Add a note for the publish_pose param; Add a table containing all cameras which are supported out of the box
* Add the cmake-build-debug folder to the .gitignore
* Merge pull request #18 from hoangthien94/pose_publisher
  Publish pose_stamped
* Add param to enable/disable publishing pose
* Add PoseStamped publisher
* Merge pull request #17 from hoangthien94/mynteye_s_camera
  Add support for MyntEye S camera
* Add support for MyntEye S camera
* Add explanation for the new min_observation param
* Enable the possibility to configurer the minimal number of observations a point must have to get into the ros point cloud
* Add an overview on how to use different cameras
* Fix variable naming
* Adjsut the individual depth thresholds for the RGBD cams
* Remove the camera name from the node name
* Merge pull request #15 from saoto28/dev
  Fix the coordinate transformation from the orb_slam frame to the ros frame.
* modify transformation matrix of Node::TransformFromMat()
* modify transformation matrix of Node::TransformFromMat()
* Only compile C++ files with the C++11 flag not C files - fixes the warning
* Due to issue #5 add the c++11 compiler option again
  This reverts commit 3c6c96e480f161ed3df85464ae45ad6d120739d2.
* Fixed the README file
* Adjusted the depth map threshold for the d435_rgbd
* New parameters for the D435 again, this time from the camera itself
* Now the new config files for the D435 are actually being used
* New calibration parameters for the D435
* Add config and launch files for the d435 camera; rename the files for the r200; adjust the readme
* Merge branch 'dynamic_reconfigure'
* Noted the possibility to use rqt_reconfigure and the three types of parameters
* Implement dynamic_reconfigure
* Moved dynamic parameters to dynamic_reconfigure
* Adjusted the CMakeList for dynamic_reconfigure; Updated the dynamic_reconfigure .cfg file with the first params from the orb_slam
* Fix the Thirdparty folder location
* Add the dynamic_reconfigure package as a dependency; Add a sample param.cfg file
* Fix the C++11 compiler warning
* Fix using an int variable as a bool
* Ignore the meta files from CLion
* Note that OpenCV is installed along with ROS
* Note that the package is now tested with melodic
* Add install rules to the CMakeLists and add the sensor_msgs to the package.xml
* Fix the link to the repo
* Make the timestamp in all published data the one from the current image; Move code to the node class to remove redundancy"
* Clean up the config files
* Fix links in the README
* Add more information about the stereo node and features to the README
* Merge with master
* Merge pull request #1 from plieningerweb/master
  add stereo node and example how to launch using recitfied stereo image
* Make the package descriptions (a little) more verbose
* Fix links to the license text
* add stereo node and example how to launch using recitfied stereo image
* Adjust the links of the readme
* Adjusted the Readme and license text
* Remove uneeded code
* Fix the rotation of the camera and the coordinates of the pointcloud
* Fix some bugs for the ros param set and get
* Fixed the foreward decleration
* Implement the reset_map and the min_num_of_kf_in_map parameters
* Add the binary vocab file to git
* Implement ros parameters to supply parameters to the node
* Removes uneeded code
* Add binary file support for a much more rapide startup and a smaller file size
* Replace usleep with the mor versatile and compatible std::this_thread::sleep_for
* Fix intendation
* Add sensor_msgs to the required packages in the CMakeList
* Make the Mono and RGBD node publish the MapPoints using the new function provided by the base class
* Make the Node base class able to publish the MapPoints as PointCloud2
* Renames the GetMapPoints for clarification
* Fix the bug where the drone rotates around the origin instead of on the spot
* Make the system able to get all available MapPoints
* Make the MonoNode use the refactored base class for less redundancy
* Make the rgbdnode use the refactored base class with less redundancy
* Add the image publisher and the orb_slam as members for the node base class for less redundancy
* Cleans up code
* Removes uneeded include
* Removed uneeded code and fixed the coordinate transform
* Clean up the config files
* Try to fix the error in the transformation from the orb-slam to the ros coordinate system
* Implements the RGBD node
* Make the SLAM publish the current camera pos as a tf;
* The Mono node now publishes the rendered image from the frameDrawer
* Deleted the old unused cmake file and adjusted the new one
* Made the launch file name more specific
* Addet config files for the intel realsense
* Deleted the unpacked vocab file from git
* Put everything in folders, deleted unused and adjusted the build files
* Initial commit
* Contributors: Andreas Plieninger, Brahim Boudamouz, CodesHub, Lennart Haller, Procópio Stein, Saoto Tsuchiya, Zach Carmichael, hoangthien94, procopiostein
