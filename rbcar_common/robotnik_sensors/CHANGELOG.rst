^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotnik_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.2 (2018-05-14)
------------------
* solved depreciation warnings
* merging with kinetic-multidevel
* Adding dependencies and updating maintainers list
* updating imu_plugin: topic and link names
* corrected collision box d435
* intel_d435 updated to run in gazebo
* added intel realsense d435
* adding frameid to imu_hector_plugin
* collision model of sick s300 corrected
* merging with kinetic multi that removes namespaces in the frames
* hokuyo ust20lx. fixing frame name to avoid namespace in it
* hokuyo ust10lx. changing links relation
* changing collision geometry for the orbbec
* adding / before any frame name
* prefix_topic param set to default values
* axis_m5013: changed name to prefix
* setting hokuyos laser_link z to 0.0 from the base_link
* renamig prefix_scan with prefix_topic for all the 2d lasers
* adding prefix topic for the orbbec astra
* adding prefix_scan to rplidars
* adding prefix_scan to all 2d lasers
* prepended 'hardware_interface/' before 'PositionJointInterface' in axis ptz urdf
* changed wrong name using prefix
* orbbec_astra: removing the suffix link from frame links
* laser hokuyo_urg04lx updated
* camera name changed
* hokuyo_urg04 model updated
* camera name changed
* added ust20 laser sensor
* robotnik_sensors: adding sensors from other branches
* added link position to hokuyo lasers
* links of xacro files renamed
* prefix added to imu hector urdf
* udf updated with prefix
* updated urdf to multirobot
* Merge branch 'indigo-devel' into kinetic-devel
* sensors updated to multirobot
* added urdf.xacro of sick_tim551
* added urdf.xacro of rplidar_a2
* added meshes of sick_tim551
* added meshes rplidar_a2
* adding prefix_topic and prefix_frame for multirobot
* adding prefix_frame and prefix_topic
* robotnik_sensors: removed _laser from rplidar_a2 frame
* robotnik_sensors: adding pointgrey_bumblebee
* updated model of axis camera
* added sick_tim551 to all_sensors
* added sick_tim551
* Adding RPLIDAR A2 model
* axis_m5013: changing joint names and new position adjustments
* adding param to ignore de tf_prefix of the laser plugin (needs modified version of the plugin
* added / to gazebo plugin frame name


1.1.1 (2016-12-16)
------------------
* Merge indigo to kinetic
* 1.0.5
* updated changelog
* 1.0.4
* update changelog
* MOD: Fixed problem with rotations
* --amend
* standarized all sensors: frames, joints, topics and params
* reduced rate drift and associated gaussian noise
* 1.0.3
* updated changelog
* Contributors: Jose Rapado, Marc Bosch-Jorge, carlos3dx, rguzman1

1.1.0 (2016-09-01 08:19:07 +0200)
---------------------------------
* updated changelog
* Contributors: carlos3dx

1.0.5 (2016-12-16 10:29:20 +0100)
---------------------------------
* updated changelog
* 1.0.4
* update changelog
* MOD: Fixed problem with rotations
* --amend
* standarized all sensors: frames, joints, topics and params
* reduced rate drift and associated gaussian noise
* Contributors: Jose Rapado, Marc Bosch-Jorge, carlos3dx, rguzman1

1.0.3 (2016-09-01 08:12:30 +0200)
---------------------------------
* updated changelog
* modified xmls:xacro
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/robotnik_sensors into indigo-devel
* Modified .xacro files
* corrected name of orientation parameters
* updated gps and imu_hector parameters
* resolved conflict
* added ueye camera
* Added rplidar to all_sensors
* Contributors: Marc Bosch-Jorge, carlos3dx, summit

1.0.2 (2016-07-12 07:30:38 +0200)
---------------------------------
* updated changelog
* Setting TIM571 params
* Added Sick Tim571 sensor
* New collision model for s3000
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/robotnik_sensors into indigo-devel
* adding the rplidar sensor
* sick_s300: changed collision model
* kinectv2: added model with no base and corrected bounding box of collision
* Merge remote-tracking branch 'origin/indigo-devel' into indigo-devel
* asus_xtrion_pro: corrected typo
* orbbec_astra: now calls the correct gazebo sensor
* Contributors: Jose Rapado, Marc Bosch-Jorge, RomanRobotnik, carlos3dx

1.0.1 (2016-06-27 09:13:11 +0200)
---------------------------------
* Adding CHANGELOG
* Setting build & run dependencies
* adding .gitignore
* Removed author
* Removing run dependencies and adding mantainers
* corrected angular resolution
* updated s300 standard sensor params
* added sick s300
* changed scale to 1 as the meshes were updated to scale 1
* added imu_hector_plugin.urdf.xacro, that seems to work better in sim
* reduced gps drift for rtk tests
* changed scale in asus_xtion_pro.urdf.xacro for new dae
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/robotnik_sensors into indigo-devel
* updated asus_xtion_pro_live.dae
* fixing 'scale' variable name. It was colliding between asus_xtion and orbec_astra sensors
* changed cam_link to have a reference frame for mounting in robot, not very useful at al...
* orbbec_astra frames updated to be compatible with gazebo plugin
* corrected scale and positions of optical frames
* added orbbec to all_sensors.urdf.xacro
* minor change names
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/robotnik_sensors into indigo-devel
  locally added orbbec astra sensor
* added orbecc_astra sensor
* minor change in collision box
* axis sensor: corrected pan tilt joints positions
* Added optical frame and modified parameters to axis_m5013.urdf.xacro
* axis_m5013 collision pos corrected
* changed collision box of ust10
* Merging master with indigo-devel to add the Sick S3000 laser
* Sicks3000
* malformed stl error rviz corrected by meshlab edit
* robotnik_sensors: adding min and max angle params to hokuyo sensors
* robotnik_sensors: removed collision model of the utm30lx sensor (it was colliding with itself)
* Adding new sensor SICKS3000
* corrected mesh file link
* renamed file, new sensor in all_sensors.urdf.xacro
* added hokuyo_ust_10lx model
* added latest stl of kinectv2. Note that dae was rotated 270ºZ from stl to be compliant with rest of rgbd devices tf
* added kinectv2 urdf (to be improved)
* added kinectv2
* Merge branch 'indigo-devel' of https://github.com/RobotnikAutomation/robotnik_sensors into indigo-devel
* added pointCloudCutoffMax param
* Change reference coordinates and topic name
* Setting hokuyo3d.dae path correctly
* compliant with new tag hardwareInterface requirement in joint
* removed dependancy from rbcar, modified sensor links and samples
* added gps_with_mast
* First commit. Compiles in indigo
* Initial commit
* Contributors: Dani Carbonell, ElenaFG, Jorge Arino, Marc Bosch-Jorge, RomanRobotnik, carlos3dx, mcantero, rguzman
