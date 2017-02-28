These are the launch files for the simulation

* classicRobotGazebo.launch
	* Calls empy_field.launch
	* Loads the 2016 robot model
	* Sets up the controllers and a GUI for the controllers
* control_beacon_localization.launch
	* Calls r2d2_world.launch
	* Passes the r2d2.xacro into the r2d2_world.launch file
	* Starts rviz, so the robot model might be displayed in it automatically
	* Sets up the controllers and a GUI for the controllers
	* Starts the navigator and beacon_localization from cr17
	* I think it was originally copied from control.launch
* control.launch
	* Calls r2d2_world.launch
	* Passes the r2d2.xacro into the r2d2_world.launch file
	* Starts rviz, so the robot model might be displayed in it automatically
	* Sets up the controllers and a GUI for the controllers
* display_r2d2.launch
	* Loads the r2d2.urdf file
	* Starts the state publishers
	* Launches rviz
	* Meant to be able to test the code that does stuff with running motors and testing code without a full simulator (I think)
* display_traditional.launch
	* This is meant to be similar to display_r2d2.launch, but with the commented out line and the double slash on the rviz xml tag, I do not know if it works
	* The traditional robot 2016 was removed from this repo since the model was bad, it may be benefical to remove this launch file as well
* empty_field.launch
	* Calls gazebo.launch, but with an empty RMC field as the world parameter
	* No robot is loaded
* gazebo.launch
	* launches gazebo
	* DO NOT LAUNCH DIRECTLY, nothing will happen and it will crash
* minibots_4_stage.launch
	* launches a Stage simulation of the field with four small robots
	* each small robot will also have a minibot_navigator node
	* includes the magic_odomFromStage_4 node
	* For minibot testing with 4 minitbots
* minibots_6_stage.launch
	* launches a Stage simulation of the field with six small robots
	* each small robot will also have a minibot_navigator node
	* includes the magic_odomFromStage_6 node
	* For minibot testing with 6 minitbots
* r2d2.launch
	* Calls r2d2_world.launch
	* Loads the r2d2.xacro model
	* Starts the r2d2 controllers and the associated GUIs
* r2d2_world.launch
	* Calls the gazebo_ros empty_world.launch file with the RMCField_gazebo.world file
	* Loads the r2d2.xacro model, but the lines that spawn it are commented
	* This can probably be removed if all the files that call it are changed to call empty_field.launch, since will launch the same things and won't load a robot that it doesn't use
* r3d1.launch
	* Loads r3dx.urdf and spawns it in gazebo as r3d1
	* Launches the controllers for r3d1
	* Starts a minibot_navigator in the r3d1 namesspace
	* Does not launch Gazebo, it expects that a different file launched it
	* Maybe depracated and replaced by r3dx.launch if nothing calls it
* r3d1_stage.launch
	* Launches stage with the mining_field.world world file
	* Is meant for stage with testing a single robot/minibot
* r3d2.launch
	* Loads r3dx.urdf and spawns it in gazebo as r3d2
	* Launches the controllers for r3d2
	* Starts a minibot_navigator in the r3d2 namesspace
	* Does not launch Gazebo, it expects that a different file launched it
	* Maybe depracated and replaced by r3dx.launch if nothing calls it
* r3dx.launch
	* Loads the model $(arg name).urdf //Where $(arg name) is the argument named name passed into it
	* Loads the r3dx control file and launches the r3dx controllers with the spanwer renamed to start with $(arg name)
	* Launches minibot_navigator in the namespace it is currently in
	* Is meant to be called by other launch files, for launching new minibots
	* There needs to be a model file with the same name as this launch file is given, since you can't change the name of the model an its internal contents.
* r3dx_1.launch
	* Calls r3dx_world.launch
	* Calls r3dx.launch in the r3d1 name space with a name of r3d1
	* Has calls to r3d1.launch and r3d2.launch commented out
* r3dx_2.launch
	* Calls r3dx_world.launch
	* Calls r3dx.launch in the r3d1 name space with a name of r3d1
	* Calls r3dx.launch in the r3d2 name space with a name of r3d2
	* Has calles to r3d1.launch and r3d2.launch commented out
* r3dx_4.launch
	* Calls r3dx_world.launch
	* Calls r3dx.launch in the r3d1 name space with a name of r3d1
	* Calls r3dx.launch in the r3d2 name space with a name of r3d2
	* Calls r3dx.launch in the r3d3 name space with a name of r3d3
	* Calls r3dx.launch in the r3d4 name space with a name of r3d4
	* Has calles to r3d1.launch and r3d2.launch commented out
* r3dx_world.launch
	* Launches gazebo with the r3dx.world file
	* Launches the magic_odomFromGazebo node
	* The r3dx.world file is an out dated verson on the RMCField
* r4d1.launch
	* Calls r2d2_world.launch
	* Spawns the r4dx.urdf model
	* Runs the controllers for it
	* Has commented out launching aries' navigator and beacon_localization
