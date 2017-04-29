These are the launch files for the simulation

* control_teleop.launch
	* Calls RMC_field.launch
	* Sets up the controllers and a GUI for the controllers
* control_autonomous.launch
	* Calls RMC_field.launch
	* Starts the navigator and beacon_localization from cr17
* RMC_field.launch
    * Loads cr17 robot and RMCField_gazebo.world
    * Places the robot in the world with state publishers
* display_robot_model.launch
    * Loads the cr17.xacro model by default
    * Starts state publishers
    * Launches rviz
    * Meant to be able to test the code that does stuff with running motors and testing code without a full simulator (I think)
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
