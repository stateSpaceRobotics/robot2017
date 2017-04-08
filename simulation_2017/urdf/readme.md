These are models for the simulations.

urdf stands for [Unified Robot Description Format](http://wiki.ros.org/urdf) and is an XML model format.

xacro, [XML macro](http://wiki.ros.org/xacro), is meant for more easily creating urdf models.

Most of the urdf files were created through the xacro files, using the command rosrun xacro xacro model.xacro > model.urdf.  I do not recall exactly, but I think the difference is that the spawner for gazebo can use either, but rviz can only use urdf.

Models:
* cr17.xacro
    * The Chosen Robot
