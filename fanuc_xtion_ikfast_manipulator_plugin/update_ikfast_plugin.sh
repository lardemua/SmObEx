rosrun moveit_kinematics create_ikfast_moveit_plugin.py --search_mode=OPTIMIZE_MAX_JOINT --srdf_filename=fanuc_xtion.srdf --robot_name_in_srdf=fanuc_xtion --moveit_config_pkg=fanuc_xtion_moveit_config fanuc_xtion manipulator fanuc_xtion_ikfast_manipulator_plugin base_link camera_depth_optical_frame /home/joao/catkin_ws/src/fanuc_xtion_ikfast_manipulator_plugin/src/fanuc_xtion_manipulator_ikfast_solver.cpp