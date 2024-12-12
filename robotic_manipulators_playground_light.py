import tkinter as tk
import tkinter.ttk as ttk
import tkinter.simpledialog as sd
import tkinter.messagebox as ms
import tkinter.colorchooser as cc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import string, copy
import os, shutil
import serial, time
import roboticstoolbox as rtb
import numpy as np
import scipy as sc
import spatialmath as sm
import swift
import threading
import serial_port_finder as spf
import gui_buttons_labels as gbl
import robots_kinematics as kin
import general_functions as gf


# the class that creates the GUI window of the application
class robotic_manipulators_playground_window():
    def __init__(self, root, instance):
        self.root = root  # the root window of the GUI
        self.root.title("Robotic manipulators playground, made by Printzios Lampros (https://github.com/Lapricode)")  # the title of the windows
        self.root.geometry("+0+0")  # the position of the window
        self.gui_instance = instance  # the instance of the class that contains the functions and the variables of the robotic manipulator playground
        # do some initial actions for saving and loading the proper folders and files in the system
        menus_descriptions_folder_name = "menus_descriptions"  # the folder name to store the descriptions of the menus
        robots_run_by_swift_file_name = "robots_run_by_swift.txt"  # the file name to store the robots that can be simulated by the Swift simulator
        robots_models_descriptions_folder_name = "robots_models_descriptions"  # the folder name to store the descriptions (meshes and urdfs/xacros) of the robots models
        saved_robotic_manipulators_folder_name = "saved_robotic_manipulators"  # the folder name to store the saved robotic manipulators
        self.menus_descriptions_folder_path = str(os.getcwd() + fr"/{menus_descriptions_folder_name}")  # the path of the folder where the descriptions of the menus are stored
        self.robots_run_by_swift_file_path = str(os.getcwd() + fr"/{robots_run_by_swift_file_name}")  # the path of the file to store the robots that can be simulated by the Swift simulator
        self.saved_robots_descriptions_folder_path = str(os.getcwd() + fr"/{robots_models_descriptions_folder_name}")  # the path of the folder where the descriptions (meshes and urdfs/xacros) of the robots models are stored
        self.saved_robotic_manipulators_folder_path = str(os.getcwd() + fr"/{saved_robotic_manipulators_folder_name}")  # the path of the folder where the saved robotic manipulators are stored
        # if robots_models_descriptions_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the descriptions (meshes and urdfs/xacros) of the robots models does not exist
        #     os.mkdir(self.saved_robots_descriptions_folder_path)  # create the folder to store the descriptions (meshes and urdfs/xacros) of the robots models
        #     for robot_name in self.swift_simulated_robots_list:  # for each robotic manipulator that can be simulated by the Swift simulator
        #         os.mkdir(self.saved_robots_descriptions_folder_path + fr"/{robot_name}_description")  # create a folder for the robotic manipulator model
        #         os.mkdir(self.saved_robots_descriptions_folder_path + fr"/{robot_name}_description/meshes")  # create a folder to store the meshes of the robotic manipulator model
        #         os.mkdir(self.saved_robots_descriptions_folder_path + fr"/{robot_name}_description/urdf")  # create a folder to store the urdf of the robotic manipulator model
        # if saved_robotic_manipulators_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the saved robotic manipulator models does not exist
        #     os.mkdir(self.saved_robotic_manipulators_folder_path)  # create the folder to store the saved robotic manipulator models
        #     os.mkdir(self.saved_control_law_parameters_folder_path)  # create the folder to store the saved control law parameters
        # if menus_descriptions_folder_name not in os.listdir(os.getcwd()):  # if the folder to store the descriptions of the menus does not exist
        #     os.mkdir(self.menus_descriptions_folder_path)  # create the folder to store the descriptions of the menus
        self.rtbdata_robots_meshes_path = str(rtb.rtb_path_to_datafile("xacro"))  # the rtbdata path to store the meshes and urdfs/xacros of the robotic manipulators models in the library
        robots_models_descriptions_folders_list = [folder for folder in os.listdir(self.saved_robots_descriptions_folder_path) if os.path.isdir(self.saved_robots_descriptions_folder_path + fr"/{folder}")]  # the folders of the robots models descriptions
        for robot_model in robots_models_descriptions_folders_list:  # for each robotic manipulator model
            if robot_model not in os.listdir(self.rtbdata_robots_meshes_path):  # if the robotic manipulator model is not in the rtbdata folder
                shutil.copytree(self.saved_robots_descriptions_folder_path + fr"/{robot_model}", self.rtbdata_robots_meshes_path + fr"/{robot_model}")  # copy the robot model to the rtbdata folder
            else:  # if the robotic manipulator model is in the rtbdata folder
                shutil.rmtree(self.rtbdata_robots_meshes_path + fr"/{robot_model}")  # remove the robotic manipulator model from the rtbdata folder
                shutil.copytree(self.saved_robots_descriptions_folder_path + fr"/{robot_model}", self.rtbdata_robots_meshes_path + fr"/{robot_model}")  # copy the robot model to the rtbdata folder
        # define the menus of the GUI
        self.main_menu_choice = 0  # the number choice of the main menu
        self.general_GUI_description = open(self.menus_descriptions_folder_path + "/general.txt", "r", encoding = "utf-8").read()  # the general description of the GUI
        self.submenus_titles = [["Define robot's parameters", "Adjust the visualization"], ["Forward kinematics", "Inverse kinematics"], ["Differential kinematics", "Inverse differential kinematics"], \
                            ["Establish communication with arduino microcontroller", "Serial monitor / Console", "Control joints and end-effector motors"]]  # the titles of the submenus
        self.submenus_descriptions = []  # the descriptions of the submenus
        for menu_num in range(len(self.submenus_titles)):  # for each menu
            self.submenus_descriptions.append([])  # append an empty list to store the descriptions of the submenus for the current menu
            for submenu_num in range(len(self.submenus_titles[menu_num])):  # for each submenu
                try: self.submenus_descriptions[-1].append("".join(open(self.menus_descriptions_folder_path + fr"/menu_{menu_num + 1}/submenu_{submenu_num + 1}.txt", "r", encoding = "utf-8").readlines()))  # read the description of the current submenu
                except: pass
        construct_robotic_manipulator_menu_build_details = dict(title = "Construct the robotic manipulator", submenus_titles = self.submenus_titles[0], submenus_descriptions = self.submenus_descriptions[0], \
                                                                build_function = self.build_construct_robotic_manipulator_menus)  # a dictionary to store the details of the construct robotic manipulator menu
        forward_kinematics_menu_build_details = dict(title = "Forward kinematics analysis", submenus_titles = self.submenus_titles[1], submenus_descriptions = self.submenus_descriptions[1], \
                                                        build_function = self.build_robotic_manipulator_forward_kinematics_menus)  # a dictionary to store the details of the forward kinematics menu
        differential_kinematics_menu_build_details = dict(title = "Differential kinematics analysis", submenus_titles = self.submenus_titles[2], submenus_descriptions = self.submenus_descriptions[2], \
                                                            build_function = self.build_robotic_manipulator_differential_kinematics_menus)  # a dictionary to store the details of the differential kinematics menu
        control_robotic_manipulator_menu_build_details = dict(title = "Control the robotic manipulator", submenus_titles = self.submenus_titles[3], submenus_descriptions = self.submenus_descriptions[3], \
                                                                build_function = self.build_control_robotic_manipulator_menus)  # a dictionary to store the details of the control robotic manipulator menu
        self.main_menus_build_details = [construct_robotic_manipulator_menu_build_details, forward_kinematics_menu_build_details, differential_kinematics_menu_build_details, control_robotic_manipulator_menu_build_details]  # a list to store the basic details of all the main menus in order to build them
        # define the workspace parameters
        self.workspace_canvas_points = []  # the list to store the points of the workspace
        self.pointing_to_point = "(0.000, 0.000, 0.000)"  # the point to which the user's cursor is pointing to in the workspace
        self.x_axis_range = 0.5  # the shown length of the x axis of the workspace
        self.y_axis_range = 0.5  # the shown length of the y axis of the workspace
        self.z_axis_range = 0.5  # the shown length of the z axis of the workspace
        self.axis_range_values = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.5, 2.0]  # the possible values of the axis ranges of the workspace
        self.magnify_workspace_constant = 400  # the constant used to magnify the workspace
        self.workspace_width_ratio = 0.5  # the ratio of the width of the workspace area to the width of the root window
        self.workspace_width_ratios_values = [0.3, 0.5, 0.7, 1.0]  # the possible values of the workspace width ratio
        self.workspace_sensitivity_values = [0.1, 0.5, 1.0]; self.workspace_sensitivity_degrees = ["high", "normal", "low"]  # the possible values/degrees of the sensitivity of the workspace control
        self.workspace_control_sensitivity = self.workspace_sensitivity_values[self.workspace_sensitivity_degrees.index("normal")]  # control the workspace motion sensitivity
        self.axis_enable_visualization = "shown"  # control the axis visualization
        self.terrain_enable_visualization = "shown"  # control the terrain visualization
        self.robotic_manipulator_enable_visualization = "shown"  # control the robotic manipulator visulization
        self.robot_joints_enable_visualization = "shown"  # control the robotic manipulator joints visualization
        self.robot_links_enable_visualization = "shown"  # control the robotic manipulator links visualization
        self.end_effector_frame_enable_visualization = "shown"  # control the end-effector frame visualization
        self.x_axis_view = "+"  # the view of the x axis of the workspace, it can be "+" (front) or "-" (back) or "0" (none)
        self.y_axis_view = "+"  # the view of the y axis of the workspace, it can be "+" (right) or "-" (left) or "0" (none)
        self.z_axis_view = "+"  # the view of the z axis of the workspace, it can be "+" (up) or "-" (down) or "0" (none)
        self.control_or_kinematics_variables_visualization_list = ["control", "kinematics"]  # the possible values of the control or fkine variables visualization
        self.control_or_kinematics_variables_visualization = self.control_or_kinematics_variables_visualization_list[0]  # determine which variables (control or forward kinematics) to visualize
        # define the robotic manipulator model technical parameters
        self.robotic_manipulator_model_name = ""  # the name of the current robotic manipulator model
        self.joints_number = 6  # set the total number of joints of the robotic manipulator
        self.frames_number, self.links_number = self.set_frames_links_number(joints_number = self.joints_number)  # set the total number of frames and links of the robotic manipulator
        self.max_joints_number = 12  # the maximum possible number of joints of the robotic manipulator
        self.chosen_joint_number_model = 1  # the number of the chosen joint of the robotic manipulator for model changes
        self.joints_types_list = ["revolute", "prismatic"]  # the possible types of the robotic manipulator joints
        self.joints_types = [self.joints_types_list[0] for _ in range(self.joints_number)]  # the types of the robotic manipulator joints
        self.den_har_parameters_extreme_limits = [[-360.0, 360.0], [-5.0, 5.0]]  # the extreme limits of the parameters in degrees or meters, depending on the joint's type
        self.a_den_har_parameters = [0.0 for _ in range(self.joints_number)]  # the a parameters of the Denavit - Hartnemberg convention in meters
        self.a_parameters_limits = copy.deepcopy(self.den_har_parameters_extreme_limits[1])  # the limits of the a parameters in meters
        self.d_den_har_parameters = [0.0 for _ in range(self.joints_number)]  # the d parameters of the Denavit - Hartnemberg convention in meters
        self.d_parameters_limits = copy.deepcopy(self.den_har_parameters_extreme_limits[1])  # the limits of the d parameters in meters
        self.alpha_den_har_parameters = [0.0 for _ in range(self.joints_number)]  # the alpha parameters of the Denavit - Hartnemberg convention in radians
        self.alpha_parameters_limits = copy.deepcopy(np.deg2rad(self.den_har_parameters_extreme_limits[0]))  # the limits of the alpha parameters in radians
        self.theta_den_har_parameters = [0.0 for _ in range(self.joints_number)]  # the theta parameters of the Denavit - Hartnemberg convention in radians
        self.theta_parameters_limits = copy.deepcopy(np.deg2rad(self.den_har_parameters_extreme_limits[0]))  # the limits of the theta parameters in radians
        self.base_position_wrt_world = np.array([0.0, 0.0, 0.0])  # the position of the base system (in meters) of the robotic manipulator in the workspace, wrt the world frame
        self.base_position_limits = [[-10.0, 10.0], [-10.0, 10.0], [-10.0, 10.0]]  # the limits of the base position in meters
        self.zero_frame_position_wrt_base = np.array([0.0, 0.0, 0.0])  # the position of the zero frame (in meters) of the robotic manipulator in the workspace, wrt the base system
        self.zero_frame_position_limits = [[-10.0, 10.0], [-10.0, 10.0], [-10.0, 10.0]]  # the limits of the zero frame position in meters
        self.base_orientation_wrt_world = np.array([0.0, 0.0, 0.0])  # the orientation of the base system (xyz extrinsic Euler angles in radians) of the robotic manipulator in the workspace, wrt the world frame
        self.base_orientation_limits = [[-180.0, 180.0], [-180.0, 180.0], [-180.0, 180.0]]  # the limits of the base orientation angles in degrees
        self.zero_frame_orientation_wrt_base = np.array([0.0, 0.0, 0.0])  # the orientation of the zero frame system (xyz extrinsic Euler angles in radians) of the robotic manipulator in the workspace, wrt the base system
        self.zero_frame_orientation_limits = [[-180.0, 180.0], [-180.0, 180.0], [-180.0, 180.0]]  # the limits of the zero frame orientation angles in degrees
        self.end_effector_position_wrt_last_frame = np.array([0.0, 0.0, 0.0])  # the position of the end-effector system (in meters) of the robotic manipulator in the workspace, wrt the last joint frame
        self.end_effector_position_limits = [[-10.0, 10.0], [-10.0, 10.0], [-10.0, 10.0]]  # the limits of the end-effector position in meters
        self.end_effector_orientation_wrt_last_frame = np.array([0.0, 0.0, 0.0])  # the orientation of the end-effector system (xyz extrinsic Euler angles in radians) of the robotic manipulator in the workspace, wrt the last joint frame
        self.end_effector_orientation_limits = [[-180.0, 180.0], [-180.0, 180.0], [-180.0, 180.0]]  # the limits of the end-effector orientation angles in degrees
        self.robotic_manipulator_is_built = False  # the flag to check if the robotic manipulator is built
        self.built_robotic_manipulator = None  # the current built robotic manipulator model
        self.built_robotic_manipulator_info = {}  # the information of the built robotic manipulator model
        self.saved_robots_models_files_list = [file[:-4] for file in os.listdir(self.saved_robotic_manipulators_folder_path)]  # the list of the saved robotic manipulator models
        # define the robotic manipulator visualization parameters
        self.min_visualization_size = 1  # the minimum size of the visualization of the workspace
        self.max_visualization_size = 10  # the maximum size of the visualization of the workspace
        self.workspace_canvas_color = "#73938b"  # the color of the workspace canvas
        self.up_side_terrain_color = "#6e6e6e"  # the color of the up side of the workspace canvas
        self.down_side_terrain_color = "#333333"  # the color of the down side of the workspace canvas
        self.workspace_terrain_color = self.up_side_terrain_color  # the color of the terrain of the workspace
        self.workspace_axis_colors = ["red", "green", "blue"]  # the colors of the x, y, z axis of the workspace
        self.workspace_axis_sizes = 5  # the sizes of the x, y, z, axis of the workspace
        self.chosen_frame_visualization = "frame 0"  # the chosen frame of the robotic manipulator for visualization
        self.chosen_link_visualization = "link 1"  # the chosen link of the robotic manipulator for visualization
        self.joints_z_axis_positions = [0.0 for _ in range(self.joints_number)]  # the z axis (of the joint's frame) positions of the robotic manipulator joints
        self.frames_origins_colors = ["black" for _ in range(self.frames_number)]  # the colors of the robotic manipulator frames origins
        self.frames_origins_sizes = [7 for _ in range(self.frames_number)]  # the widths of the robotic manipulator frames origins
        self.initial_links_lengths = [0.0 for _ in range(self.links_number)]  # the initial lengths of the robotic manipulator links
        self.links_colors = ["red" for _ in range(self.links_number)]  # the colors of the robotic manipulator links
        self.links_sizes = [5 for _ in range(self.links_number)]  # the widths of the robotic manipulator links
        # define the parameters for the robotic manipulator control
        self.angles_precision = 1  # the decimal precision of the revolute variables
        self.distances_precision = 3  # the decimal precision of the prismatic variables
        self.chosen_joint_number_control = 1  # the number of the chosen joint of the robotic manipulator for the control
        self.control_joints_variables = [0.0 for _ in range(self.joints_number)]  # the variables for the robotic manipulator control in radians or meters, depending on the joint's type
        self.control_joints_variables_extreme_limits = copy.deepcopy(self.den_har_parameters_extreme_limits)  # the extreme limits of the control joints variables in degrees or meters, depending on the joint's type
        self.control_joints_variables_limits = [copy.deepcopy(self.control_joints_variables_extreme_limits) for _ in range(self.joints_number)]  # the limits of the control joints variables in degrees or meters, depending on the joint's type
        self.chosen_joint_motor_number = 0  # the order number of the chosen control joint motor of the robotic manipulator
        self.joints_motors_list = list([string.ascii_uppercase[joint]] for joint in range(self.joints_number))  # the list of the motors of the robotic manipulator
        self.joints_motors_mult_factors = [[1.0] for _ in range(self.joints_number)]  # a list to store the multiplication factors for the joints variables to be sent as commands to the joints motors
        self.control_end_effector_variable = 0.0  # the control variable for the control end-effector
        self.control_end_effector_limits = [0, 100]  # the control limits of the control end-effector
        self.end_effector_motor = "S"  # the motor of the end-effector
        self.end_effector_motor_mult_factor = 1.0  # the multiplication factor for the end-effector motor
        self.robotic_manipulator_control_mode = "manual"  # the mode of the robotic manipulator control, it can be "manual" or "automatic"
        self.robotic_manipulator_control_modes_list = ["manual", "automatic"]  # the possible modes of the robotic manipulator control
        # define the serial communication parameters
        self.serial_connection = serial.Serial()  # define the serial communication object for the serial connection between the arduino microcontroller and the computer
        self.serial_connection_thread_flag = False  # the flag to run/stop the serial communication thread
        self.serial_connection_elapsed_time = 0.0  # the elapsed time of the serial communication
        self.closed_serial_connection_message = "The serial connection is closed!"  # the message to be emitted when the serial connection is closed
        self.serial_connect_disconnect_command = "Connect"  # the command to connect or disconnect the serial communication between the arduino microcontroller and the computer
        self.available_serial_ports = []  # a list to store the available serial ports of the computer
        self.serial_port = ""  # the serial port used for the serial communication with the arduino microcontroller
        self.baudrates_list = ["115200", "57600", "38400", "28800", "19200", "14400", "9600", "4800", "2400", "1200", "600", "300"]  # the possible baudrates of the serial communication
        self.baudrate = self.baudrates_list[0]  # the baudrate used for the serial communication with the arduino microcontroller
        self.serial_connection_state = "Disconnected"  # the state of the serial connection with the arduino microcontroller
        self.serial_connection_states_list = ["Disconnected", "Connected", "Connected - Idle", "Connected - Run", "Connected - Home", "Connected - Almanipulator", "Connected - Hold", "Connected to Port"]  # the possible states of the serial connection with the arduino microcontroller
        self.serial_connection_indicator_colors = ["#ff0000", "#00ff00", "#00ff4b", "#00ff96", "#00ffff", "#ffff00", "#ff00ff", "#ffffff"]  # the colors of the serial connection indicator
        self.serial_monitor_text = ""  # the string variable to store the text of the serial monitor
        self.command_starting_text = ""  # the starting text of the commands sent to the arduino microcontroller
        self.command_ending_text = ""  # the ending text of the commands sent to the arduino microcontroller
        self.show_status_responses = False  # the flag to show the status responses of the robotic manipulator
        self.show_ok_responses = False  # the flag to show the ok responses for the commands sent to the robotic manipulator
        self.show_responses_indicators = ["✖", "✔"]  # the possible values of the show responses indicators
        self.expanded_serial_monitor = False  # choose if you want to expand the serial monitor menu
        self.allow_sending_all_ports = False  # choose if you want to allow sending commands to all serial ports
        # define the parameters for the kinematics (forward kinematics, inverse kinematics, differential kinematics, inverse differential kinematics) of the robotic manipulator
        self.chosen_joint_number_fkine = 1  # the number of the chosen joint of the robotic manipulator for the forward kinematics
        self.forward_kinematics_variables = [0.0 for _ in range(self.joints_number)]  # the variables of the forward kinematics (if the joint is revolute, the variable is the theta angle parameter in radians, else if the joint is prismatic, the variable is the d distance parameter in meters)
        self.chosen_frame_fkine = "end-effector"  # the chosen robotic manipulator frame for the forward kinematics
        self.fkine_frame_position = np.array([0.0, 0.0, 0.0])  # the position of the chosen robotic manipulator frame for the forward kinematics in meters
        self.fkine_frame_orientation = np.array([0.0, 0.0, 0.0])  # the orientation of the chosen robotic manipulator frame for the forward kinematics in radians (initialized in Euler angles form in radians)
        self.orientation_representation_fkine = "Euler xyz (°)"  # the representation of the orientation of the chosen robotic manipulator frame for the forward kinematics
        self.orientation_representations_fkine_list = ["Euler xyz (°)", "Quaternion", "Rot. matrix"]  # the possible representations of the orientation of the chosen robotic manipulator frame for the forward kinematics
        self.joints_range_divisions_list = [2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20]  # the possible values of the joints range divisions for the calculation of the robot's reachable workspace
        self.joints_range_divisions = self.joints_range_divisions_list[1]  # the number of joints range divisions for the calculation of the robot's reachable workspace
        self.chosen_invkine_3d_position = np.array([0.0, 0.0, 0.0])  # the 3D position of the end-effector for the inverse kinematics in meters
        self.chosen_invkine_orientation = np.array([0.0, 0.0, 0.0])  # the orientation of the end-effector for the inverse kinematics in radians (expressed in xyz extrinsic Euler angles in radians)
        self.invkine_joints_configuration = [0.0 for _ in range(self.joints_number)]  # the joints configuration for the inverse kinematics in radians or meters, depending on the joint's type
        self.invkine_tolerance = 1e-10  # the tolerance for the inverse kinematics
        self.chosen_joint_number_diffkine = 1  # the number of the chosen joint of the robotic manipulator for the differential kinematics
        self.differential_kinematics_velocities = [0.0 for _ in range(self.joints_number)]  # the velocities of the joints for the differential kinematics
        self.diffkine_velocities_limits = [[[-360, 360], [-1, 1]] for _ in range(self.joints_number)]  # the limits of the velocities of the joints for the differential kinematics
        self.diffkine_linear_vel = np.array([0.0, 0.0, 0.0])  # the velocity of the end-effector in the workspace for the differential kinematics in meters/sec
        self.diffkine_angular_vel = np.array([0.0, 0.0, 0.0])  # the angular velocity of the end-effector for the differential kinematics in radians/sec
        self.diffkine_wrt_frame_list = ["world", "end-effector"]  # the possible values of the differential kinematics wrt frame
        self.diffkine_wrt_frame = self.diffkine_wrt_frame_list[0]  # the frame wrt which the differential kinematics is calculated
        self.chosen_invdiffkine_linear_vel = np.array([0.0, 0.0, 0.0])  # the velocity of the end-effector for the inverse differential kinematics in meters/sec
        self.chosen_invdiffkine_angular_vel = np.array([0.0, 0.0, 0.0])  # the angular velocity of the end-effector for the inverse differential kinematics in radians/sec
        self.invdiffkine_joints_velocities = [0.0 for _ in range(self.joints_number)]  # the joints velocities for the inverse differential kinematics in radians/sec or meters/sec, depending on the joint's type
        self.invdiffkine_wrt_frame_list = ["world", "end-effector"]  # the possible values of the inverse differential kinematics wrt frame
        self.invdiffkine_wrt_frame = self.invdiffkine_wrt_frame_list[0]  # the frame wrt which the inverse differential kinematics is calculated
        # define the variables for the online Swift simulator
        self.swift_simulated_robots_list = [line.strip() for line in open(self.robots_run_by_swift_file_path)]  # the list of the robotic manipulators that can be simulated by the Swift simulator
        self.swift_sim_env = None  # the variable for the Swift simulator environment
        self.swift_sim_thread_flag = False  # the flag to run/stop the Swift simulation thread
        self.swift_sim_dt = 0.01  # the time step of the Swift simulation in seconds
        self.swift_robotic_manipulator = None  # the robotic manipulator model built inside the Swift simulator
        # do some initial actions for the GUI window
        initial_root_width = 0.9 * self.root.winfo_screenwidth()  # the initial width of the root window
        self.workspace_area_width = self.workspace_width_ratio * initial_root_width  # the width of the workspace area
        self.workspace_area_height = 4/5 * self.root.winfo_screenheight()  # the height of the workspace area
        self.workspace_menus_bd_width = 5  # the border width of the workspace and menus areas
        self.menus_area_width = (1 - self.workspace_width_ratio) * initial_root_width  # the width of the menus area
        self.menus_area_height = self.workspace_area_height  # the height of the menus area
        self.root_was_not_in_focus = False  # boolean variable to check if the GUI window was previously minimized or generally not in focus
        self.canvas_fps = 100  # the frames per second of the workspace canvas
        self.canvas_destroy_counter = 0  # the counter to destroy the workspace canvas
        self.create_workspace_menus_areas()  # create the workspace area and the menus area
        self.change_main_menu(self.main_menu_choice)  # create the sub menus for the current main menu
        self.reset_workspace_canvas()  # reset the position of the axis origin to be on the center of the workspace
        self.draw_next_workspace_canvas_frame()  # begin the loop for controlling the motion of the workspace visualization
        # self.load_robotic_manipulator_model("Default")  # load the default robotic manipulator model
        # create the necessary bindings for the GUI window
        self.root.bind("<Control-r>", self.resize_root_window); self.root.bind("<Control-R>", self.resize_root_window)  # bind the resize window function to the Control + r and Control + R keys combinations
        self.root.bind("<Control-s>", self.save_robotic_manipulator_model_file); self.root.bind("<Control-S>", self.save_robotic_manipulator_model_file)  # bind the save robot model function to the Control + s and Control + S keys combinations
        self.root.bind("<Control-b>", self.build_robotic_manipulator_model); self.root.bind("<Control-B>", self.build_robotic_manipulator_model)  # bind the build robot model function to the Control + b and Control + B keys combinations
        self.root.bind("<Control-v>", self.visualize_control_or_kinematics_variables); self.root.bind("<Control-V>", self.visualize_control_or_kinematics_variables)  # bind the visualize control or kinematics variables function to the Control + v and Control + V keys combinations
        self.root.bind("<Control-z>", self.visualize_robotic_manipulator); self.root.bind("<Control-Z>", self.visualize_robotic_manipulator)  # bind the visualize robotic manipulator function to the Control + z and Control + Z keys combinations
        # self.root.bind("<FocusIn>", self.resize_root_window_2)  # bind the resize window function to the focus in event
        # self.root.bind("<Unmap>", self.root_not_in_focus)  # bind the resize window function to the focus out event
        for main_menu_num in range(len(self.main_menus_build_details)):
            self.root.bind(f"<Control-Key-{main_menu_num + 1}>", lambda event, num = main_menu_num: self.change_main_menu(num, event))
        self.root.bind("<Control-e>", lambda event: self.change_workspace_width_ratio(-1, event)); self.root.bind("<Control-E>", lambda event: self.change_workspace_width_ratio(-1, event))  # bind the change workspace width ratio function to the Control + e and Control + E keys combinations
        self.root.bind("<Control-d>", lambda event: self.change_workspace_width_ratio(+1, event)); self.root.bind("<Control-D>", lambda event: self.change_workspace_width_ratio(+1, event))  # bind the change workspace width ratio function to the Control + d and Control + D keys combinations
    def resize_root_window(self, event = None):  # resize the root window
        self.resize_root_window_2()  # resize the root window
    def resize_root_window_2(self, event = None):  # resize the root window
        if event == None or (self.root.state() == "normal" and self.root_was_not_in_focus):  # if the root window is in focus or the resize function is called by another function
            self.workspace_area.destroy()  # destroy the workspace area
            self.menus_area.destroy()  # destroy the menus area
            self.workspace_area_width = self.workspace_width_ratio * self.root.winfo_width() - 4. * self.workspace_menus_bd_width  # recalculate the width of the workspace area
            self.workspace_area_height = self.root.winfo_height() - 4. * self.workspace_menus_bd_width  # recalculate the height of the workspace area
            self.menus_area_width = (1 - self.workspace_width_ratio) * self.root.winfo_width() - 4. * self.workspace_menus_bd_width  # the width of the menus area
            self.menus_area_height = self.workspace_area_height  # the height of the menus area
            self.root_was_not_in_focus = False  # the GUI window was previously in focus
            self.create_workspace_menus_areas()  # recreate the workspace area and the menus area
            self.change_main_menu(self.main_menu_choice)  # recreate the sub menus for the current main menu
            self.reset_workspace_canvas_2()  # reset the position of the axis origin to be on the center of the workspace
            self.draw_next_workspace_canvas_frame()  # begin the loop for controlling the motion of the workspace visualization
    def root_not_in_focus(self, event):  # the function run when the root window is not in focus
        self.root_was_not_in_focus = True  # the GUI window was previously not in focus
    def alternate_matrix_elements(self, matrix, index_element):  # alternate the parametres that are inside the matrix based on the current index_element
        return (matrix[1:] + [matrix[0]])[matrix.index(index_element)]
    def create_workspace_menus_areas(self):  # create the workspace area and the menus area
        # create the workspace area
        self.workspace_area_borders_length = self.workspace_area_height / 12.0  # the length of the borders of the workspace area
        self.workspace_up_edge_height = self.workspace_area_borders_length  # the height of the up edge of the workspace area
        self.workspace_left_edge_width = 2.5 * self.workspace_area_borders_length  # the width of the left edge of the workspace area
        self.workspace_right_edge_width = 0.5 * self.workspace_area_borders_length  # the width of the right edge of the workspace area
        self.workspace_canvas_width = self.workspace_area_width - (self.workspace_left_edge_width + self.workspace_right_edge_width)  # the width of the canvas of the workspace area
        self.workspace_canvas_height = self.workspace_area_height - self.workspace_up_edge_height  # the height of the canvas of the workspace area
        self.switch_coor_system_matrix = np.array([[0, 1, 0, self.workspace_canvas_width / 2], [0, 0, -1, self.workspace_canvas_height / 2], [1, 0, 0, 0], [0, 0, 0, 1]])  # the transformation matrix needed because of the different workspace and canvas coordinates systems
        self.workspace_area_edges_color = "cyan"  # the color of the edges of the workspace area
        self.workspace_area_canvas_color = self.workspace_canvas_color  # the color of the canvas of the workspace area
        self.workspace_options_rows = 20  # the number of rows of the options located at the borders of the workspace area
        self.workspace_borders_font = 9  # the font of the options located at the borders of the workspace area
        self.workspace_title_font = 15  # the font of the title of the workspace area
        self.workspace_area = tk.Frame(self.root, width = self.workspace_area_width, height = self.workspace_area_height, bg = "black", bd = self.workspace_menus_bd_width, relief = "solid")
        self.workspace_area.grid(row = 0, column = 0, sticky = tk.NSEW)
        self.workspace_area_up_edge = tk.Frame(self.workspace_area, width = self.workspace_area_width, height = self.workspace_up_edge_height, bg = self.workspace_area_edges_color, bd = 5, relief = "ridge")
        self.workspace_area_up_edge.grid(row = 0, column = 0, columnspan = 3, sticky = tk.NSEW)
        self.workspace_area_left_edge = tk.Frame(self.workspace_area, width = self.workspace_left_edge_width, height = self.workspace_canvas_height, bg = self.workspace_area_edges_color, bd = 3, relief = "solid")
        self.workspace_area_left_edge.grid(row = 1, column = 0, sticky = tk.NSEW)
        self.workspace_area_right_edge = tk.Frame(self.workspace_area, width = self.workspace_right_edge_width, height = self.workspace_canvas_height, bg = self.workspace_area_edges_color, bd = 3, relief = "solid")
        self.workspace_area_right_edge.grid(row = 1, column = 2, sticky = tk.NSEW)
        workspace_area_title_ord = 1/2.5; workspace_area_title_x = 1/2; gbl.menu_label(self.workspace_area_up_edge, "World frame - Robotic manipulator and its Environment", f"Calibri {self.workspace_title_font} bold", "black", self.workspace_area_edges_color, workspace_area_title_x * self.workspace_area_width, workspace_area_title_ord * self.workspace_up_edge_height)
        general_info_button_ord = 1/2.5; general_info_button_x = 1/20; self.general_info_button = gbl.menu_button(self.workspace_area_up_edge, "ⓘ", f"Calibri {self.workspace_borders_font+2} bold", "black", self.workspace_area_edges_color, general_info_button_x * self.workspace_area_width, general_info_button_ord * self.workspace_up_edge_height, lambda event: ms.showinfo("General instructions", self.general_GUI_description, master = self.root)).button
        self.create_workspace_canvas()  # create the workspace canvas
        # create the workspace options located at the borders of the workspace area
        workspace_width_ratio_label_ord = 1
        workspace_width_ratio_button_ord = workspace_width_ratio_label_ord+1.2
        workspace_sensitivity_label_ord = workspace_width_ratio_button_ord+1
        workspace_sensitivity_button_ord = workspace_sensitivity_label_ord+0.8
        x_axis_range_label_ord = workspace_sensitivity_button_ord+1.4
        x_axis_range_button_ord = x_axis_range_label_ord+1.2
        y_axis_range_label_ord = x_axis_range_label_ord
        y_axis_range_button_ord = y_axis_range_label_ord+1.2
        z_axis_range_label_ord = x_axis_range_label_ord
        z_axis_range_button_ord = z_axis_range_label_ord+1.2
        show_axis_label_ord = z_axis_range_button_ord+1
        show_axis_button_ord = show_axis_label_ord+0.8
        show_terrain_label_ord = show_axis_label_ord
        show_terrain_button_ord = show_terrain_label_ord+0.8
        show_robotic_manipulator_label_ord = show_terrain_button_ord+1
        show_robotic_manipulator_button_ord = show_robotic_manipulator_label_ord+0.8
        show_manipulator_points_label_ord = show_robotic_manipulator_button_ord+1
        show_manipulator_points_button_ord = show_manipulator_points_label_ord+0.8
        show_manipulator_links_label_ord = show_manipulator_points_button_ord+1
        show_manipulator_links_button_ord = show_manipulator_links_label_ord+0.8
        show_end_effector_frame_label_ord = show_manipulator_links_button_ord+1
        show_end_effector_frame_button_ord = show_end_effector_frame_label_ord+0.8
        x_axis_view_label_ord = show_end_effector_frame_button_ord+1.4
        x_axis_view_button_ord = x_axis_view_label_ord+1.2
        y_axis_view_label_ord = x_axis_view_label_ord
        y_axis_view_button_ord = y_axis_view_label_ord+1.2
        z_axis_view_label_ord = x_axis_view_label_ord
        z_axis_view_button_ord = z_axis_view_label_ord+1.2
        information_text_label_ord = self.workspace_options_rows
        workspace_width_ratio_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Workspace / window\nwidth ratio:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, workspace_width_ratio_label_x * self.workspace_left_edge_width, workspace_width_ratio_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        workspace_width_ratio_button_x = workspace_width_ratio_label_x; self.change_workspace_width_ratio_button = gbl.menu_button(self.workspace_area_left_edge, self.workspace_width_ratio, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, workspace_width_ratio_button_x * self.workspace_left_edge_width, workspace_width_ratio_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), lambda event: self.change_workspace_width_ratio(+1, event)).button
        workspace_sensitivity_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Control sensitivity:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, workspace_sensitivity_label_x * self.workspace_left_edge_width, workspace_sensitivity_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        workspace_sensitivity_button_x = workspace_sensitivity_label_x; self.change_workspace_sensitivity_button = gbl.menu_button(self.workspace_area_left_edge, self.workspace_sensitivity_degrees[self.workspace_sensitivity_values.index(self.workspace_control_sensitivity)], f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, workspace_sensitivity_button_x * self.workspace_left_edge_width, workspace_sensitivity_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_workspace_control_sensitivity).button
        x_axis_range_label_x = 1/4; gbl.menu_label(self.workspace_area_left_edge, "x axis\n(m):", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, x_axis_range_label_x * self.workspace_left_edge_width, x_axis_range_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        x_axis_range_button_x = x_axis_range_label_x; self.change_x_axis_range_button = gbl.menu_button(self.workspace_area_left_edge, self.x_axis_range, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, x_axis_range_button_x * self.workspace_left_edge_width, x_axis_range_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_x_axis_range).button
        y_axis_range_label_x = 2/4; gbl.menu_label(self.workspace_area_left_edge, "y axis\n(m):", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, y_axis_range_label_x * self.workspace_left_edge_width, y_axis_range_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        y_axis_range_button_x = y_axis_range_label_x; self.change_y_axis_range_button = gbl.menu_button(self.workspace_area_left_edge, self.y_axis_range, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, y_axis_range_button_x * self.workspace_left_edge_width, y_axis_range_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_y_axis_range).button
        z_axis_range_label_x = 3/4; gbl.menu_label(self.workspace_area_left_edge, "z axis\n(m):", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, z_axis_range_label_x * self.workspace_left_edge_width, z_axis_range_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        z_axis_range_button_x = z_axis_range_label_x; self.change_z_axis_range_button = gbl.menu_button(self.workspace_area_left_edge, self.z_axis_range, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, z_axis_range_button_x * self.workspace_left_edge_width, z_axis_range_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_z_axis_range).button
        show_axis_label_x = 1/3; gbl.menu_label(self.workspace_area_left_edge, "Axis:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_axis_label_x * self.workspace_left_edge_width, show_axis_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_axis_button_x = show_axis_label_x; self.show_axis_button = gbl.menu_button(self.workspace_area_left_edge, self.axis_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_axis_button_x * self.workspace_left_edge_width, show_axis_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_axis).button
        show_terrain_label_x = 2/3; gbl.menu_label(self.workspace_area_left_edge, "Terrain:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_terrain_label_x * self.workspace_left_edge_width, show_terrain_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_terrain_button_x = show_terrain_label_x; self.show_terrain_button = gbl.menu_button(self.workspace_area_left_edge, self.terrain_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_terrain_button_x * self.workspace_left_edge_width, show_terrain_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_terrain).button
        show_robotic_manipulator_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Robotic manipulator:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_robotic_manipulator_label_x * self.workspace_left_edge_width, show_robotic_manipulator_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_robotic_manipulator_button_x = show_robotic_manipulator_label_x; self.show_robotic_manipulator_button = gbl.menu_button(self.workspace_area_left_edge, self.robotic_manipulator_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_robotic_manipulator_button_x * self.workspace_left_edge_width, show_robotic_manipulator_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_robotic_manipulator).button
        show_manipulator_points_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Manipulator points:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_manipulator_points_label_x * self.workspace_left_edge_width, show_manipulator_points_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_manipulator_points_button_x = show_manipulator_points_label_x; self.show_manipulator_points_button = gbl.menu_button(self.workspace_area_left_edge, self.robot_joints_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_manipulator_points_button_x * self.workspace_left_edge_width, show_manipulator_points_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_manipulator_joints).button
        show_manipulator_links_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "Manipulator links:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_manipulator_links_label_x * self.workspace_left_edge_width, show_manipulator_links_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_manipulator_links_button_x = show_manipulator_links_label_x; self.show_manipulator_links_button = gbl.menu_button(self.workspace_area_left_edge, self.robot_links_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_manipulator_links_button_x * self.workspace_left_edge_width, show_manipulator_links_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_manipulator_links).button
        show_end_effector_frame_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "End-effector frame:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, show_end_effector_frame_label_x * self.workspace_left_edge_width, show_end_effector_frame_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        show_end_effector_frame_button_x = show_end_effector_frame_label_x; self.show_end_effector_frame_button = gbl.menu_button(self.workspace_area_left_edge, self.end_effector_frame_enable_visualization, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, show_end_effector_frame_button_x * self.workspace_left_edge_width, show_end_effector_frame_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.show_end_effector_frame).button
        x_axis_view_label_x = 1/4; gbl.menu_label(self.workspace_area_left_edge, "x axis\nview:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, x_axis_view_label_x * self.workspace_left_edge_width, x_axis_view_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        x_axis_view_button_x = x_axis_view_label_x; self.change_x_axis_view_button = gbl.menu_button(self.workspace_area_left_edge, self.x_axis_view, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, x_axis_view_button_x * self.workspace_left_edge_width, x_axis_view_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_x_axis_view).button
        y_axis_view_label_x = 2/4; gbl.menu_label(self.workspace_area_left_edge, "y axis\nview:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, y_axis_view_label_x * self.workspace_left_edge_width, y_axis_view_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        y_axis_view_button_x = y_axis_view_label_x; self.change_y_axis_view_button = gbl.menu_button(self.workspace_area_left_edge, self.y_axis_view, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, y_axis_view_button_x * self.workspace_left_edge_width, y_axis_view_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_y_axis_view).button
        z_axis_view_label_x = 3/4; gbl.menu_label(self.workspace_area_left_edge, "z axis\nview:", f"Calibri {self.workspace_borders_font} bold", "black", self.workspace_area_edges_color, z_axis_view_label_x * self.workspace_left_edge_width, z_axis_view_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        z_axis_view_button_x = z_axis_view_label_x; self.change_z_axis_view_button = gbl.menu_button(self.workspace_area_left_edge, self.z_axis_view, f"Calibri {self.workspace_borders_font} bold", "magenta", self.workspace_area_edges_color, z_axis_view_button_x * self.workspace_left_edge_width, z_axis_view_button_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1), self.change_z_axis_view).button
        information_text_label_x = 1/2; gbl.menu_label(self.workspace_area_left_edge, "If the window freezes\nyou can press \"Ctrl-R\"\n(used for resizing too).", f"Calibri {self.workspace_borders_font} bold", "brown", self.workspace_area_edges_color, information_text_label_x * self.workspace_left_edge_width, information_text_label_ord * self.workspace_canvas_height / (self.workspace_options_rows + 1))
        # create the menus area
        self.menus_area_borders_length = self.workspace_area_borders_length  # the length of the borders of the menus area
        self.menus_background_width = self.menus_area_width  # the width of the menus background
        self.menus_background_height = self.menus_area_height - self.menus_area_borders_length  # the height of the menus background
        self.menus_area_edges_color = "lime"
        self.menus_area_color = "orange"
        self.menus_title_font = 15  # the font of the title of the menus area
        self.menus_area = tk.Frame(self.root, width = self.menus_area_width, height = self.menus_area_height, bg = self.menus_area_color, bd = self.workspace_menus_bd_width, relief = "solid")
        self.menus_area.grid(row = 0, column = 1, sticky = tk.NSEW)
        self.menus_area_up_edge = tk.Frame(self.menus_area, width = self.menus_area_width, height = self.menus_area_borders_length, bg = self.menus_area_edges_color, bd = 5, relief = "ridge")
        self.menus_area_up_edge.grid(row = 0, column = 0, sticky = tk.NSEW)
        menus_area_title_ord = 1/2.5; menus_area_title_x = 1/2; self.main_menu_label = gbl.menu_label(self.menus_area_up_edge, self.main_menus_build_details[self.main_menu_choice]['title'], f"Calibri {self.menus_title_font} bold", "black", self.menus_area_edges_color, menus_area_title_x * self.menus_area_width, menus_area_title_ord * self.menus_area_borders_length).label
        previous_main_menu_button_ord = 1/2.5; previous_main_menu_button_x = 1/6; self.previous_main_menu_button = gbl.menu_button(self.menus_area_up_edge, "⇦", f"Calibri {self.menus_title_font} bold", "red", self.menus_area_edges_color, previous_main_menu_button_x * self.menus_area_width, previous_main_menu_button_ord * self.menus_area_borders_length, lambda event: self.change_main_menu(self.main_menu_choice-1, event))
        next_main_menu_button_ord = 1/2.5; next_main_menu_button_x = 5/6; self.next_main_menu_button = gbl.menu_button(self.menus_area_up_edge, "⇨", f"Calibri {self.menus_title_font} bold", "red", self.menus_area_edges_color, next_main_menu_button_x * self.menus_area_width, next_main_menu_button_ord * self.menus_area_borders_length, lambda event: self.change_main_menu(self.main_menu_choice+1, event))
        self.clear_menus_background()  # clear the menus background
    def create_workspace_canvas(self, event = None):  # create the workspace canvas where the robotic manipulator and its environment are visualized
        try: self.workspace_canvas.destroy()  # destroy the previous workspace canvas
        except: pass
        # create the workspace canvas
        self.workspace_canvas = tk.Canvas(self.workspace_area, width = int(self.workspace_canvas_width), height = int(self.workspace_canvas_height), bg = self.workspace_canvas_color, bd = 3, relief = "raised")
        self.workspace_canvas.grid(row = 1, column = 1, sticky = tk.NSEW)
        # create the buttons existing on the canvas
        try:
            self.choose_visualized_variables_button.destroy()
            self.visualized_variables_values_indicator.destroy()
        except: pass
        self.choose_visualized_variables_button = gbl.menu_button(self.workspace_canvas, self.control_or_kinematics_variables_visualization, f"Calibri {int(1.0*self.workspace_borders_font)} bold", "black", self.workspace_canvas_color, 50, 50, self.visualize_control_or_kinematics_variables).button
        self.visualized_variables_values_indicator = gbl.menu_label(self.workspace_canvas, "", f"Calibri {int(1.0*self.workspace_borders_font)} bold", "black", self.workspace_canvas_color, 50, self.workspace_canvas_height / 2).label
        # create the necessary bindings for the robotic manipulator workspace visualization
        self.workspace_canvas.bind("<Button-3>", lambda event: self.transfer_workspace_start(event))
        self.workspace_canvas.bind("<B3-Motion>", lambda event: self.transfer_workspace(event))
        self.workspace_canvas.bind("<Double-Button-3>", lambda event: self.reset_workspace_canvas_2(event))
        self.workspace_canvas.bind("<Button-1>", lambda event: self.rotate_workspace_start(event))
        self.workspace_canvas.bind("<B1-Motion>", lambda event: self.rotate_workspace(event))
        self.workspace_canvas.bind("<MouseWheel>", lambda event: self.scale_workspace(event))

    # functions for the workspace options located at the borders of the workspace area
    def change_workspace_width_ratio(self, change_type = +1, event = None):  # change the width ratio of the workspace area to the root window
        if change_type == +1:  # if the width ratio of the workspace area is increased
            self.workspace_width_ratio = self.alternate_matrix_elements(self.workspace_width_ratios_values, self.workspace_width_ratio)
        elif change_type == -1:  # if the width ratio of the workspace area is decreased
            self.workspace_width_ratio = self.alternate_matrix_elements(self.workspace_width_ratios_values[::-1], self.workspace_width_ratio)
        self.change_workspace_width_ratio_button.configure(text = self.workspace_width_ratio)
        self.resize_root_window()  # resize the root window
    def change_workspace_control_sensitivity(self, event = None):  # change the workspace mouse control sensitivity
        self.workspace_control_sensitivity = self.alternate_matrix_elements(self.workspace_sensitivity_values, self.workspace_control_sensitivity)
        self.change_workspace_sensitivity_button.configure(text = self.workspace_sensitivity_degrees[self.workspace_sensitivity_values.index(self.workspace_control_sensitivity)])
    def change_x_axis_range(self, event = None):  # change the x axis range
        self.x_axis_range = self.alternate_matrix_elements(self.axis_range_values, self.x_axis_range)
        self.change_x_axis_range_button.configure(text = self.x_axis_range)
    def change_y_axis_range(self, event = None):  # change the y axis range
        self.y_axis_range = self.alternate_matrix_elements(self.axis_range_values, self.y_axis_range)
        self.change_y_axis_range_button.configure(text = self.y_axis_range)
    def change_z_axis_range(self, event = None):  # change the z axis range
        self.z_axis_range = self.alternate_matrix_elements(self.axis_range_values, self.z_axis_range)
        self.change_z_axis_range_button.configure(text = self.z_axis_range)
    def show_axis(self, event = None):  # show or hide the axis
        self.axis_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.axis_enable_visualization)
        self.show_axis_button.configure(text = self.axis_enable_visualization)
    def show_terrain(self, event = None):  # show or hide the terrain
        self.terrain_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.terrain_enable_visualization)
        self.show_terrain_button.configure(text = self.terrain_enable_visualization)
    def show_robotic_manipulator(self, event = None):  # show or hide the robotic manipulator
        self.robotic_manipulator_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.robotic_manipulator_enable_visualization)
        self.show_robotic_manipulator_button.configure(text = self.robotic_manipulator_enable_visualization)
    def show_manipulator_joints(self, event = None):  # show or hide the joints of the robotic manipulator
        self.robot_joints_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.robot_joints_enable_visualization)
        self.show_manipulator_points_button.configure(text = self.robot_joints_enable_visualization)
    def show_manipulator_links(self, event = None):  # show or hide the links of the robotic manipulator
        self.robot_links_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.robot_links_enable_visualization)
        self.show_manipulator_links_button.configure(text = self.robot_links_enable_visualization)
    def show_end_effector_frame(self, event = None):  # show or hide the frame of the end-effector
        self.end_effector_frame_enable_visualization = self.alternate_matrix_elements(["shown", "hidden"], self.end_effector_frame_enable_visualization)
        self.show_end_effector_frame_button.configure(text = self.end_effector_frame_enable_visualization)
    def change_x_axis_view(self, event = None):  # change the x axis view
        self.x_axis_view = self.alternate_matrix_elements(["0", "-", "+"], self.x_axis_view)
        self.change_x_axis_view_button.configure(text = self.x_axis_view)
        self.set_canvas_view()
    def change_y_axis_view(self, event = None):  # change the y axis view
        self.y_axis_view = self.alternate_matrix_elements(["0", "-", "+"], self.y_axis_view)
        self.change_y_axis_view_button.configure(text = self.y_axis_view)
        self.set_canvas_view()
    def change_z_axis_view(self, event = None):  # change the z axis view
        self.z_axis_view = self.alternate_matrix_elements(["0", "-", "+"], self.z_axis_view)
        self.change_z_axis_view_button.configure(text = self.z_axis_view)
        self.set_canvas_view()
    def visualize_control_or_kinematics_variables(self, event = None):  # choose the control or forward kinematics variables
        self.control_or_kinematics_variables_visualization = self.alternate_matrix_elements(self.control_or_kinematics_variables_visualization_list, self.control_or_kinematics_variables_visualization)
        self.update_model_visualization_indicators()  # update the indicators of the model visualization
        self.update_differential_kinematics_indicators()  # update the indicators of the differential kinematics
        self.update_inverse_differential_kinematics_indicators()  # update the indicators of the inverse differential kinematics
    def visualize_robotic_manipulator(self, event = None):  # visualize or hide the robotic manipulator
        if self.robotic_manipulator_enable_visualization == "shown":  # if the robotic manipulator is shown
            self.robotic_manipulator_enable_visualization = "hidden"
            self.robot_joints_enable_visualization = "hidden"
            self.robot_links_enable_visualization = "hidden"
            self.end_effector_frame_enable_visualization = "hidden"
        else:  # if the robotic manipulator is hidden
            self.robotic_manipulator_enable_visualization = "shown"
            self.robot_joints_enable_visualization = "shown"
            self.robot_links_enable_visualization = "shown"
            self.end_effector_frame_enable_visualization = "shown"
        self.show_robotic_manipulator_button.configure(text = self.robotic_manipulator_enable_visualization)
        self.show_manipulator_points_button.configure(text = self.robot_joints_enable_visualization)
        self.show_manipulator_links_button.configure(text = self.robot_links_enable_visualization)
        self.show_end_effector_frame_button.configure(text = self.end_effector_frame_enable_visualization)

    # functions for the control of the workspace
    def apply_workspace_transformation(self, event = None):  # apply the transformation defined by the proper transfer, rotation and scale variables to all the points of the workspace
        self.workspace_transfer_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, self.y_cor_workspace_origin], [0, 0, 1, self.z_cor_workspace_origin], [0, 0, 0, 1]])
        self.workspace_scale_matrix = np.array([[self.magnify_workspace_constant * self.scale_parameter, 0, 0, 0], [0, self.magnify_workspace_constant * self.scale_parameter, 0, 0], [0, 0, self.magnify_workspace_constant * self.scale_parameter, 0], [0, 0, 0, 1]])
        self.workspace_y_rot_matrix = np.array([[np.cos(np.deg2rad(self.rot_y_workspace)), 0, np.sin(np.deg2rad(self.rot_y_workspace)), 0], [0, 1, 0, 0], [-np.sin(np.deg2rad(self.rot_y_workspace)), 0, np.cos(np.deg2rad(self.rot_y_workspace)), 0], [0, 0, 0, 1]])
        self.workspace_z_rot_matrix = np.array([[np.cos(np.deg2rad(self.rot_z_workspace)), -np.sin(np.deg2rad(self.rot_z_workspace)), 0, 0], [np.sin(np.deg2rad(self.rot_z_workspace)), np.cos(np.deg2rad(self.rot_z_workspace)), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.workspace_rotation_matrix = self.workspace_y_rot_matrix @ self.workspace_z_rot_matrix
        self.workspace_transformation_matrix = self.workspace_transfer_matrix @ self.workspace_rotation_matrix @ self.workspace_scale_matrix
        transformed_axis_terrain_points = np.copy(self.axis_terrain_points)  # the transformed points of the axis and terrain
        transformed_robotic_manipulator_points = self.apply_robotic_manipulator_transformation()  # the transformed points of the robotic manipulator
        transformed_end_effector_points = self.apply_end_effector_transformation(self.end_effector_frame_points)  # the transformed points of the robotic manipulator end-effector
        self.workspace_canvas_points = np.concatenate((transformed_axis_terrain_points, transformed_robotic_manipulator_points, transformed_end_effector_points), axis = 0)
        self.canvas_moved_points = (self.switch_coor_system_matrix @ self.workspace_transformation_matrix @ self.workspace_canvas_points.T).T  # the moved points of the workspace, converted to canvas coordinates, after the workspace transformation is applied
        # for the visualization, based on the orientation of the terrain plane
        terrain_draw_side = np.dot(self.workspace_rotation_matrix[:3, :3] @ np.array([0, 0, 1]), np.array([1, 0, 0]))  # the draw side of the axis and terrain
        if terrain_draw_side >= 0:  # if the up side of the terrain is visible
            self.workspace_terrain_color = self.up_side_terrain_color  # the color of the up side of the terrain
        else:  # if the down side of the terrain is visible
            self.workspace_terrain_color = self.down_side_terrain_color  # the color of the down side of the terrain
    def apply_robotic_manipulator_transformation(self, event = None):  # apply the transformation defined by the proper transfer and rotation matrices to the points of the robotic manipulator
        if self.robotic_manipulator_is_built:  # if a robotic manipulator is built
            # q_joints = self.get_robot_joints_variables(self.control_or_kinematics_variables_visualization)
            self.built_robotic_manipulator.q = self.get_robot_joints_variables(self.control_or_kinematics_variables_visualization)  # get the proper values for the joints variables (control or fkine)
            frames_origins, frames_orientations = self.get_all_frames_positions_orientations(self.built_robotic_manipulator.q)  # get the positions and the orientations of the frames of the robotic manipulator
            robotic_manipulator_points = [frames_origins[k].tolist() + [1.0] for k in range(len(frames_origins))]  # add the homogeneous coordinate to the origins of the frames of the robotic manipulator
            # add the points of the joints of the robotic manipulator
            for k in range(self.joints_number):  # for all the joints of the robotic manipulator
                robotic_manipulator_points.append((frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]).tolist() + [1.0])  # add the points of the joints of the robotic manipulator
            robotic_manipulator_points.append(frames_origins[0].tolist() + [1.0])  # add the origin of the base frame
            for k in range(self.joints_number):  # for all the joints of the robotic manipulator
                robotic_manipulator_points.append((frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]).tolist() + [1.0])  # add the points of the joints of the robotic manipulator
            robotic_manipulator_points.append(frames_origins[-1].tolist() + [1.0])  # add the origin of the end-effector frame
            return robotic_manipulator_points  # return the transformed points of the robotic manipulator
        else:  # if no robotic manipulator is built
            return [[0.0, 0.0, 0.0, 1.0] for k in range(self.robotic_manipulator_points_num)]  # return the default points of the robotic manipulator
    def apply_end_effector_transformation(self, end_effector_frame_points, event = None):  # apply the transformation defined by the proper transfer and rotation matrices to the points of the end-effector
        if self.robotic_manipulator_is_built:  # if a robotic manipulator is built
            q_joints = self.get_robot_joints_variables(self.control_or_kinematics_variables_visualization)  # get the proper values for the joints variables (control or fkine)
            end_effector_frame_origin, end_effector_frame_orientation = self.get_fkine_frame_position_orientation(q_joints, "end-effector")  # get the position and the orientation of the end-effector frame
            end_effector_T = np.eye(4)  # the transformation matrix of the end-effector frame
            end_effector_T[:3, :3] = end_effector_frame_orientation  # update the rotation matrix of the end-effector frame
            end_effector_T[:3, 3] = end_effector_frame_origin  # update the translation vector of the end-effector frame
            return (end_effector_T @ end_effector_frame_points.T).T  # return the transformed points of the end-effector
        else:  # if no robotic manipulator is built
            return (np.eye(4) @ end_effector_frame_points.T).T  # return the points of the end-effector
    def apply_obstacles_transformation(self, obstacles_points, event = None):  # apply the transformation defined by the proper transfer and rotation matrices to the points of the obstacles
        Tobs_total = self.obst_plane_wrt_world_transformation_matrix  # the total transformation matrix of the obstacles points
        return (Tobs_total @ obstacles_points.T).T  # return the transformed points of the obstacles that lie on the 2D plane
    def apply_camera_transformation(self, camera_points, event = None):  # apply the transformation defined by the proper transfer and rotation matrices to the points of the camera
        Tcam_total = self.camera_wrt_world_transformation_matrix  # the total transformation matrix of the camera points
        return (Tcam_total @ camera_points.T).T  # return the transformed points of the camera
    def reset_workspace_canvas(self, event = None):  # reset the workspace to its initial state
        self.scale_parameter = 1.0  # initialize the scale parameter of the workspace
        self.y_cor_workspace_origin = 0.0; self.z_cor_workspace_origin = 0.0  # initialize the coordinates of the origin of the workspace
        self.rot_y_workspace = 0.0; self.rot_z_workspace = 0.0  # initialize the rotation angles of the workspace
        self.last_transfer_y = 0.0; self.last_transfer_z = 0.0  # initialize the coordinates of the last mouse position when the user starts to transfer the workspace
        self.last_rotation_y = 0.0; self.last_rotation_z = 0.0  # initialize the coordinates of the last mouse position when the user starts to rotate the workspace
    def reset_workspace_canvas_2(self, event = None):  # reset the workspace to its initial state
        self.y_cor_workspace_origin = 0; self.z_cor_workspace_origin = 0  # initialize the coordinates of the origin of the workspace
    def transfer_workspace_start(self, event):  # initialize the coordinates of the last mouse position when the user starts to transfer the workspace
        self.last_transfer_y = event.x
        self.last_transfer_z = event.y
    def transfer_workspace(self, event):  # transfer the workspace according to the mouse movement
        self.y_cor_workspace_origin = self.y_cor_workspace_origin + 2 * self.workspace_control_sensitivity * (event.x - self.last_transfer_y)
        self.z_cor_workspace_origin = self.z_cor_workspace_origin - 2 * self.workspace_control_sensitivity * (event.y - self.last_transfer_z)
        self.last_transfer_y = event.x
        self.last_transfer_z = event.y
    def scale_workspace(self, event):  # scale the workspace according to the mouse wheel movement
        if event.delta == -120 and self.scale_parameter >= 0.2:
            self.scale_parameter -= self.workspace_control_sensitivity / 5
        elif event.delta == 120 and self.scale_parameter <= 15:
            self.scale_parameter += self.workspace_control_sensitivity / 5
    def rotate_workspace_start(self, event):  # initialize the coordinates of the last mouse position when the user starts to rotate the workspace
        self.last_rotation_y = event.y
        self.last_rotation_z = event.x
    def rotate_workspace(self, event):  # rotate the workspace according to the mouse movement
        self.rot_y_workspace = self.rot_y_workspace + self.workspace_control_sensitivity / 2 * (event.y - self.last_rotation_y)
        self.rot_z_workspace = self.rot_z_workspace + self.workspace_control_sensitivity / 2 * (event.x - self.last_rotation_z)
        self.last_rotation_y = event.y
        self.last_rotation_z = event.x
    def set_canvas_view(self, event = None):  # set the canvas view based on the x, y and z axis defined views
        x_axis_view = [0, -1, 1][["0", "-", "+"].index(self.x_axis_view)]
        y_axis_view = [0, -1, 1][["0", "-", "+"].index(self.y_axis_view)]
        z_axis_view = [0, -1, 1][["0", "-", "+"].index(self.z_axis_view)]
        self.rot_y_workspace = z_axis_view * [45, 90][[False, True].index(x_axis_view == 0 and y_axis_view == 0)]
        if y_axis_view == 0: self.rot_z_workspace = [4, 0, 0][[-1, 0, 1].index(x_axis_view)] * 45
        else: self.rot_z_workspace = (x_axis_view - 2) * (y_axis_view) * 45
    
    # functions for the visualization of the workspace
    def create_initial_workspace_canvas_points_connections(self, event = None):  # create the points and edges of the workspace (axis and the robotic manipulator)
        # define all the points of the workspace
        # create the axis and terrain points and edges
        self.axis_terrain_points = []  # initialize the points of the axis and terrain
        self.axis_terrain_points.append([0, 0, 0, 1])
        self.axis_terrain_points.append([self.x_axis_range, 0, 0, 1])
        self.axis_terrain_points.append([0, self.y_axis_range, 0, 1])
        self.axis_terrain_points.append([0, 0, self.z_axis_range, 1])
        self.axis_terrain_points.append([0, 0, 0, 1])
        self.axis_terrain_points.append([self.x_axis_range, self.y_axis_range, 0, 1])
        self.axis_terrain_points.append([-self.x_axis_range, self.y_axis_range, 0, 1])
        self.axis_terrain_points.append([-self.x_axis_range, -self.y_axis_range, 0, 1])
        self.axis_terrain_points.append([self.x_axis_range, -self.y_axis_range, 0, 1])
        self.axis_terrain_points = np.array(self.axis_terrain_points, dtype = float)  # convert the points of the axis and terrain to a numpy array
        self.axis_terrain_points_num = len(self.axis_terrain_points)  # update the number of the axis and terrain points
        self.axis_terrain_edges = self.axis_terrain_points_num * [None]  # initialize the edges of the axis and terrain
        self.axis_terrain_edges[0] = [[1, self.workspace_axis_colors[0], self.workspace_axis_sizes], [2, self.workspace_axis_colors[1], self.workspace_axis_sizes], [3, self.workspace_axis_colors[2], self.workspace_axis_sizes]]
        # create the robotic manipulator points and edges
        self.robotic_manipulator_points = []  # initialize the points of the robotic manipulator
        frames_origins, frames_orientations = self.get_all_frames_positions_orientations(np.zeros(self.joints_number,))  # get the positions and the orientations of the frames of the robotic manipulator
        self.robotic_manipulator_points = [frames_origins[k].tolist() + [1.0] for k in range(self.frames_number)]  # add the homogeneous coordinate to the points of the robotic manipulator
        for k in range(self.joints_number):  # for all the joints of the robotic manipulator
            self.robotic_manipulator_points.append((frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]).tolist() + [1.0])  # add the points (homogeneous) of the joints of the robotic manipulator
        points_num_before_links = len(self.robotic_manipulator_points)  # the number of the points of the robotic manipulator
        robotic_manipulator_links_points = [np.array(frames_origins[0])] + [np.array(frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]) for k in range(self.joints_number)] + [np.array(frames_origins[-1])]  # the points of the links of the robotic manipulator
        for k in range(self.joints_number + 2):  # for all the joints of the robotic manipulator (plus the base and the end-effector frames)
            self.robotic_manipulator_points.append(robotic_manipulator_links_points[k].tolist() + [1.0])  # add the points (homogeneous) of the links of the robotic manipulator
        self.robotic_manipulator_points = np.array(self.robotic_manipulator_points, dtype = float)  # convert the points of the robotic manipulator to a numpy array
        self.robotic_manipulator_points_num = len(self.robotic_manipulator_points)  # update the number of the robotic manipulator points
        self.robotic_manipulator_edges = self.robotic_manipulator_points_num * [None]  # initialize the edges of the robotic manipulator
        for k in range(self.links_number):
            self.robotic_manipulator_edges[points_num_before_links + k] = [[points_num_before_links + k + 1, self.links_colors[k], self.links_sizes[k]]]
        # create the robotic manipulator end-effector frame points and edges
        frame_axis_length = 0.05  # the length of the frame axis
        frame_axis_size = 2  # the size of the frame axis
        self.end_effector_frame_points = []  # initialize the points of the end-effector
        self.end_effector_frame_points.append([0.0, 0.0, 0.0, 1.0])
        self.end_effector_frame_points.append([frame_axis_length, 0.0, 0.0, 1.0])
        self.end_effector_frame_points.append([0.0, frame_axis_length, 0.0, 1.0])
        self.end_effector_frame_points.append([0.0, 0.0, frame_axis_length, 1.0])
        self.end_effector_frame_points = np.array(self.end_effector_frame_points, dtype = float)  # convert the points of the end-effector to a numpy array
        self.end_effector_frame_points_num = len(self.end_effector_frame_points)  # update the number of the end-effector points
        self.end_effector_frame_edges = self.end_effector_frame_points_num * [None]  # initialize the edges of the end-effector
        self.end_effector_frame_edges[0] = [[1, self.workspace_axis_colors[0], frame_axis_size], [2, self.workspace_axis_colors[1], frame_axis_size], [3, self.workspace_axis_colors[2], frame_axis_size]]
    def draw_next_workspace_canvas_frame(self):  # draw the next frame of the workspace
        self.canvas_destroy_counter += 1  # increase the counter of the canvas destroy
        if self.canvas_destroy_counter % (5.0 * self.canvas_fps) == 0:
            self.create_workspace_canvas()  # recreate the workspace canvas
            self.canvas_destroy_counter = 0  # reset the counter of the canvas destroy
        self.create_initial_workspace_canvas_points_connections()  # create the points and edges of the workspace (axis and the robotic manipulator)
        self.apply_workspace_transformation()  # apply the transformation defined by the proper transfer, rotation and scale variables to all the points of the workspace
        self.workspace_canvas.delete("all")  # clear the workspace canvas
        # choose the points and edges to draw
        a = self.axis_terrain_points_num; b = self.robotic_manipulator_points_num; c = self.end_effector_frame_points_num
        self.canvas_moved_points = np.array(self.canvas_moved_points); self.canvas_moved_points = self.canvas_moved_points.tolist()  # convert the canvas moved points to a list
        self.axis_terrain_moved_points = self.canvas_moved_points[0:a]  # the moved points of the axis and terrain
        self.robotic_manipulator_moved_points = self.canvas_moved_points[a : a + b]  # the moved points of the robotic manipulator
        self.end_effector_frame_moved_points = self.canvas_moved_points[a + b : a + b + c]  # the moved points of the end-effector frame
        # prepare the points of the various objects to draw on the canvas
        # prepare the points of the axis and terrain object
        points_counter = 0  # initialize the count of the points of the workspace
        if self.axis_enable_visualization == "shown":  # if the axis is enabled to be visualized
            for k in range(3):  # iterate through the three axis
                self.axis_terrain_edges[0][k][1] = self.workspace_axis_colors[k]  # change the color of the axis connecting lines
            axis_terrain_moved_points_properties = [[self.axis_terrain_moved_points[k], "black", self.workspace_axis_sizes + 5] for k in range(len(self.axis_terrain_moved_points))]  # the points of the axis and terrain with their properties
            axis_terrain_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.axis_terrain_points_num  # update the count of the points of the workspace
        # prepare the points of the robotic manipulator object
        if self.robotic_manipulator_enable_visualization == "shown":  # if the robotic manipulator is enabled to be visualized
            robotic_manipulator_moved_points_properties = [[self.robotic_manipulator_moved_points[k], self.frames_origins_colors[k], self.frames_origins_sizes[k] + 3] for k in range(self.frames_number)]  # the points of the frames of the robotic manipulator with their properties
            robotic_manipulator_moved_points_properties += [[self.robotic_manipulator_moved_points[self.frames_number + k], self.frames_origins_colors[k + 1], self.frames_origins_sizes[k + 1] + 5] for k in range(self.joints_number)]  # the points of the joints of the robotic manipulator with their properties
            robotic_manipulator_moved_points_properties += [[self.robotic_manipulator_moved_points[self.joints_number + self.frames_number + k], "black", 3] for k in range(self.links_number + 1)]  # the points of the links of the robotic manipulator with their properties
            robotic_manipulator_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.robotic_manipulator_points_num  # update the count of the points of the workspace
        # prepare the points of the robotic manipulator end-effector object
        if self.end_effector_frame_enable_visualization == "shown":  # if the end-effector frame is enabled to be visualized
            end_effector_frame_moved_points_properties = [[self.end_effector_frame_moved_points[k], "black", 5] for k in range(len(self.end_effector_frame_moved_points))]  # the points of the end-effector frame with their properties
            end_effector_frame_moved_points_properties.append(points_counter)  # add the count of the points of the workspace
        points_counter += self.end_effector_frame_points_num  # update the count of the points of the workspace
        # draw the various objects on the workspace canvas
        # draw the terrain and the world axis objects
        if self.terrain_enable_visualization == "shown":  # if the axis and terrain are enabled to be visualized
            first_terrain_point = 5; self.workspace_canvas.create_polygon([self.axis_terrain_moved_points[first_terrain_point][0], self.axis_terrain_moved_points[first_terrain_point][1], self.axis_terrain_moved_points[first_terrain_point+1][0], self.axis_terrain_moved_points[first_terrain_point+1][1], \
                                                                            self.axis_terrain_moved_points[first_terrain_point+2][0], self.axis_terrain_moved_points[first_terrain_point+2][1], self.axis_terrain_moved_points[first_terrain_point+3][0], self.axis_terrain_moved_points[first_terrain_point+3][1]], width = 1, fill = self.workspace_terrain_color, outline = "black")  # draw the terrain plane
        if self.axis_enable_visualization == "shown":  # if the axis is enabled to be visualized
            self.draw_workspace_object(axis_terrain_moved_points_properties, self.axis_terrain_edges, True, True)  # draw the axis and terrain objects
            self.workspace_canvas.create_text(self.axis_terrain_moved_points[0][0]-15, self.axis_terrain_moved_points[0][1], text = "O", font = "Calibri 15 bold", fill = "black")  # draw the letter "O" on the axis origin
            self.workspace_canvas.create_text(self.axis_terrain_moved_points[1][0]-15, self.axis_terrain_moved_points[1][1], text = "x", font = "Calibri 15 bold", fill = self.workspace_axis_colors[0])  # draw the letter "x" on the x axis
            self.workspace_canvas.create_text(self.axis_terrain_moved_points[2][0]+15, self.axis_terrain_moved_points[2][1], text = "y", font = "Calibri 15 bold", fill = self.workspace_axis_colors[1])  # draw the letter "y" on the y axis
            self.workspace_canvas.create_text(self.axis_terrain_moved_points[3][0]+15, self.axis_terrain_moved_points[3][1], text = "z", font = "Calibri 15 bold", fill = self.workspace_axis_colors[2])  # draw the letter "z" on the z axis
        # draw the robotic manipulator and the obstacles in the right order
        if self.robotic_manipulator_enable_visualization == "shown":
            self.draw_workspace_object(robotic_manipulator_moved_points_properties, self.robotic_manipulator_edges, self.robot_joints_enable_visualization == "shown", self.robot_links_enable_visualization == "shown")
        if self.end_effector_frame_enable_visualization == "shown":
            self.draw_workspace_object(end_effector_frame_moved_points_properties, self.end_effector_frame_edges, self.robotic_manipulator_enable_visualization == "shown", self.robotic_manipulator_enable_visualization == "shown")
        # write the names of the current and the pending robotic manipulator models on the workspace canvas
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            self.workspace_canvas.create_text(self.workspace_canvas_width / 2, 20, text = f"Current robotic manipulator:   {self.built_robotic_manipulator_info['name']}", font = "Calibri 12 bold", fill = "black")  # write the name of the built robotic manipulator model on the workspace canvas
            if self.robotic_manipulator_model_name != "" and self.robotic_manipulator_model_name != self.built_robotic_manipulator_info["name"]:
                self.workspace_canvas.create_text(self.workspace_canvas_width / 2, 40, text = f"Pending:   {self.robotic_manipulator_model_name}", font = "Calibri 12 bold", fill = "red")  # write the name of the pending robotic manipulator model on the workspace canvas
        else:
            if self.robotic_manipulator_model_name != "":
                self.workspace_canvas.create_text(self.workspace_canvas_width / 2, 20, text = f"Pending:   {self.robotic_manipulator_model_name}", font = "Calibri 12 bold", fill = "red")  # write the name of the pending robotic manipulator model on the workspace canvas
        # write the values of the visualized variables on the workspace canvas
        self.choose_visualized_variables_button.configure(text = self.control_or_kinematics_variables_visualization)  # update the text of the button that allows the user to choose the visualized variables
        joints_configuration_columns_indicator = 1  # the number of rows of the joints configuration indicator
        joints_configuration = ""  # the configuration of the joints of the robotic manipulator
        visualized_joints_values = [self.control_joints_variables, self.forward_kinematics_variables][[False, True].index(self.control_or_kinematics_variables_visualization == "kinematics")]  # the values of the visualized joints
        for k in range(self.joints_number):
            joints_configuration += f"{k + 1}" + [f"(°): {np.rad2deg(visualized_joints_values[k]):.1f}", f"(m): {visualized_joints_values[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])]  # the joints configuration indicator of the robotic manipulator
            if k <= self.joints_number - 1: joints_configuration += "   "
            if (k + 1) % joints_configuration_columns_indicator == 0 and (k + 1) != self.joints_number: joints_configuration += "\n"
        self.visualized_variables_values_indicator.configure(text = joints_configuration)  # update the text of the indicator that shows the values of the visualized variables
        # bind the points of the workspace to show their coordinates when the user's cursor is pointing to them
        for point in range(len(self.workspace_canvas_points)):
            self.workspace_canvas.tag_unbind(f"point{point}", "<Enter>")
            self.workspace_canvas.tag_bind(f"point{point}", "<Enter>", self.show_point_coordinates_helper(point))
        # loop the visualization function
        self.workspace_canvas.after(int(1000.0 / self.canvas_fps), self.draw_next_workspace_canvas_frame)  # loop the visualization function
    def draw_workspace_object(self, points, edges, points_are_shown, edges_are_shown):  # draw an object in the workspace
        # draw the edges (connecting lines) between every point and its chosen neighbours
        if edges_are_shown:  # if the edges are shown
            for start_point in range(len(edges)):
                if edges[start_point] != None:
                    for [end_point, line_color, line_width] in edges[start_point]:
                        try: self.workspace_canvas.create_line(points[start_point][0][0], points[start_point][0][1], points[end_point][0][0], points[end_point][0][1], width = line_width, fill = line_color, activefill = "white")
                        except: pass
        # draw the points
        counter = points[-1]  # the count of the points of the workspace
        points = points[:-1]
        if points_are_shown:  # if the points are shown
            for k in range(len(points)):
                if points[k][0] != None:
                    try: self.workspace_canvas.create_line(points[k][0][0], points[k][0][1], points[k][0][0], points[k][0][1], width = points[k][2], fill = points[k][1], capstyle = "round", activefill = "white", tags = f"point{counter + k}")
                    except: pass
    def show_point_coordinates_helper(self, point):  # helper function that returns the function that shows the coordinates of the point the user's cursor is pointing to
        return lambda event: self.show_point_coordinates(point, event)
    def show_point_coordinates(self, point, event = None):  # show the coordinates of the point the user's cursor is pointing to
        self.pointing_to_point = f"({self.workspace_canvas_points[point][0]:.3f}, {self.workspace_canvas_points[point][1]:.3f}, {self.workspace_canvas_points[point][2]:.3f})"
        self.workspace_canvas.create_text(self.workspace_canvas_width / 2, self.workspace_canvas_height - 20, text = f"Pointing to (m): {self.pointing_to_point}", font = "Calibri 12 bold", fill = "black")

    # functions for the creation of the menus
    def clear_menus_background(self, event = None):  # clear the menus background
        try: self.menus_background.destroy();   # destroy the previous sub menus of the current main menu
        except: pass
        self.menus_background = tk.Frame(self.menus_area, width = self.menus_background_width, height = self.menus_background_height, bg = self.menus_area_color, bd = self.workspace_menus_bd_width, relief = "groove")  # recreate the menus background
        self.menus_background.grid(row = 1, column = 0, sticky = tk.NSEW)  # place the menus background in the menus area
    def change_main_menu(self, main_menu_num, event = None):  # change the main menu
        self.main_menu_choice = main_menu_num % len(self.main_menus_build_details)  # change the main menu choice
        if self.main_menu_choice != 0:  # if the main menu number is not 0
            self.reload_built_robotic_manipulator_info_data()  # reload the data of the built robotic manipulator model, if any
        if self.main_menu_choice == len(self.main_menus_build_details)-1:  # if the main menu is the last main menu
            self.obst_avoid_solver_menu_is_enabled = True  # set the proper flag to True
        else:
            self.obst_avoid_solver_menu_is_enabled = False  # set the proper flag to False
        new_main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title']  # the title of the new main menu
        self.main_menu_label.configure(text = new_main_menu_title + f" ({self.main_menu_choice+1}/{len(self.main_menus_build_details)})")  # change the main menu title
        self.main_menus_build_details[self.main_menu_choice]['build_function'](self.main_menus_build_details[self.main_menu_choice]['submenus_titles'], self.main_menus_build_details[self.main_menu_choice]['submenus_descriptions'])  # build the sub menus for the new main menu
    def create_static_menu_frame(self, menu_properties, event = None):
        menu = tk.Frame(self.menus_background, width = menu_properties['width'], height = menu_properties['height'], bg = menu_properties['bg_color'], highlightbackground = "red", highlightcolor = "red", highlightthickness = 2)
        menu.grid(row = menu_properties['row'], column = menu_properties['column'], sticky = tk.NS)
        return menu  # return the static menu frame created
    def build_construct_robotic_manipulator_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus for the main menu that constructs the robotic manipulator
        self.clear_menus_background()  # clear the menus background
        # create the define parameters sub menu
        parameters_menu_title = submenus_titles[0]
        parameters_menu_info = f"--- {parameters_menu_title} ---\n" + submenus_descriptions[0]
        parameters_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = parameters_menu_title, info = parameters_menu_info, row = 0, column = 0, width = self.menus_background_width / 2, height = self.menus_background_height, \
                                            rows = 21, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # create the adjust visualization sub menu
        visualization_menu_title = submenus_titles[1]
        visualization_menu_info = f"--- {visualization_menu_title} ---\n" + submenus_descriptions[1]
        visualization_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = visualization_menu_title, info = visualization_menu_info, row = 0, column = 1, width = self.menus_background_width / 2, height = self.menus_background_height, \
                                                rows = 18, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_define_parameters_menu(self.create_static_menu_frame(parameters_menu_properties), parameters_menu_properties)
        self.generate_adjust_visualization_menu(self.create_static_menu_frame(visualization_menu_properties), visualization_menu_properties)
    def generate_define_parameters_menu(self, menu_frame, menu_properties, event = None):  # build the define parameters menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        model_name_label_ord = 2
        joints_number_model_label_ord = 3
        den_har_parameters_label_ord = joints_number_model_label_ord+1
        choose_joint_number_model_label_ord = den_har_parameters_label_ord+1
        choose_joint_type_label_ord = choose_joint_number_model_label_ord+1
        a_parameter_label_ord = choose_joint_type_label_ord+1
        alpha_parameter_label_ord = a_parameter_label_ord+1
        d_parameter_label_ord = alpha_parameter_label_ord+1
        theta_parameter_label_ord = d_parameter_label_ord+1
        var_limits_label_ord = theta_parameter_label_ord+1.4
        var_min_limit_label_ord = var_limits_label_ord-0.4
        var_max_limit_label_ord = var_limits_label_ord+0.4
        base_end_effector_label_ord = var_limits_label_ord+1.4
        base_label_ord = base_end_effector_label_ord+1.4
        base_pos_button_ord = base_label_ord-0.4
        base_orient_button_ord = base_label_ord+0.4
        end_effector_label_ord = base_label_ord+1.6
        end_effector_pos_button_ord = end_effector_label_ord-0.4
        end_effector_orient_button_ord = end_effector_label_ord+0.4
        robot_control_operations_label_ord = end_effector_label_ord+1.4
        build_robot_button_ord = robot_control_operations_label_ord+1.0
        destroy_robot_button_ord = build_robot_button_ord+0.8
        show_model_info_button_ord = build_robot_button_ord+0.4
        save_model_button_ord = build_robot_button_ord+1.8
        delete_model_button_ord = save_model_button_ord+1.0
        load_model_label_ord = save_model_button_ord
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        model_name_label_x = 1/4; gbl.menu_label(menu_frame, "Model name:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], model_name_label_x * menu_properties['width'], model_name_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.model_name_entrybox = ttk.Entry(menu_frame, font = f"Calibri {menu_properties['options_font']}", width = 16, justify = "center")
        model_name_entrybox_x = 2/3; self.model_name_entrybox.place(x = model_name_entrybox_x * menu_properties['width'], y = model_name_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.model_name_entrybox.insert(0, self.robotic_manipulator_model_name)
        self.model_name_entrybox.bind("<Return>", self.change_robotic_manipulator_model_name)
        joints_number_model_label_x = 1/3; gbl.menu_label(menu_frame, "Joints number (n):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_number_model_label_x * menu_properties['width'], joints_number_model_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_number_button_ord = joints_number_model_label_ord; joints_number_button_x = 2/3; self.joints_number_button = gbl.menu_button(menu_frame, self.joints_number, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], joints_number_button_x * menu_properties['width'], joints_number_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_joints_number).button
        den_har_parameters_label_x = 1/2; gbl.menu_label(menu_frame, "Standard Denavit - Hartenberg parameters:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], den_har_parameters_label_x * menu_properties['width'], den_har_parameters_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_joint_number_model_label_x = 1/3; gbl.menu_label(menu_frame, "Joint number:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_number_model_label_x * menu_properties['width'], choose_joint_number_model_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_number_model_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = [f"joint {joint}" for joint in range(1, self.joints_number + 1)], justify = "center")
        choose_joint_number_model_combobox_x = 3/4; self.choose_joint_number_model_combobox.place(x = choose_joint_number_model_combobox_x * menu_properties['width'], y = choose_joint_number_model_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_number_model_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_model)
        choose_joint_type_label_x = 1/3; gbl.menu_label(menu_frame, "Joint type:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_type_label_x * menu_properties['width'], choose_joint_type_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.joints_types_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = self.joints_types_list, justify = "center")
        joints_types_combobox_x = 3/4; self.joints_types_combobox.place(x = joints_types_combobox_x * menu_properties['width'], y = choose_joint_type_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.joints_types_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_type)
        a_den_har_parameter_label_x = 1/3; gbl.menu_label(menu_frame, "a (meters):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], a_den_har_parameter_label_x * menu_properties['width'], a_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        a_den_har_parameter_button_ord = a_parameter_label_ord; a_den_har_parameter_button_x = 3/4; self.a_den_har_parameter_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], a_den_har_parameter_button_x * menu_properties['width'], a_den_har_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_a_den_har_parameter).button
        alpha_den_har_parameter_label_x = 1/3; gbl.menu_label(menu_frame, "alpha (degrees):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], alpha_den_har_parameter_label_x * menu_properties['width'], alpha_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        alpha_den_har_parameter_button_ord = alpha_parameter_label_ord; alpha_den_har_parameter_button_x = 3/4; self.alpha_den_har_parameter_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], alpha_den_har_parameter_button_x * menu_properties['width'], alpha_den_har_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_alpha_den_har_parameter).button
        d_den_har_parameter_label_x = 1/3; gbl.menu_label(menu_frame, "d (meters):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], d_den_har_parameter_label_x * menu_properties['width'], d_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        d_den_har_parameter_button_ord = d_parameter_label_ord; d_den_har_parameter_button_x = 3/4; self.d_den_har_parameter_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], d_den_har_parameter_button_x * menu_properties['width'], d_den_har_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_d_den_har_parameter).button
        theta_den_har_parameter_label_x = 1/3; gbl.menu_label(menu_frame, "theta (degrees):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], theta_den_har_parameter_label_x * menu_properties['width'], theta_parameter_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        theta_den_har_parameter_button_ord = theta_parameter_label_ord; theta_den_har_parameter_button_x = 3/4; self.theta_den_har_parameter_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], theta_den_har_parameter_button_x * menu_properties['width'], theta_den_har_parameter_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_theta_den_har_parameter).button
        var_limits_label_x = 1/3; self.var_limits_label = gbl.menu_label(menu_frame, "variable limits:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], var_limits_label_x * menu_properties['width'], var_limits_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        var_min_limit_label_x = 3/5; gbl.menu_label(menu_frame, "min:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], var_min_limit_label_x * menu_properties['width'], var_min_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        var_min_limit_button_x = 4/5; self.var_min_limit_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], var_min_limit_button_x * menu_properties['width'], var_min_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_joint_variable_min_limit).button
        var_max_limit_label_x = 3/5; gbl.menu_label(menu_frame, "max:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], var_max_limit_label_x * menu_properties['width'], var_max_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        var_max_limit_button_x = 4/5; self.var_max_limit_button = gbl.menu_button(menu_frame, "0.0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], var_max_limit_button_x * menu_properties['width'], var_max_limit_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_joint_variable_max_limit).button
        base_end_effector_label_x = 1/2; gbl.menu_label(menu_frame, "Base and end-effector systems:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], base_end_effector_label_x * menu_properties['width'], base_end_effector_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        base_label_x = 1/3; gbl.menu_label(menu_frame, "World ⭢ Base ⭢\nFrame \"0\"", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], base_label_x * menu_properties['width'], base_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        base_pos_button_x = 2/3; self.base_pos_button = gbl.menu_button(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], base_pos_button_x * menu_properties['width'], base_pos_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_base_zero_frame_positions).button
        base_orient_button_x = 2/3; self.base_orient_button = gbl.menu_button(menu_frame, "orientation", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], base_orient_button_x * menu_properties['width'], base_orient_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_base_zero_frame_orientations).button
        end_effector_label_x = 1/3; gbl.menu_label(menu_frame, "Frame \"n\" ⭢\nEnd-effector:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_label_x * menu_properties['width'], end_effector_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_pos_button_x = 2/3; self.end_effector_pos_button = gbl.menu_button(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], end_effector_pos_button_x * menu_properties['width'], end_effector_pos_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_end_effector_position).button
        end_effector_orient_button_x = 2/3; self.end_effector_orient_button = gbl.menu_button(menu_frame, "orientation", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], end_effector_orient_button_x * menu_properties['width'], end_effector_orient_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_end_effector_orientation).button
        robot_control_operations_label_x = 1/2; gbl.menu_label(menu_frame, "Robot control operations:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], robot_control_operations_label_x * menu_properties['width'], robot_control_operations_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_model_info_button_x = 4/16; self.show_model_info_button = gbl.menu_button(menu_frame, "show current\nrobot info", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_model_info_button_x * menu_properties['width'], show_model_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_robotic_manipulator_model_info).button
        save_model_button_ord_x = 4/16; self.save_robotic_manipulator_model_file_button = gbl.menu_button(menu_frame, "save robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], save_model_button_ord_x * menu_properties['width'], save_model_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.save_robotic_manipulator_model_file).button
        delete_model_button_x = 4/16; self.delete_robotic_amr_model_button = gbl.menu_button(menu_frame, "delete robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], delete_model_button_x * menu_properties['width'], delete_model_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.delete_robotic_manipulator_model_file).button
        build_robot_button_x = 11/16; self.build_robot_button = gbl.menu_button(menu_frame, "build robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], build_robot_button_x * menu_properties['width'], build_robot_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.build_robotic_manipulator_model).button
        destroy_robot_button_x = 11/16; self.destroy_robot_button = gbl.menu_button(menu_frame, "destroy robot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], destroy_robot_button_x * menu_properties['width'], destroy_robot_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.destroy_robotic_manipulator_model).button
        load_model_label_x = 11/16; gbl.menu_label(menu_frame, "Load robot model:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], load_model_label_x * menu_properties['width'], load_model_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.load_model_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 15, values = self.saved_robots_models_files_list, justify = "center")
        load_model_combobox_x = load_model_label_x; self.load_model_combobox.place(x = load_model_combobox_x * menu_properties['width'], y = (load_model_label_ord + 0.8) * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.load_model_combobox.bind("<<ComboboxSelected>>", lambda event: self.load_robotic_manipulator_model("", event))
        self.load_model_combobox.bind("<Return>", lambda event: self.load_robotic_manipulator_model("", event))
    def generate_adjust_visualization_menu(self, menu_frame, menu_properties, event = None):  # build the adjust visualization menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        robotic_manipulator_visualization_label_ord = 2.5
        choose_frame_visualization_label_ord = robotic_manipulator_visualization_label_ord+1
        choose_frame_color_label_ord = choose_frame_visualization_label_ord+1
        change_frame_color_button_ord = choose_frame_color_label_ord
        apply_color_all_frames_button_ord = choose_frame_color_label_ord
        choose_frame_size_label_ord = choose_frame_color_label_ord+1
        change_frame_size_button_ord = choose_frame_size_label_ord
        apply_size_all_frames_button_ord = choose_frame_size_label_ord
        choose_joint_position_label_ord = apply_size_all_frames_button_ord+1
        change_joint_position_button_ord = choose_joint_position_label_ord
        choose_link_number_visualization_label_ord = change_joint_position_button_ord+1
        choose_link_color_label_ord = choose_link_number_visualization_label_ord+1
        change_link_color_button_ord = choose_link_color_label_ord
        apply_color_all_links_button_ord = choose_link_color_label_ord
        choose_link_size_label_ord = choose_link_color_label_ord+1
        change_link_size_button_ord = choose_link_size_label_ord
        apply_size_all_links_button_ord = choose_link_size_label_ord
        link_length_label_ord = apply_size_all_links_button_ord+1
        link_length_indicator_ord = link_length_label_ord
        workspace_visualization_label_ord = link_length_indicator_ord+1.5
        canvas_visual_label_ord = workspace_visualization_label_ord+1
        canvas_color_button_ord = canvas_visual_label_ord
        terrain_visual_label_ord = canvas_visual_label_ord+1
        terrain_color_button_ord = terrain_visual_label_ord
        axis_visual_label_ord = terrain_visual_label_ord+1
        axis_color_button_ord = axis_visual_label_ord
        axis_size_button_ord = axis_visual_label_ord
        other_simulators_label_ord = axis_size_button_ord+1.5
        matplotlib_simulator_button_ord = other_simulators_label_ord+1
        swift_simulator_button_ord = matplotlib_simulator_button_ord
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        robotic_manipulator_visualization_label_x = 1/2; gbl.menu_label(menu_frame, "Robotic manipulator visualization:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], robotic_manipulator_visualization_label_x * menu_properties['width'], robotic_manipulator_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_frame_visualization_label_x = 1/3; gbl.menu_label(menu_frame, "Frame:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_frame_visualization_label_x * menu_properties['width'], choose_frame_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_frame_visualization_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 12, values = ["base"] + [f"frame {frame}" for frame in range(self.links_number)] + ["end-effector"], justify = "center")
        choose_frame_visualization_combobox_x = 2/3; self.choose_frame_visualization_combobox.place(x = choose_frame_visualization_combobox_x * menu_properties['width'], y = choose_frame_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_frame_visualization_combobox.bind("<<ComboboxSelected>>", self.change_chosen_frame_visualization)
        choose_joint_position_label_x = 1/3; gbl.menu_label(menu_frame, "Joint position (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_position_label_x * menu_properties['width'], choose_joint_position_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_joint_position_button_x = 2/3; self.change_joint_position_button = gbl.menu_button(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_joint_position_button_x * menu_properties['width'], change_joint_position_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_joint_position).button
        choose_frame_color_label_x = 1/4; gbl.menu_label(menu_frame, "Frame color:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_frame_color_label_x * menu_properties['width'], choose_frame_color_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_frame_color_button_x = 2/4; self.change_frame_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_frame_color_button_x * menu_properties['width'], change_frame_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_frame_color).button
        apply_color_all_frames_button_x = 3/4; self.apply_color_all_frames_button = gbl.menu_button(menu_frame, "all frames", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_color_all_frames_button_x * menu_properties['width'], apply_color_all_frames_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_color_to_all_frames).button
        choose_frame_size_label_x = 1/4; gbl.menu_label(menu_frame, "Frame size:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_frame_size_label_x * menu_properties['width'], choose_frame_size_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_frame_size_button_x = 2/4; self.change_frame_size_button = gbl.menu_button(menu_frame, "size", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_frame_size_button_x * menu_properties['width'], change_frame_size_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_frame_size).button
        apply_size_all_frames_button_x = 3/4; self.apply_size_all_frames_button = gbl.menu_button(menu_frame, "all frames", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_size_all_frames_button_x * menu_properties['width'], apply_size_all_frames_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_size_to_all_frames).button
        choose_link_number_visualization_label_x = 1/3; gbl.menu_label(menu_frame, "Link:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_link_number_visualization_label_x * menu_properties['width'], choose_link_number_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_link_visualization_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 12, values = ["base"] + [f"link {link}" for link in range(1, self.links_number)], justify = "center")
        choose_link_visualization_combobox_x = 2/3; self.choose_link_visualization_combobox.place(x = choose_link_visualization_combobox_x * menu_properties['width'], y = choose_link_number_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_link_visualization_combobox.bind("<<ComboboxSelected>>", self.change_chosen_link_visualization)
        link_length_label_x = 1/3; gbl.menu_label(menu_frame, "Link length (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], link_length_label_x * menu_properties['width'], link_length_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        link_length_indicator_x = 2/3; self.link_length_indicator = gbl.menu_label(menu_frame, "length", f"Calibri {menu_properties['options_font']} bold", "yellow", menu_properties['bg_color'], link_length_indicator_x * menu_properties['width'], link_length_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        choose_link_color_label_x = 1/4; gbl.menu_label(menu_frame, "Link color:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_link_color_label_x * menu_properties['width'], choose_link_color_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_link_color_button_x = 2/4; self.change_link_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_link_color_button_x * menu_properties['width'], change_link_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_link_color).button
        apply_color_all_links_button_x = 3/4; self.apply_color_all_links_button = gbl.menu_button(menu_frame, "all links", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_color_all_links_button_x * menu_properties['width'], apply_color_all_links_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_color_to_all_links).button
        choose_link_size_label_x = 1/4; gbl.menu_label(menu_frame, "Link size:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_link_size_label_x * menu_properties['width'], choose_link_size_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        change_link_size_button_x = 2/4; self.change_link_size_button = gbl.menu_button(menu_frame, "size", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], change_link_size_button_x * menu_properties['width'], change_link_size_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_link_size).button
        apply_size_all_links_button_x = 3/4; self.apply_size_all_links_button = gbl.menu_button(menu_frame, "all links", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], apply_size_all_links_button_x * menu_properties['width'], apply_size_all_links_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.apply_size_to_all_links).button
        workspace_visualization_label_x = 1/2; gbl.menu_label(menu_frame, "Workspace visualization:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], workspace_visualization_label_x * menu_properties['width'], workspace_visualization_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        canvas_visual_label_x = 1/3; gbl.menu_label(menu_frame, "Canvas:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], canvas_visual_label_x * menu_properties['width'], canvas_visual_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        canvas_color_button_x = 2/3; self.canvas_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], canvas_color_button_x * menu_properties['width'], canvas_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_canvas_color).button
        terrain_visual_label_x = 1/3; gbl.menu_label(menu_frame, "Terrain:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], terrain_visual_label_x * menu_properties['width'], terrain_visual_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        terrain_color_button_x = 2/3; self.terrain_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], terrain_color_button_x * menu_properties['width'], terrain_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_terrain_color).button
        axis_visual_label_x = 1/4; gbl.menu_label(menu_frame, "Axis:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], axis_visual_label_x * menu_properties['width'], axis_visual_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        axis_color_button_x = 2/4; self.axis_color_button = gbl.menu_button(menu_frame, "color", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], axis_color_button_x * menu_properties['width'], axis_color_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_axis_colors).button
        axis_size_button_x = 3/4; self.axis_size_button = gbl.menu_button(menu_frame, "size", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], axis_size_button_x * menu_properties['width'], axis_size_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_axis_size).button
        other_simulators_label_x = 1/2; gbl.menu_label(menu_frame, "Simulators:", f"Calibri {menu_properties['options_font']} bold", menu_properties['subtitles_color'], menu_properties['bg_color'], other_simulators_label_x * menu_properties['width'], other_simulators_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        matplotlib_simulator_button_x = 1/3; self.matplotlib_simulator_button = gbl.menu_button(menu_frame, "Matplotlib", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], matplotlib_simulator_button_x * menu_properties['width'], matplotlib_simulator_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.open_matplotlib_simulator).button
        swift_simulator_button_x = 2/3; self.swift_simulator_button = gbl.menu_button(menu_frame, "Swift", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], swift_simulator_button_x * menu_properties['width'], swift_simulator_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.open_close_swift_simulator).button
        # create an option for the item appearing as the end-effector of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def build_robotic_manipulator_forward_kinematics_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus of the main menu that analyzes the forward kinematics (and inverse kinematics) of the robotic manipulator
        self.clear_menus_background()  # clear the menus background
        # create the forward kinematics sub menu
        fkine_menu_title = submenus_titles[0]
        fkine_menu_info = f"--- {fkine_menu_title} ---\n" + submenus_descriptions[0]
        fkine_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = fkine_menu_title, info = fkine_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 11 / 20, 
                                        rows = 12, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # create the inverse kinematics sub menu
        invkine_menu_title = submenus_titles[1]
        invkine_menu_info = f"--- {invkine_menu_title} ---\n" + submenus_descriptions[1]
        invkine_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = invkine_menu_title, info = invkine_menu_info, row = 2, column = 0, width = self.menus_background_width, height = self.menus_background_height * 9 / 20, \
                                        rows = 9, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_forward_kinematics_menu(self.create_static_menu_frame(fkine_menu_properties), fkine_menu_properties)
        self.generate_inverse_kinematics_menu(self.create_static_menu_frame(invkine_menu_properties), invkine_menu_properties)
    def generate_forward_kinematics_menu(self, menu_frame, menu_properties, event = None):  # build the forward kinematics menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        choose_joint_number_fkine_label_ord = 2.5
        choose_joint_number_fkine_combobox_ord = choose_joint_number_fkine_label_ord
        choose_fkine_variable_value_label_ord = choose_joint_number_fkine_label_ord-0.4
        fkine_value_unit_indicator_ord = choose_joint_number_fkine_label_ord+0.4
        joint_fkine_value_combobox_ord = choose_joint_number_fkine_label_ord
        get_control_values_button_ord = choose_joint_number_fkine_label_ord
        choose_frame_label_ord = choose_joint_number_fkine_label_ord+6.5
        choose_frame_combobox_ord = choose_frame_label_ord+0.8
        frame_position_label_ord = choose_frame_label_ord
        frame_position_indicator_ord = frame_position_label_ord+0.8
        frame_orientation_label_ord = choose_frame_label_ord
        frame_orientation_representation_combobox_ord = frame_orientation_label_ord+0.8
        frame_orientation_indicator_ord = choose_frame_label_ord+0.5
        find_reachable_workspace_label_ord = choose_frame_label_ord+2.5
        joints_range_divisions_label_ord = find_reachable_workspace_label_ord
        joints_range_divisions_button_ord = find_reachable_workspace_label_ord
        compute_reachable_workspace_button_ord = find_reachable_workspace_label_ord
        show_fkine_info_button_ord = menu_properties['rows']-1.0
        fkine_variables_sliders_rows = 3; fkine_variables_sliders_ord_list = []
        for k in range(self.joints_number):
            fkine_variables_sliders_ord_list.append(choose_joint_number_fkine_label_ord+1.7 + 1.5*(k % fkine_variables_sliders_rows))
            # fkine_variables_sliders_ord_list.append(choose_joint_number_fkine_label_ord+2.5)
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        choose_joint_number_fkine_label_x = 1/6; gbl.menu_label(menu_frame, "Joint\nnumber:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_number_fkine_label_x * menu_properties['width'], choose_joint_number_fkine_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_number_fkine_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = [f"joint {joint}" for joint in range(1, self.joints_number + 1)], justify = "center")
        choose_joint_number_fkine_combobox_x = 2/6; self.choose_joint_number_fkine_combobox.place(x = choose_joint_number_fkine_combobox_x * menu_properties['width'], y = choose_joint_number_fkine_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_number_fkine_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_fkine)
        choose_fkine_variable_value_label_x = 3/6; gbl.menu_label(menu_frame, "Joint value:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_fkine_variable_value_label_x * menu_properties['width'], choose_fkine_variable_value_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        fkine_value_unit_indicator_x = 3/6; self.fkine_value_unit_indicator = gbl.menu_label(menu_frame, self.joints_types[self.chosen_joint_number_fkine - 1] + " " + ["(°)", "(m)"][self.joints_types_list.index(self.joints_types[self.chosen_joint_number_fkine - 1])], f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], fkine_value_unit_indicator_x * menu_properties['width'], fkine_value_unit_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        self.joint_fkine_value_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 8, values = "", justify = "center")
        joint_fkine_value_combobox_x = 4/6; self.joint_fkine_value_combobox.place(x = joint_fkine_value_combobox_x * menu_properties['width'], y = joint_fkine_value_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.joint_fkine_value_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_fkine_2)
        self.joint_fkine_value_combobox.bind("<Return>", self.change_chosen_fkine_variable)
        get_control_values_button_x = 5/6; self.get_control_values_button = gbl.menu_button(menu_frame, "get control\nvalues", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], get_control_values_button_x * menu_properties['width'], get_control_values_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.copy_control_to_fkine_values).button
        choose_frame_label_x = 1/7; gbl.menu_label(menu_frame, "Frame:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_frame_label_x * menu_properties['width'], choose_frame_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_frame_fkine_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 12, values = ["base"] + [f"frame {frame}" for frame in range(self.links_number)] + ["end-effector"], justify = "center")
        choose_frame_combobox_x = 1/7; self.choose_frame_fkine_combobox.place(x = choose_frame_combobox_x * menu_properties['width'], y = choose_frame_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_frame_fkine_combobox.bind("<<ComboboxSelected>>", self.change_chosen_frame_fkine)
        frame_position_label_x = 2/6; gbl.menu_label(menu_frame, "Position (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], frame_position_label_x * menu_properties['width'], frame_position_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        frame_position_indicator_x = 2/6; self.frame_position_indicator = gbl.menu_label(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], frame_position_indicator_x * menu_properties['width'], frame_position_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        frame_orientation_label_x = 9/16; gbl.menu_label(menu_frame, "Orientation:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], frame_orientation_label_x * menu_properties['width'], frame_orientation_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.frame_orientation_representation_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 12, values = self.orientation_representations_fkine_list, justify = "center")
        frame_orientation_representation_combobox_x = 9/16; self.frame_orientation_representation_combobox.place(x = frame_orientation_representation_combobox_x * menu_properties['width'], y = frame_orientation_representation_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.frame_orientation_representation_combobox.bind("<<ComboboxSelected>>", self.change_orientation_representation_fkine)
        frame_orientation_indicator_x = 31/40; self.frame_orientation_indicator = gbl.menu_label(menu_frame, "orientation", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], frame_orientation_indicator_x * menu_properties['width'], frame_orientation_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        find_reachable_workspace_label_x = 1/5; gbl.menu_label(menu_frame, "Find reachable workspace:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], find_reachable_workspace_label_x * menu_properties['width'], find_reachable_workspace_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_range_divisions_label_x = 4/9; gbl.menu_label(menu_frame, "Joints range\ndivisions:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_range_divisions_label_x * menu_properties['width'], joints_range_divisions_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_range_divisions_button_x = 5/9; self.joints_range_divisions_button = gbl.menu_button(menu_frame, "set\ndivs", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], joints_range_divisions_button_x * menu_properties['width'], joints_range_divisions_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.set_joints_range_divisions).button
        compute_reachable_workspace_button_x = 3/4; self.compute_reachable_workspace_button = gbl.menu_button(menu_frame, "compute and plot", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], compute_reachable_workspace_button_x * menu_properties['width'], compute_reachable_workspace_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.compute_plot_reachable_workspace).button
        # show_fkine_info_button_x = 19/20; self.show_fkine_info_button = gbl.menu_button(menu_frame, "🕮", f"Calibri {menu_properties['options_font']+5} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_fkine_info_button_x * menu_properties['width'], show_fkine_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_fkine_info).button
        self.fkine_variables_sliders = []
        for k in range(self.joints_number):
            joint_type_index = self.joints_types_list.index(self.joints_types[k])  # the index of the current joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
            fkine_variables_limits = self.control_joints_variables_limits[k][joint_type_index]  # the control variable limits of the current joint
            self.fkine_variables_sliders.append(tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font'] - 3}", orient = tk.HORIZONTAL, from_ = fkine_variables_limits[0], to = fkine_variables_limits[1], resolution = [10**(-self.angles_precision), 10**(-self.distances_precision)][joint_type_index], length = menu_properties['width'] / 7.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = lambda event, slider_num = k: self.change_fkine_variable_slider(slider_num, event)))
            # fkine_variables_sliders_x = 1/10 * (k + 1); self.fkine_variables_sliders.append(tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font'] - 3}", orient = tk.VERTICAL, length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = lambda event, slider_num = k: self.change_fkine_variable_slider(slider_num, event)))
            fkine_variables_sliders_x = 1/(np.ceil(self.joints_number / fkine_variables_sliders_rows) + 1) * (k // fkine_variables_sliders_rows + 1); self.fkine_variables_sliders[k].place(x = fkine_variables_sliders_x * menu_properties['width'], y = fkine_variables_sliders_ord_list[k] * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
            fkine_variables_labels_x = 1/(np.ceil(self.joints_number / fkine_variables_sliders_rows) + 1) * (k // fkine_variables_sliders_rows + 1) - 2/3*1/7; gbl.menu_label(menu_frame, f"{k + 1}\n{['(°)', '(m)'][joint_type_index]}:", f"Calibri {menu_properties['options_font']-3} bold", menu_properties['labels_color'], menu_properties['bg_color'], fkine_variables_labels_x * menu_properties['width'], fkine_variables_sliders_ord_list[k] * menu_properties['height'] / (menu_properties['rows'] + 1) - 0.8)
        self.update_forward_kinematics_indicators()  # update the indicators of the forward kinematics of the robotic manipulator
    def generate_inverse_kinematics_menu(self, menu_frame, menu_properties, event = None):  # build the inverse kinematics menus
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        get_fkine_result_button_ord = 2.5
        send_invkine_result_button_ord = get_fkine_result_button_ord
        choose_end_effector_position_label = get_fkine_result_button_ord+1.8
        choose_end_effector_position_button = choose_end_effector_position_label
        choose_end_effector_orientation_label = choose_end_effector_position_label
        choose_end_effector_orientation_button = choose_end_effector_position_label
        choose_invkine_tolerance_label_ord = choose_end_effector_position_label+1.8
        choose_invkine_tolerance_button_ord = choose_invkine_tolerance_label_ord
        joints_configuration_label_ord = choose_end_effector_position_label+3.5
        joints_configuration_indicator_ord = joints_configuration_label_ord
        show_invkine_info_button_ord = menu_properties['rows']-1.0
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        get_fkine_result_button_x = 1/3; self.get_fkine_result_button = gbl.menu_button(menu_frame, "↓ get forward kinematics\nanalysis result", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], get_fkine_result_button_x * menu_properties['width'], get_fkine_result_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.get_fkine_pose_result).button
        send_invkine_result_button_x = 2/3; self.send_invkine_result_button = gbl.menu_button(menu_frame, "↑ send inverse kinematics\nanalysis result", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_invkine_result_button_x * menu_properties['width'], send_invkine_result_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_invkine_joints_config_result).button
        choose_end_effector_position_label_x = 1/5; gbl.menu_label(menu_frame, "End-effector\nposition (m):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_end_effector_position_label_x * menu_properties['width'], choose_end_effector_position_label * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_end_effector_position_button_x = 2/5; self.choose_end_effector_position_button = gbl.menu_button(menu_frame, "position", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_end_effector_position_button_x * menu_properties['width'], choose_end_effector_position_button * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_end_effector_position_invkine).button
        choose_end_effector_orientation_label_x = 3/5; gbl.menu_label(menu_frame, "End-effector\norientation (°):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_end_effector_orientation_label_x * menu_properties['width'], choose_end_effector_orientation_label * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_end_effector_orientation_button_x = 4/5; self.choose_end_effector_orientation_button = gbl.menu_button(menu_frame, "orientation", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_end_effector_orientation_button_x * menu_properties['width'], choose_end_effector_orientation_button * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_end_effector_orientation_invkine).button
        choose_invkine_tolerance_label_x = 2/5; gbl.menu_label(menu_frame, "Numerical solver maximum allowed error (tolerance):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_invkine_tolerance_label_x * menu_properties['width'], choose_invkine_tolerance_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_invkine_tolerance_button_x = 3/4; self.choose_invkine_tolerance_button = gbl.menu_button(menu_frame, self.invkine_tolerance, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_invkine_tolerance_button_x * menu_properties['width'], choose_invkine_tolerance_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_invkine_tolerance).button
        joints_configuration_label_x = 1/6; gbl.menu_label(menu_frame, "Joints\nconfiguration:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_configuration_label_x * menu_properties['width'], joints_configuration_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_configuration_indicator_x = 3/5; self.joints_configuration_indicator = gbl.menu_label(menu_frame, "joints configuration", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], joints_configuration_indicator_x * menu_properties['width'], joints_configuration_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        # show_invkine_info_button_x = 19/20; self.show_invkine_info_button = gbl.menu_button(menu_frame, "🕮", f"Calibri {menu_properties['options_font']+5} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_invkine_info_button_x * menu_properties['width'], show_invkine_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_invkine_info).button
        self.update_inverse_kinematics_indicators()  # update the indicators of the inverse kinematics of the robotic manipulator
    def build_robotic_manipulator_differential_kinematics_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus of the main menu that analyzes the differential kinematics of the robotic manipulator
        self.clear_menus_background()  # clear the menus background
        # create the differential kinematics sub menu
        diffkine_menu_title = submenus_titles[0]
        diffkine_menu_info = f"--- {diffkine_menu_title} ---\n" + submenus_descriptions[0]
        diffkine_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = diffkine_menu_title, info = diffkine_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 2, \
                                        rows = 10, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # create the inverse differential kinematics sub menu
        invdiffkine_menu_title = submenus_titles[1]
        invdiffkine_menu_info = f"--- {invdiffkine_menu_title} ---\n" + submenus_descriptions[1]
        invdiffkine_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = invdiffkine_menu_title, info = invdiffkine_menu_info, row = 2, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 2, \
                                            rows = 9, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
        # generate the sub menus
        self.generate_differential_kinematics_menu(self.create_static_menu_frame(diffkine_menu_properties), diffkine_menu_properties)
        self.generate_inverse_differential_kinematics_menu(self.create_static_menu_frame(invdiffkine_menu_properties), invdiffkine_menu_properties)
    def generate_differential_kinematics_menu(self, menu_frame, menu_properties, event = None):  # build the differential kinematics menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        choose_joint_number_diffkine_label_ord = 2.5
        joints_configuration_diffkine_label_ord = choose_joint_number_diffkine_label_ord-0.4
        joints_configuration_diffkine_indicator_ord = joints_configuration_diffkine_label_ord+0.8
        choose_joint_number_diffkine_combobox_ord = choose_joint_number_diffkine_label_ord
        choose_diffkine_velocity_label_ord = choose_joint_number_diffkine_label_ord-0.3
        diffkine_value_unit_indicator_ord = choose_joint_number_diffkine_label_ord+0.3
        joint_diffkine_velocity_combobox_ord = choose_joint_number_diffkine_label_ord
        end_effector_linear_vel_label_ord = choose_joint_number_diffkine_label_ord+6.5
        end_effector_linear_vel_indicator_ord = end_effector_linear_vel_label_ord+0.8
        end_effector_angular_vel_label_ord = end_effector_linear_vel_label_ord
        end_effector_angular_vel_indicator_ord = end_effector_angular_vel_label_ord+0.8
        diffkine_wrt_frame_label_ord = end_effector_linear_vel_label_ord
        diffkine_wrt_frame_button_ord = end_effector_linear_vel_label_ord+0.8
        show_diffkine_info_button_ord = menu_properties['rows']-1.0
        difffkine_variables_sliders_rows = 3; diffkine_variables_sliders_ord_list = []
        for k in range(self.joints_number):
            diffkine_variables_sliders_ord_list.append(choose_joint_number_diffkine_label_ord+1.5 + 1.5*(k % difffkine_variables_sliders_rows))
            # fkine_variables_sliders_ord_list.append(choose_joint_number_fkine_label_ord+2.5)
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        joints_configuration_diffkine_label_x = 6/7; gbl.menu_label(menu_frame, "Configuration:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_configuration_diffkine_label_x * menu_properties['width'], joints_configuration_diffkine_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_configuration_diffkine_indicator_x = joints_configuration_diffkine_label_x; self.joints_configuration_diffkine_indicator = gbl.menu_label(menu_frame, self.control_or_kinematics_variables_visualization, f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], joints_configuration_diffkine_indicator_x * menu_properties['width'], joints_configuration_diffkine_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        choose_joint_number_diffkine_label_x = 1/6; gbl.menu_label(menu_frame, "Joint\nnumber:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_number_diffkine_label_x * menu_properties['width'], choose_joint_number_diffkine_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_number_diffkine_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = [f"joint {joint}" for joint in range(1, self.joints_number + 1)], justify = "center")
        choose_joint_number_diffkine_combobox_x = 2/6; self.choose_joint_number_diffkine_combobox.place(x = choose_joint_number_diffkine_combobox_x * menu_properties['width'], y = choose_joint_number_diffkine_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_number_diffkine_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_diffkine)
        choose_diffkine_velocity_label_x = 3/6; gbl.menu_label(menu_frame, "Joint velocity:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_diffkine_velocity_label_x * menu_properties['width'], choose_diffkine_velocity_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        diffkine_value_unit_indicator_x = 3/6; self.diffkine_value_unit_indicator = gbl.menu_label(menu_frame, ["(rad/s)", "(m/s)"][self.joints_types_list.index(self.joints_types[self.chosen_joint_number_diffkine - 1])], f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], diffkine_value_unit_indicator_x * menu_properties['width'], diffkine_value_unit_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        self.joint_diffkine_velocity_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 8, values = "", justify = "center")
        joint_diffkine_velocity_combobox_x = 4/6; self.joint_diffkine_velocity_combobox.place(x = joint_diffkine_velocity_combobox_x * menu_properties['width'], y = joint_diffkine_velocity_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.joint_diffkine_velocity_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_diffkine_2)
        self.joint_diffkine_velocity_combobox.bind("<Return>", self.change_chosen_diffkine_velocity)
        end_effector_linear_vel_label_x = 1/5; gbl.menu_label(menu_frame, "End-effector linear velocity (m/s):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_linear_vel_label_x * menu_properties['width'], end_effector_linear_vel_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_linear_vel_indicator_x = end_effector_linear_vel_label_x; self.end_effector_linear_vel_indicator = gbl.menu_label(menu_frame, "linear velocity", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], end_effector_linear_vel_indicator_x * menu_properties['width'], end_effector_linear_vel_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        end_effector_angular_vel_label_x = 11/20; gbl.menu_label(menu_frame, "End-effector angular velocity (°/s):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_angular_vel_label_x * menu_properties['width'], end_effector_angular_vel_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_angular_vel_indicator_x = end_effector_angular_vel_label_x; self.end_effector_angular_vel_indicator = gbl.menu_label(menu_frame, "angular velocity", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], end_effector_angular_vel_indicator_x * menu_properties['width'], end_effector_angular_vel_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        diffkine_wrt_frame_label_x = 5/6; gbl.menu_label(menu_frame, "W.r.t. frame:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], diffkine_wrt_frame_label_x * menu_properties['width'], diffkine_wrt_frame_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        diffkine_wrt_frame_button_x = diffkine_wrt_frame_label_x; self.diffkine_wrt_frame_button = gbl.menu_button(menu_frame, self.diffkine_wrt_frame, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], diffkine_wrt_frame_button_x * menu_properties['width'], diffkine_wrt_frame_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_diffkine_wrt_frame).button
        # show_diffkine_info_button_x = 19/20; self.show_diffkine_info_button = gbl.menu_button(menu_frame, "🕮", f"Calibri {menu_properties['options_font']+5} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_diffkine_info_button_x * menu_properties['width'], show_diffkine_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_diffkine_info).button
        self.diffkine_variables_sliders = []
        for k in range(self.joints_number):
            joint_type_index = self.joints_types_list.index(self.joints_types[k])  # the index of the current joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
            diffkine_velocities_limits = self.diffkine_velocities_limits[k][joint_type_index]  # the control variable limits of the current joint
            self.diffkine_variables_sliders.append(tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font'] - 3}", orient = tk.HORIZONTAL, from_ = diffkine_velocities_limits[0], to = diffkine_velocities_limits[1], resolution = [10**(-self.angles_precision), 10**(-self.distances_precision)][joint_type_index], length = menu_properties['width'] / 7.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = lambda event, slider_num = k: self.change_diffkine_variable_slider(slider_num, event)))
            # fkine_variables_sliders_x = 1/10 * (k + 1); self.fkine_variables_sliders.append(tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font'] - 3}", orient = tk.VERTICAL, length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = lambda event, slider_num = k: self.change_fkine_variable_slider(slider_num, event)))
            diffkine_variables_sliders_x = 1/(np.ceil(self.joints_number / difffkine_variables_sliders_rows) + 1) * (k // difffkine_variables_sliders_rows + 1); self.diffkine_variables_sliders[k].place(x = diffkine_variables_sliders_x * menu_properties['width'], y = diffkine_variables_sliders_ord_list[k] * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
            diffkine_variables_labels_x = 1/(np.ceil(self.joints_number / difffkine_variables_sliders_rows) + 1) * (k // difffkine_variables_sliders_rows + 1) - 2/3*1/7; gbl.menu_label(menu_frame, f"{k + 1}\n{['(°/s)', '(m/s)'][joint_type_index]}:", f"Calibri {menu_properties['options_font']-3} bold", menu_properties['labels_color'], menu_properties['bg_color'], diffkine_variables_labels_x * menu_properties['width'], diffkine_variables_sliders_ord_list[k] * menu_properties['height'] / (menu_properties['rows'] + 1) - 0.8)
        self.update_differential_kinematics_indicators()  # update the indicators of the differential kinematics of the robotic manipulator
    def generate_inverse_differential_kinematics_menu(self, menu_frame, menu_properties, event = None):  # build the inverse differential kinematics menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        get_diffkine_result_button_ord = 2.5
        joints_configuration_invkine_label_ord = get_diffkine_result_button_ord-0.4
        joints_configuration_invkine_indicator_ord = joints_configuration_invkine_label_ord+0.8
        send_invdiffkine_result_button_ord = get_diffkine_result_button_ord
        end_effector_linear_velocity_label_ord = get_diffkine_result_button_ord+1.8
        end_effector_linear_velocity_button_ord = end_effector_linear_velocity_label_ord
        end_effector_angular_velocity_label_ord = end_effector_linear_velocity_label_ord
        end_effector_angular_velocity_button_ord = end_effector_angular_velocity_label_ord
        invdiffkine_wrt_frame_label_ord = end_effector_linear_velocity_label_ord+1.8
        invdiffkine_wrt_frame_button_ord = invdiffkine_wrt_frame_label_ord
        joints_velocities_label_ord = end_effector_linear_velocity_label_ord+3.5
        joints_velocities_indicator_ord = joints_velocities_label_ord
        show_invdiffkine_info_button_ord = menu_properties['rows']-1.0
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        joints_configuration_invkine_label_x = 6/7; gbl.menu_label(menu_frame, "Configuration:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_configuration_invkine_label_x * menu_properties['width'], joints_configuration_invkine_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_configuration_invkine_indicator_x = joints_configuration_invkine_label_x; self.joints_configuration_invkine_indicator = gbl.menu_label(menu_frame, self.control_or_kinematics_variables_visualization, f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], joints_configuration_invkine_indicator_x * menu_properties['width'], joints_configuration_invkine_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        get_diffkine_result_button_x = 1/4; self.get_diffkine_result_button = gbl.menu_button(menu_frame, "↓ get differential kinematics\nanalysis result", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], get_diffkine_result_button_x * menu_properties['width'], get_diffkine_result_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.get_diffkine_velocities_result).button
        send_invdiffkine_result_button_x = 3/5; self.send_invdiffkine_result_button = gbl.menu_button(menu_frame, "↑ send inverse differential\nkinematics analysis result", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_invdiffkine_result_button_x * menu_properties['width'], send_invdiffkine_result_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_invdiffkine_joints_velocities_result).button
        end_effector_linear_velocity_label_x = 5/35; gbl.menu_label(menu_frame, "End-effector\nlinear velocity (m/s):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_linear_velocity_label_x * menu_properties['width'], end_effector_linear_velocity_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_linear_velocity_button_x = 13/35; self.end_effector_linear_velocity_button = gbl.menu_button(menu_frame, "linear\nvelocity", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], end_effector_linear_velocity_button_x * menu_properties['width'], end_effector_linear_velocity_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_end_effector_linear_velocity).button
        end_effector_angular_velocity_label_x = 21/35; gbl.menu_label(menu_frame, "End-effector\nangular velocity (°/s):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_angular_velocity_label_x * menu_properties['width'], end_effector_angular_velocity_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        end_effector_angular_velocity_button_x = 29/35; self.end_effector_angular_velocity_button = gbl.menu_button(menu_frame, "angular\nvelocity", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], end_effector_angular_velocity_button_x * menu_properties['width'], end_effector_angular_velocity_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.choose_end_effector_angular_velocity).button
        invdiffkine_wrt_frame_label_x = 2/5; gbl.menu_label(menu_frame, "Velocities defined w.r.t. frame:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], invdiffkine_wrt_frame_label_x * menu_properties['width'], invdiffkine_wrt_frame_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        invdiffkine_wrt_frame_button_x = 7/10; self.invdiffkine_wrt_frame_button = gbl.menu_button(menu_frame, self.invdiffkine_wrt_frame, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], invdiffkine_wrt_frame_button_x * menu_properties['width'], invdiffkine_wrt_frame_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_invdiffkine_wrt_frame).button
        joints_velocities_label_x = 1/6; gbl.menu_label(menu_frame, "Joints\nvelocities:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joints_velocities_label_x * menu_properties['width'], joints_velocities_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        joints_velocities_indicator_x = 3/5; self.joints_velocities_indicator = gbl.menu_label(menu_frame, "joints velocities", f"Calibri {menu_properties['options_font']} bold", menu_properties['indicators_color'], menu_properties['bg_color'], joints_velocities_indicator_x * menu_properties['width'], joints_velocities_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        # show_invdiffkine_info_button_x = 19/20; self.show_invdiffkine_info_button = gbl.menu_button(menu_frame, "🕮", f"Calibri {menu_properties['options_font']+5} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_invdiffkine_info_button_x * menu_properties['width'], show_invdiffkine_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_invdiffkine_info).button
        self.update_inverse_differential_kinematics_indicators()  # update the indicators of the inverse differential kinematics of the robotic manipulator
    def build_control_robotic_manipulator_menus(self, submenus_titles, submenus_descriptions, event = None):  # build the sub menus for the main menu that controls the robotic manipulator
        self.clear_menus_background()  # clear the menus background
        connection_menu_title = submenus_titles[0]
        connection_menu_info = f"--- {connection_menu_title} ---\n" + submenus_descriptions[0]
        monitor_menu_title = submenus_titles[1]
        monitor_menu_info = f"--- {monitor_menu_title} ---\n" + submenus_descriptions[1]
        control_menu_title = submenus_titles[2]
        control_menu_info = f"--- {control_menu_title} ---\n" + submenus_descriptions[2]
        if not self.expanded_serial_monitor:  # if the user does not choose the expanded serial monitor
            # create the establish connection sub menu
            connection_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = connection_menu_title, info = connection_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 6, \
                                                rows = 3, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # create the serial monitor menu
            monitor_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = monitor_menu_title, info = monitor_menu_info, row = 2, column = 0, width = self.menus_background_width, height = self.menus_background_height * 2 / 6, \
                                            rows = 8, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # create the control joints sub menu
            control_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = control_menu_title, info = control_menu_info, row = 3, column = 0, width = self.menus_background_width, height = self.menus_background_height * 3 / 6, \
                                            rows = 10, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # generate the sub menus
            self.generate_establish_connection_menu(self.create_static_menu_frame(connection_menu_properties), connection_menu_properties)
            self.generate_serial_monitor_menu(self.create_static_menu_frame(monitor_menu_properties), monitor_menu_properties)
            self.generate_control_joints_menu(self.create_static_menu_frame(control_menu_properties), control_menu_properties)
        else:  # if the user chooses the expanded serial monitor
            # create the serial monitor menu
            monitor_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = monitor_menu_title, info = monitor_menu_info, row = 1, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 2, \
                                            rows = 8, bg_color = "black", title_font = 15, options_font = 11, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # create the control joints sub menu
            control_menu_properties = dict(main_menu_title = self.main_menus_build_details[self.main_menu_choice]['title'], sub_menu_title = control_menu_title, info = control_menu_info, row = 2, column = 0, width = self.menus_background_width, height = self.menus_background_height * 1 / 2, \
                                            rows = 10, bg_color = "black", title_font = 15, options_font = 12, indicators_color = "yellow", subtitles_color = "magenta", labels_color = "lime", buttons_color = "white")
            # generate the sub menus
            self.generate_serial_monitor_menu(self.create_static_menu_frame(monitor_menu_properties), monitor_menu_properties)
            self.generate_control_joints_menu(self.create_static_menu_frame(control_menu_properties), control_menu_properties)
    def generate_establish_connection_menu(self, menu_frame, menu_properties, event = None):  # build the establish connection menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        serial_port_label_ord = 2
        baudrate_label_ord = 3
        serial_ports_search_button_ord = serial_port_label_ord
        serial_connection_state_label_ord = 2
        serial_connection_indicator_ord = 3
        serial_connect_button_ord = 2.5
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        serial_port_label_x = 1/7; gbl.menu_label(menu_frame, "Serial port:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], serial_port_label_x * menu_properties['width'], serial_port_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.serial_ports_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 7, values = self.available_serial_ports, justify = "center")
        serial_ports_combobox_x = 2/7; self.serial_ports_combobox.place(x = serial_ports_combobox_x * menu_properties['width'], y = serial_port_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.serial_ports_combobox.bind("<<ComboboxSelected>>", self.change_serial_port)
        serial_ports_search_button_x = 2.7/7; self.serial_ports_search_button = gbl.menu_button(menu_frame, "⟲", f"Calibri {menu_properties['options_font']-2} bold", menu_properties['buttons_color'], menu_properties['bg_color'], serial_ports_search_button_x * menu_properties['width'], serial_ports_search_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.obtain_serial_ports).button
        baudrate_label_x = serial_port_label_x; gbl.menu_label(menu_frame, "Baud rate:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], baudrate_label_x * menu_properties['width'], baudrate_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.baudrates_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 7, values = self.baudrates_list, justify = "center")
        baudrates_combobox_x = serial_ports_combobox_x; self.baudrates_combobox.place(x = baudrates_combobox_x * menu_properties['width'], y = baudrate_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.baudrates_combobox.bind("<<ComboboxSelected>>", self.change_baudrate)
        serial_connection_state_label_x = 4/7; gbl.menu_label(menu_frame, "Serial connection state:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], serial_connection_state_label_x * menu_properties['width'], serial_connection_state_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        serial_connection_indicator_x = 4/7; self.serial_connection_indicator = gbl.menu_label(menu_frame, self.serial_connection_state, f"Calibri {menu_properties['options_font']} bold", "black", self.serial_connection_indicator_colors[self.serial_connection_states_list.index(self.serial_connection_state)], serial_connection_indicator_x * menu_properties['width'], serial_connection_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        serial_connect_button_x = 6/7; self.serial_connect_button = gbl.menu_button(menu_frame, self.serial_connect_disconnect_command, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], serial_connect_button_x * menu_properties['width'], serial_connect_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.serial_connect_disconnect).button
        self.update_serial_connection_indicators(self.serial_connection_state)  # update the serial connection indicator according to the current serial connection state
    def generate_serial_monitor_menu(self, menu_frame, menu_properties, event = None):  # build the serial monitor menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        console_serial_monitor_ord = int(menu_properties['rows'] / 2) + 0.5
        command_entrybox_label_ord = menu_properties['rows']
        command_entrybox_ord = command_entrybox_label_ord
        send_command_button_ord = command_entrybox_label_ord
        command_starting_text_label_ord = 3
        command_starting_text_entrybox_ord = command_starting_text_label_ord+1
        command_ending_text_label_ord = command_starting_text_label_ord+2
        command_ending_text_entrybox_ord = command_ending_text_label_ord+1
        show_hide_ok_label_ord = 1
        show_hide_ok_button_ord = 1
        show_hide_status_label_ord = 2
        show_hide_status_button_ord = 2
        expand_serial_monitor_menu_ord = 1
        clear_serial_monitor_ord = 2
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        self.console_serial_monitor = tk.Text(menu_frame, font = f"Calibri 10", width = 60, height = [8, 14][[False, True].index(self.expanded_serial_monitor)], bg = "white", fg = "black", wrap = "word")
        console_serial_monitor_x = 3/8; self.console_serial_monitor.place(x = console_serial_monitor_x * menu_properties['width'], y = console_serial_monitor_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.console_serial_monitor.insert(tk.END, self.serial_monitor_text)  # insert the saved serial monitor text to the console window
        self.console_serial_monitor.see("end")  # make the console serial monitor to scroll to the end
        command_entrybox_label_x = 1/10; gbl.menu_label(menu_frame, "Command:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], command_entrybox_label_x * menu_properties['width'], command_entrybox_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.command_entrybox = ttk.Entry(menu_frame, font = f"Calibri 10", width = 60, justify = "left")
        command_entrybox_x = 1/2; self.command_entrybox.place(x = command_entrybox_x * menu_properties['width'], y = command_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.command_entrybox.bind("<Return>", lambda event: self.send_serial_command(self.command_starting_text + self.command_entrybox.get() + self.command_ending_text, event))
        send_command_button_x = 9/10; self.send_command_button = gbl.menu_button(menu_frame, "Send", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_command_button_x * menu_properties['width'], send_command_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: self.send_serial_command(self.command_starting_text + self.command_entrybox.get() + self.command_ending_text, event)).button
        command_starting_text_label_x = 5/6; gbl.menu_label(menu_frame, "Command starting text:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], command_starting_text_label_x * menu_properties['width'], command_starting_text_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.command_starting_text_entrybox = ttk.Entry(menu_frame, font = f"Calibri 10", width = 15, justify = "left")
        command_starting_text_entrybox_x = 5/6; self.command_starting_text_entrybox.place(x = command_starting_text_entrybox_x * menu_properties['width'], y = command_starting_text_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.command_starting_text_entrybox.insert(0, self.command_starting_text)  # insert the saved command starting text to the entry box
        self.command_starting_text_entrybox.bind("<Return>", self.change_command_starting_text)
        command_ending_text_label_x = 5/6; gbl.menu_label(menu_frame, "Command ending text:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], command_ending_text_label_x * menu_properties['width'], command_ending_text_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.command_ending_text_entrybox = ttk.Entry(menu_frame, font = f"Calibri 10", width = 15, justify = "left")
        command_ending_text_entrybox_x = 5/6; self.command_ending_text_entrybox.place(x = command_ending_text_entrybox_x * menu_properties['width'], y = command_ending_text_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.command_ending_text_entrybox.insert(0, self.command_ending_text)  # insert the saved command ending text to the entry box
        self.command_ending_text_entrybox.bind("<Return>", self.change_command_ending_text)
        show_hide_ok_label_x = 8/10; gbl.menu_label(menu_frame, "OK:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], show_hide_ok_label_x * menu_properties['width'], show_hide_ok_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_hide_ok_button_x = 13/15; self.show_hide_ok_button = gbl.menu_button(menu_frame, self.show_responses_indicators[[False, True].index(self.show_ok_responses)], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_hide_ok_button_x * menu_properties['width'], show_hide_ok_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_hide_ok_responses_on_console).button
        show_hide_status_label_x = 8/10; gbl.menu_label(menu_frame, "Status:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], show_hide_status_label_x * menu_properties['width'], show_hide_status_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        show_hide_status_button_x = 13/15; self.show_hide_status_button = gbl.menu_button(menu_frame, self.show_responses_indicators[[False, True].index(self.show_status_responses)], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], show_hide_status_button_x * menu_properties['width'], show_hide_status_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.show_hide_status_responses_on_console).button
        expand_serial_monitor_menu_x = 19/20; self.expand_serial_monitor_menu_button = gbl.menu_button(menu_frame, ["⇱", "⇲"][[False, True][self.expanded_serial_monitor]], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], expand_serial_monitor_menu_x * menu_properties['width'], expand_serial_monitor_menu_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.expand_serial_monitor_menu).button
        clear_serial_monitor_x = 19/20; self.clear_serial_monitor_button = gbl.menu_button(menu_frame, "🗑", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], clear_serial_monitor_x * menu_properties['width'], clear_serial_monitor_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.clear_serial_monitor).button
    def generate_control_joints_menu(self, menu_frame, menu_properties, event = None):  # build the control joints menu
        # options order
        menu_title_ord = 1
        menu_info_button_ord = 1
        choose_joint_number_control_label_ord = 2.5
        joint_type_indicator_ord = choose_joint_number_control_label_ord
        joint_control_variable_combobox_ord = choose_joint_number_control_label_ord
        get_kinematics_values_button_ord = joint_control_variable_combobox_ord
        choose_joint_motors_label_ord = choose_joint_number_control_label_ord+1.5
        increase_joint_motors_button_ord = choose_joint_motors_label_ord-0.4
        decrease_joints_motors_button_ord = choose_joint_motors_label_ord+0.4
        choose_motors_mult_factors_label_ord = choose_joint_motors_label_ord
        joint_variable_slider_label_ord = choose_joint_motors_label_ord+1.5
        joint_variable_slider_ord = joint_variable_slider_label_ord+0.5
        set_joints_variables_to_zero_button_ord = joint_variable_slider_label_ord+1.0
        increase_joint_var_1_button_ord = joint_variable_slider_ord-0.4
        decrease_joint_var_1_button_ord = joint_variable_slider_ord+0.4
        increase_joint_var_2_button_ord = joint_variable_slider_ord-0.4
        decrease_joint_var_2_button_ord = joint_variable_slider_ord+0.4
        increase_joint_var_3_button_ord = joint_variable_slider_ord-0.4
        decrease_joint_var_3_button_ord = joint_variable_slider_ord+0.4
        end_effector_slider_label_ord = joint_variable_slider_label_ord+2
        end_effector_slider_ord = end_effector_slider_label_ord+0.5
        set_end_effector_to_zero_button_ord = end_effector_slider_label_ord+1
        choose_end_effector_motor_label_ord = end_effector_slider_label_ord
        choose_end_effector_motor_entrybox_ord = choose_end_effector_motor_label_ord+1
        choose_end_effector_mult_factor_label_ord = end_effector_slider_label_ord
        choose_end_effector_mult_factor_entrybox_ord = choose_end_effector_mult_factor_label_ord+1
        choose_control_mode_label_ord = end_effector_slider_label_ord+2.5
        choose_control_mode_button_ord = choose_control_mode_label_ord
        send_command_end_effector_button_ord = choose_control_mode_label_ord
        send_command_all_motors_button_ord = choose_control_mode_label_ord
        send_command_joint_motors_button_ord = choose_control_mode_label_ord
        # create the options
        menu_title_x = 1/2; gbl.menu_label(menu_frame, menu_properties['sub_menu_title'], f"Calibri {menu_properties['title_font']} bold underline", "gold", menu_properties['bg_color'], menu_title_x * menu_properties['width'], menu_title_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        menu_info_button_x = 1/20; gbl.menu_button(menu_frame, "ⓘ", f"Calibri {menu_properties['title_font'] - 5} bold", "white", menu_properties['bg_color'], menu_info_button_x * menu_properties['width'], menu_info_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event: ms.showinfo(menu_properties['sub_menu_title'], menu_properties['info'], master = self.root)).button
        choose_joint_number_control_label_x = 4/30; gbl.menu_label(menu_frame, "Joint number:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_number_control_label_x * menu_properties['width'], choose_joint_number_control_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_number_control_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = [f"joint {joint}" for joint in range(1, self.joints_number + 1)], justify = "center")
        choose_joint_number_control_combobox_x = 10/30; self.choose_joint_number_control_combobox.place(x = choose_joint_number_control_combobox_x * menu_properties['width'], y = choose_joint_number_control_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_number_control_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_control)
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        joint_type_indicator_x = 16/30; self.joint_type_indicator = gbl.menu_label(menu_frame, self.joints_types[self.chosen_joint_number_control - 1] + [" (degrees)", " (meters)"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", "black", "orange", joint_type_indicator_x * menu_properties['width'], joint_type_indicator_ord * menu_properties['height'] / (menu_properties['rows'] + 1)).label
        self.joint_control_variable_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "normal", width = 8, values = "", justify = "center")
        joint_control_variable_combobox_x = 22/30; self.joint_control_variable_combobox.place(x = joint_control_variable_combobox_x * menu_properties['width'], y = joint_control_variable_combobox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.joint_control_variable_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_number_control_2)
        self.joint_control_variable_combobox.bind("<Return>", self.change_chosen_control_variable)
        get_kinematics_values_button_x = 27/30; self.get_kinematics_values_button = gbl.menu_button(menu_frame, "get\nkinematics\nvalues", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], get_kinematics_values_button_x * menu_properties['width'], get_kinematics_values_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.copy_fkine_to_control_values).button
        choose_joint_motors_label_x = 4/30; gbl.menu_label(menu_frame, "Joint motors:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_joint_motors_label_x * menu_properties['width'], choose_joint_motors_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.choose_joint_motors_combobox = ttk.Combobox(menu_frame, font = f"Calibri {menu_properties['options_font']}", state = "readonly", width = 8, values = self.joints_motors_list[self.chosen_joint_number_control - 1], justify = "center")
        choose_joint_motors_combobox_x = 10/30; self.choose_joint_motors_combobox.place(x = choose_joint_motors_combobox_x * menu_properties['width'], y = choose_joint_motors_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.choose_joint_motors_combobox.bind("<<ComboboxSelected>>", self.change_chosen_joint_motors)
        increase_joint_motors_button_x = 13/30; self.increase_joint_motors_button = gbl.menu_button(menu_frame, "+", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], increase_joint_motors_button_x * menu_properties['width'], increase_joint_motors_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.increase_chosen_joint_motors).button
        decrease_joints_motors_button_x = 13/30; self.decrease_joints_motors_button = gbl.menu_button(menu_frame, "-", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], decrease_joints_motors_button_x * menu_properties['width'], decrease_joints_motors_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.decrease_chosen_joint_motors).button
        choose_motors_mult_factors_label_x = 18/30; gbl.menu_label(menu_frame, "Motors factors\n(for commands):", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_motors_mult_factors_label_x * menu_properties['width'], choose_motors_mult_factors_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.motors_mult_factors_entrybox = ttk.Entry(menu_frame, font = f"Calibri {menu_properties['options_font']}", width = 8, justify = "center")
        motors_mult_factors_entrybox_x = 23/30; self.motors_mult_factors_entrybox.place(x = motors_mult_factors_entrybox_x * menu_properties['width'], y = choose_motors_mult_factors_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.motors_mult_factors_entrybox.bind("<Return>", self.change_motors_mult_factors_entrybox)
        joint_variable_slider_label_x = 1/6; gbl.menu_label(menu_frame, "Joint variable control:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], joint_variable_slider_label_x * menu_properties['width'], joint_variable_slider_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.joint_variable_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']}", orient = tk.HORIZONTAL, length = menu_properties['width'] / 3.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_control_variable_slider)
        joint_variable_slider_x = 1/2; self.joint_variable_slider.place(x = joint_variable_slider_x * menu_properties['width'], y = joint_variable_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        set_joints_variables_to_zero_button_x = 1/6; self.set_joints_variables_to_zero_button = gbl.menu_button(menu_frame, "set joints to 0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], set_joints_variables_to_zero_button_x * menu_properties['width'], set_joints_variables_to_zero_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.set_joints_variables_to_zero).button
        increase_joint_var_1_button_x = 9/12; self.increase_joint_var_1_button = gbl.menu_button(menu_frame, ["+0.1", "+0.001"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], increase_joint_var_1_button_x * menu_properties['width'], increase_joint_var_1_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = 1: self.increase_decrease_control_variable(change_type)).button
        decrease_joint_var_1_button_x = 9/12; self.decrease_joint_var_1_button = gbl.menu_button(menu_frame, ["-0.1", "-0.001"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], decrease_joint_var_1_button_x * menu_properties['width'], decrease_joint_var_1_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = -1: self.increase_decrease_control_variable(change_type)).button
        increase_joint_var_2_button_x = 10/12; self.increase_joint_var_2_button = gbl.menu_button(menu_frame, ["+1", "+0.01"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], increase_joint_var_2_button_x * menu_properties['width'], increase_joint_var_2_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = 2: self.increase_decrease_control_variable(change_type)).button
        decrease_joint_var_2_button_x = 10/12; self.decrease_joint_var_2_button = gbl.menu_button(menu_frame, ["-1", "-0.01"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], decrease_joint_var_2_button_x * menu_properties['width'], decrease_joint_var_2_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = -2: self.increase_decrease_control_variable(change_type)).button
        increase_joint_var_3_button_x = 11/12; self.increase_joint_var_3_button = gbl.menu_button(menu_frame, ["+10", "+0.1"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], increase_joint_var_3_button_x * menu_properties['width'], increase_joint_var_3_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = 3: self.increase_decrease_control_variable(change_type)).button
        decrease_joint_var_3_button_x = 11/12; self.decrease_joint_var_3_button = gbl.menu_button(menu_frame, ["-10", "-0.1"][joint_type_index], f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], decrease_joint_var_3_button_x * menu_properties['width'], decrease_joint_var_3_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), lambda event, change_type = -3: self.increase_decrease_control_variable(change_type)).button
        end_effector_slider_label_x = 1/6; gbl.menu_label(menu_frame, "End-effector control:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], end_effector_slider_label_x * menu_properties['width'], end_effector_slider_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.end_effector_slider = tk.Scale(menu_frame, font = f"Arial {menu_properties['options_font']}", orient = tk.HORIZONTAL, from_ = self.control_end_effector_limits[0], to = self.control_end_effector_limits[1], length = menu_properties['width'] / 5.0, bg = menu_properties['bg_color'], fg = "white", troughcolor = "brown", highlightbackground = menu_properties['bg_color'], highlightcolor = menu_properties['bg_color'], highlightthickness = 2, sliderlength = 25, command = self.change_control_end_effector_slider)
        end_effector_slider_x = 3/7; self.end_effector_slider.place(x = end_effector_slider_x * menu_properties['width'], y = end_effector_slider_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        set_end_effector_to_zero_button_x = 1/6; self.set_end_effector_to_zero_button = gbl.menu_button(menu_frame, "set end-effector to 0", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], set_end_effector_to_zero_button_x * menu_properties['width'], set_end_effector_to_zero_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.set_end_effector_to_zero).button
        choose_end_effector_motor_label_x = 4/6; gbl.menu_label(menu_frame, "Motor:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_end_effector_motor_label_x * menu_properties['width'], choose_end_effector_motor_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.end_effector_motor_entrybox = ttk.Entry(menu_frame, font = f"Calibri {menu_properties['options_font']}", width = 8, justify = "center")
        end_effector_motor_entrybox_x = 4/6; self.end_effector_motor_entrybox.place(x = end_effector_motor_entrybox_x * menu_properties['width'], y = choose_end_effector_motor_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.end_effector_motor_entrybox.bind("<Return>", self.change_end_effector_motor)
        choose_end_effector_mult_factor_label_x = 6/7; gbl.menu_label(menu_frame, "Motor factor:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_end_effector_mult_factor_label_x * menu_properties['width'], choose_end_effector_mult_factor_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        self.end_effector_mult_factor_entrybox = ttk.Entry(menu_frame, font = f"Calibri {menu_properties['options_font']}", width = 8, justify = "center")
        end_effector_mult_factor_x = 6/7; self.end_effector_mult_factor_entrybox.place(x = end_effector_mult_factor_x * menu_properties['width'], y = choose_end_effector_mult_factor_entrybox_ord * menu_properties['height'] / (menu_properties['rows'] + 1), anchor = "center")
        self.end_effector_mult_factor_entrybox.bind("<Return>", self.change_end_effector_mult_factor)
        choose_control_mode_label_x = 1/6; gbl.menu_label(menu_frame, "Control mode:", f"Calibri {menu_properties['options_font']} bold", menu_properties['labels_color'], menu_properties['bg_color'], choose_control_mode_label_x * menu_properties['width'], choose_control_mode_label_ord * menu_properties['height'] / (menu_properties['rows'] + 1))
        choose_control_mode_button_x = 2/6; self.choose_control_mode_button = gbl.menu_button(menu_frame, self.robotic_manipulator_control_mode, f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], choose_control_mode_button_x * menu_properties['width'], choose_control_mode_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.change_robotic_manipulator_control_mode).button
        send_command_joint_motors_button_x = 3/6; self.send_command_joint_motors_button = gbl.menu_button(menu_frame, "GO\njoint", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_command_joint_motors_button_x * menu_properties['width'], send_command_joint_motors_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_command_to_chosen_joint_motors).button
        send_command_all_motors_button_x = 4/6; self.send_command_all_motors_button = gbl.menu_button(menu_frame, "GO ALL\njoints", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_command_all_motors_button_x * menu_properties['width'], send_command_all_motors_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_command_to_all_motors).button
        send_command_end_effector_button_x = 5/6; self.send_command_end_effector_button = gbl.menu_button(menu_frame, "GO\nend-effector", f"Calibri {menu_properties['options_font']} bold", menu_properties['buttons_color'], menu_properties['bg_color'], send_command_end_effector_button_x * menu_properties['width'], send_command_end_effector_button_ord * menu_properties['height'] / (menu_properties['rows'] + 1), self.send_command_to_end_effector).button
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator

    # functions for all the menus options
    # functions for the main menu where the robotic manipulator model is built
    def change_robotic_manipulator_model_name(self, event = None):  # change the name of the robotic manipulator model
        self.robotic_manipulator_model_name = self.model_name_entrybox.get()  # change the name of the robotic manipulator model
    def set_frames_links_number(self, joints_number, event = None):  # set the total frames and links number of the robotic manipulator
        frames_number = joints_number + 3  # set the frames number of the robotic manipulator
        links_number = joints_number + 1  # set the links number of the robotic manipulator
        return frames_number, links_number  # return the frames and links number
    def change_joints_number(self, event = None):  # change the joints number of the robotic manipulator
        if not self.robotic_manipulator_is_built:  # if there is no robotic manipulator model built
            ask_joints_number = sd.askinteger("Joints number", f"Enter the number of the robotic manipulator joints ({self.max_joints_number} maximum):", initialvalue = self.joints_number, minvalue = 1, maxvalue = self.max_joints_number, parent = self.menus_area)  # ask the user to enter the number of the robotic manipulator joints
            if ask_joints_number != None:  # if the user enters a number
                previous_joints_number = self.joints_number  # remember the previous joints number of the robotic manipulator
                self.joints_number = ask_joints_number  # set the total number of joints of the robotic manipulator
                self.frames_number, self.links_number = self.set_frames_links_number(joints_number = self.joints_number)  # set the total number of frames and links of the robotic manipulator
                if self.joints_number < previous_joints_number:  # if the new joints number is less than the previous joints number
                    self.chosen_joint_number_model = 1  # set the chosen joint number for the model to the last joint
                    self.chosen_frame_visualization = "frame 0"  # set the chosen frame for the visualization to the "frame 0" ("joint 1") frame
                    self.chosen_link_visualization = "link 1"  # set the chosen link for the visualization to the "link 1" link
                    self.chosen_joint_number_control = 1  # set the chosen joint number for the control to the first joint
                self.joints_number_update_lists_values(previous_joints_number, self.joints_number)  # update some lists values according to the new joints number of the robotic manipulator
                self.update_model_visualization_indicators()  # update the model and visualization indicators
        else:  # if a robotic manipulator model is built
            ms.showerror("Error", "It is not allowed to change the joints number of the built robotic manipulator! You have to destroy it first!", parent = self.menus_area)  # show an error message
    def joints_number_update_lists_values(self, previous_joints_number, current_joints_number):  # update some lists values according to the new joints number of the robotic manipulator
        if current_joints_number >= previous_joints_number:  # if the new joints number is greater than or equal to the previous joints number
            self.joints_types += [self.joints_types_list[0] for _ in range(current_joints_number - previous_joints_number)]  # add the new joints types to the list of the joints types of the robotic manipulator
            self.a_den_har_parameters += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new a parameters to the list of the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator
            self.d_den_har_parameters += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new d parameters to the list of the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator
            self.alpha_den_har_parameters += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new alpha parameters to the list of the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator
            self.theta_den_har_parameters += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new theta parameters to the list of the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator
            self.control_joints_variables += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new control variables to the list of the control joints variables of the robotic manipulator
            self.control_joints_variables_limits += [copy.deepcopy(self.control_joints_variables_extreme_limits) for _ in range(current_joints_number - previous_joints_number)]  # add the new control variables limits to the list of the control joints variables limits of the robotic manipulator
            self.joints_z_axis_positions += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new joints z axis positions to the list of the joints z axis positions of the robotic manipulator
            self.frames_origins_colors = list(self.frames_origins_colors[:-1]) + ["black" for _ in range(current_joints_number - previous_joints_number)] + [self.frames_origins_colors[-1]]  # add the new frames origins colors to the list of the frames origins colors of the robotic manipulator
            self.frames_origins_sizes = list(self.frames_origins_sizes[:-1]) + [self.frames_origins_sizes[-1] for _ in range(current_joints_number - previous_joints_number)] + [self.frames_origins_sizes[-1]]  # add the new frames origins sizes to the list of the frames origins sizes of the robotic manipulator
            self.initial_links_lengths = list(self.initial_links_lengths[:-1]) + [self.initial_links_lengths[-1] for _ in range(current_joints_number - previous_joints_number)] + [self.initial_links_lengths[-1]]  # add the new initial links lengths to the list of the initial links lengths of the robotic manipulator
            self.links_colors = list(self.links_colors[:-1]) + ["red" for _ in range(current_joints_number - previous_joints_number)] + [self.links_colors[-1]]  # add the new links colors to the list of the links colors of the robotic manipulator
            self.links_sizes = list(self.links_sizes[:-1]) + [self.links_sizes[-1] for _ in range(current_joints_number - previous_joints_number)] + [self.links_sizes[-1]]  # add the new links sizes to the list of the links sizes of the robotic manipulator
            self.joints_motors_list += [["X"] for _ in range(current_joints_number - previous_joints_number)]  # add the new joints motors to the list of the joints motors of the robotic manipulator
            self.joints_motors_mult_factors += [[1.0] for _ in range(current_joints_number - previous_joints_number)]  # add the new motors multiplication factors to the list of the motors multiplication factors of the robotic manipulator
            self.forward_kinematics_variables += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new forward kinematics variables to the list of the forward kinematics variables of the robotic manipulator
            self.differential_kinematics_velocities += [0.0 for _ in range(current_joints_number - previous_joints_number)]  # add the new differential kinematics velocities to the list of the differential kinematics velocities of the robotic manipulator
            self.diffkine_velocities_limits += [copy.deepcopy(self.diffkine_velocities_limits[0]) for _ in range(current_joints_number - previous_joints_number)]  # add the new differential kinematics velocities limits of the robotic manipulator
        else:  # if the new joints number is less than the previous joints number
            self.joints_types = copy.deepcopy(self.joints_types[:self.joints_number])  # remove the joints types that are not needed anymore
            self.a_den_har_parameters = copy.deepcopy(self.a_den_har_parameters[:self.joints_number])  # remove the a parameters that are not needed anymore
            self.d_den_har_parameters = copy.deepcopy(self.d_den_har_parameters[:self.joints_number])  # remove the d parameters that are not needed anymore
            self.alpha_den_har_parameters = copy.deepcopy(self.alpha_den_har_parameters[:self.joints_number])  # remove the alpha parameters that are not needed anymore
            self.theta_den_har_parameters = copy.deepcopy(self.theta_den_har_parameters[:self.joints_number])  # remove the theta parameters that are not needed anymore
            self.control_joints_variables = copy.deepcopy(self.control_joints_variables[:self.joints_number])  # remove the control variables that are not needed anymore
            self.control_joints_variables_limits = copy.deepcopy(self.control_joints_variables_limits[:self.joints_number])  # remove the control limits that are not needed anymore
            self.joints_z_axis_positions = copy.deepcopy(self.joints_z_axis_positions[:self.joints_number])  # remove the joints z axis positions that are not needed anymore
            self.frames_origins_colors = copy.deepcopy(self.frames_origins_colors[:(self.frames_number - 1)] + [self.frames_origins_colors[-1]])  # remove the frames origins colors that are not needed anymore
            self.frames_origins_sizes = copy.deepcopy(self.frames_origins_sizes[:(self.frames_number - 1)] + [self.frames_origins_sizes[-1]])  # remove the frames origins sizes that are not needed anymore
            self.initial_links_lengths = copy.deepcopy(self.initial_links_lengths[:(self.links_number - 1)] + [self.initial_links_lengths[-1]])  # remove the initial links lengths that are not needed anymore
            self.links_colors = copy.deepcopy(self.links_colors[:(self.links_number - 1)] + [self.links_colors[-1]])  # remove the links colors that are not needed anymore
            self.links_sizes = copy.deepcopy(self.links_sizes[:(self.links_number - 1)] + [self.links_sizes[-1]])  # remove the links sizes that are not needed anymore
            self.joints_motors_list = copy.deepcopy(self.joints_motors_list[:self.joints_number])  # remove the joints motors that are not needed anymore
            self.joints_motors_mult_factors = copy.deepcopy(self.joints_motors_mult_factors[:self.joints_number])  # remove the motors multiplication factors that are not needed anymore
            self.forward_kinematics_variables = copy.deepcopy(self.forward_kinematics_variables[:self.joints_number])  # remove the forward kinematics variables that are not needed anymore
            self.differential_kinematics_velocities = copy.deepcopy(self.differential_kinematics_velocities[:self.joints_number])  # remove the differential kinematics velocities that are not needed anymore
            self.diffkine_velocities_limits = copy.deepcopy(self.diffkine_velocities_limits[:self.joints_number])  # remove the differential kinematics velocities limits that are not needed anymore
    def change_chosen_joint_number_model(self, event = None):  # change the chosen joint number of the robotic manipulator
        self.chosen_joint_number_model = int(self.choose_joint_number_model_combobox.get().split(" ")[-1])  # change the chosen joint number of the robotic manipulator
        self.chosen_frame_visualization = f"frame {self.chosen_joint_number_model - 1}"  # change the chosen frame of the robotic manipulator for visualization
        self.chosen_link_visualization = f"link {self.chosen_joint_number_model}"  # change the chosen link of the robotic manipulator for visualization
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_chosen_joint_type(self, event = None):  # change the chosen joint type of the robotic manipulator
        self.joints_types[self.chosen_joint_number_model - 1] = self.joints_types_combobox.get()  # change the chosen joint type of the robotic manipulator
        if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1]:
            self.control_joints_variables[self.chosen_joint_number_model - 1] = self.d_den_har_parameters[self.chosen_joint_number_model - 1]  # set the current control variable to the d parameter of the chosen joint
        else:  # if the chosen joint is revolute
            self.control_joints_variables[self.chosen_joint_number_model - 1] = self.theta_den_har_parameters[self.chosen_joint_number_model - 1]  # set the current control variable to the theta parameter of the chosen joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_a_den_har_parameter(self, event = None):  # change the a parameter of the Denavit - Hartenberg parameters of the robotic manipulator
        ask_a_den_har_parameter = sd.askfloat("Define parameter a", f"Enter the parameter a for the joint {self.chosen_joint_number_model} (in meters):", initialvalue = self.a_den_har_parameters[self.chosen_joint_number_model - 1], minvalue = self.a_parameters_limits[0], maxvalue = self.a_parameters_limits[1], parent = self.menus_area)
        if ask_a_den_har_parameter != None:  # if the user enters a number
            self.a_den_har_parameters[self.chosen_joint_number_model - 1] = ask_a_den_har_parameter  # change the a parameter for the chosen model joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_d_den_har_parameter(self, event = None):  # change the d parameter of the Denavit - Hartenberg parameters of the robotic manipulator
        if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0]:  # if the chosen joint is revolute
            ask_d_den_har_parameter = sd.askfloat("Define parameter d", f"Enter the parameter d for the \nrevolute joint {self.chosen_joint_number_model} (in meters):", initialvalue = self.d_den_har_parameters[self.chosen_joint_number_model - 1], minvalue = self.d_parameters_limits[0], maxvalue = self.d_parameters_limits[1], parent = self.menus_area)
        elif self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1]:  # if the chosen joint is prismatic
            ask_d_den_har_parameter = sd.askfloat("Define variable d", f"Enter the parameter d for the \nprismatic joint {self.chosen_joint_number_model} (in meters):", initialvalue = self.d_den_har_parameters[self.chosen_joint_number_model - 1], minvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][1][0], maxvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][1][1], parent = self.menus_area)
        if ask_d_den_har_parameter != None:  # if the user enters a number
            self.d_den_har_parameters[self.chosen_joint_number_model - 1] = ask_d_den_har_parameter  # change the d parameter for the chosen model joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_alpha_den_har_parameter(self, event = None):  # change the alpha parameter of the Denavit - Hartenberg parameters of the robotic manipulator
        ask_alpha_den_har_parameter = sd.askfloat("Define parameter alpha", f"Enter the parameter alpha for the joint {self.chosen_joint_number_model} (in degrees):", initialvalue = np.rad2deg(self.alpha_den_har_parameters[self.chosen_joint_number_model - 1]), minvalue = np.rad2deg(self.alpha_parameters_limits[0]), maxvalue = np.rad2deg(self.alpha_parameters_limits[1]), parent = self.menus_area)
        if ask_alpha_den_har_parameter != None:  # if the user enters a number
            self.alpha_den_har_parameters[self.chosen_joint_number_model - 1] = np.deg2rad(ask_alpha_den_har_parameter)  # change the alpha parameter for the chosen model joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_theta_den_har_parameter(self, event = None):  # change the theta parameter of the Denavit - Hartenberg parameters of the robotic manipulator
        if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1]:  # if the chosen joint is prismatic
            ask_theta_den_har_parameter = sd.askfloat("Define parameter theta", f"Enter the parameter theta for the \nprismatic joint {self.chosen_joint_number_model} (in degrees):", initialvalue = np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]), minvalue = np.rad2deg(self.theta_parameters_limits[0]), maxvalue = np.rad2deg(self.theta_parameters_limits[1]), parent = self.menus_area)
        elif self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0]:  # if the chosen joint is revolute
            ask_theta_den_har_parameter = sd.askfloat("Define variable theta", f"Enter the parameter theta for the \nrevolute joint {self.chosen_joint_number_model} (in degrees):", initialvalue = np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]), minvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][0][0], maxvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][0][1], parent = self.menus_area)
        if ask_theta_den_har_parameter != None:  # if the user enters a number
            self.theta_den_har_parameters[self.chosen_joint_number_model - 1] = np.deg2rad(ask_theta_den_har_parameter)  # change the theta parameter for the chosen model joint
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_joint_variable_min_limit(self, event = None):  # change the minimum limit of the variable of the chosen joint
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_model - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the model
        ask_joint_variable_min_limit = sd.askfloat("Define variable's minimum limit", f"Enter the minimum limit of the joint {self.chosen_joint_number_model} \nvariable (in {['degrees', 'meters'][joint_type_index]}):", initialvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][0], minvalue = self.control_joints_variables_extreme_limits[joint_type_index][0], maxvalue = self.control_joints_variables_extreme_limits[joint_type_index][1], parent = self.menus_area)  # ask the user to enter the minimum limit of the joint variable
        if ask_joint_variable_min_limit != None:  # if the user enters a number
            if ask_joint_variable_min_limit <= self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][1]:  # if the minimum limit is less than or equal to the maximum limit
                self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][0] = ask_joint_variable_min_limit
                if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1] and self.d_den_har_parameters[self.chosen_joint_number_model - 1] < ask_joint_variable_min_limit:  # if the chosen joint is prismatic and the d parameter is less than the minimum limit
                    self.d_den_har_parameters[self.chosen_joint_number_model - 1] = ask_joint_variable_min_limit  # change the d parameter for the chosen model joint
                elif self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0] and np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]) < ask_joint_variable_min_limit:  # if the chosen joint is revolute and the theta parameter is less than the minimum limit
                    self.theta_den_har_parameters[self.chosen_joint_number_model - 1] = np.deg2rad(ask_joint_variable_min_limit)  # change the theta parameter for the chosen model joint
            else:  # if the minimum limit is greater than the maximum limit
                ms.showerror("Error", "The minimum limit must be less than or equal to the maximum limit.", parent = self.menus_area)  # show an error message
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_joint_variable_max_limit(self, event = None):  # change the maximum limit of the variable of the chosen joint
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_model - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the model
        ask_joint_variable_max_limit = sd.askfloat("Define variable's maximum limit", f"Enter the maximum limit of the joint {self.chosen_joint_number_model} \nvariable (in {['degrees', 'meters'][joint_type_index]}):", initialvalue = self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][1], minvalue = self.control_joints_variables_extreme_limits[joint_type_index][0], maxvalue = self.control_joints_variables_extreme_limits[joint_type_index][1], parent = self.menus_area)  # ask the user to enter the maximum limit of the joint variable
        if ask_joint_variable_max_limit != None:  # if the user enters a number
            if ask_joint_variable_max_limit >= self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][0]:  # if the maximum limit is greater than or equal to the minimum limit
                self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][1] = ask_joint_variable_max_limit
                if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[1] and self.d_den_har_parameters[self.chosen_joint_number_model - 1] > ask_joint_variable_max_limit:  # if the chosen joint is prismatic and the d parameter is greater than the maximum limit
                    self.d_den_har_parameters[self.chosen_joint_number_model - 1] = ask_joint_variable_max_limit  # change the d parameter for the chosen model joint
                elif self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0] and np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]) > ask_joint_variable_max_limit:  # if the chosen joint is revolute and the theta parameter is greater than the maximum limit
                    self.theta_den_har_parameters[self.chosen_joint_number_model - 1] = np.deg2rad(ask_joint_variable_max_limit)  # change the theta parameter for the chosen model joint
            else:  # if the maximum limit is less than the minimum limit
                ms.showerror("Error", "The maximum limit must be greater than or equal to the minimum limit.", parent = self.menus_area)  # show an error message
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_base_zero_frame_positions(self, event = None):  # change the positions of the robotic manipulator base system (wrt the world frame) and zero frame (wrt the base system)
        base_position_x = sd.askfloat("Base position", "Enter the x position of the robotic manipulator base \nwrt the world frame (in meters):", initialvalue = self.base_position_wrt_world[0], minvalue = self.base_position_limits[0][0], maxvalue = self.base_position_limits[0][1], parent = self.menus_area)  # ask the user to enter the x position of the robotic manipulator base wrt the world frame
        if base_position_x != None:  # if the user enters a number
            self.base_position_wrt_world[0] = base_position_x  # change the x position of the robotic manipulator base
        base_position_y = sd.askfloat("Base position", "Enter the y position of the robotic manipulator base \nwrt the world frame (in meters):", initialvalue = self.base_position_wrt_world[1], minvalue = self.base_position_limits[1][0], maxvalue = self.base_position_limits[1][1], parent = self.menus_area)  # ask the user to enter the y position of the robotic manipulator base wrt the world frame
        if base_position_y != None:  # if the user enters a number
            self.base_position_wrt_world[1] = base_position_y  # change the y position of the robotic manipulator base
        base_position_z = sd.askfloat("Base position", "Enter the z position of the robotic manipulator base \nwrt the world frame (in meters):", initialvalue = self.base_position_wrt_world[2], minvalue = self.base_position_limits[2][0], maxvalue = self.base_position_limits[2][1], parent = self.menus_area)  # ask the user to enter the z position of the robotic manipulator base wrt the world frame
        if base_position_z != None:  # if the user enters a number
            self.base_position_wrt_world[2] = base_position_z  # change the z position of the robotic manipulator base
        zero_frame_position_x = sd.askfloat("Frame \"0\" position", "Enter the x position of frame \"0\" \nwrt the robotic manipulator base system (in meters):", initialvalue = self.zero_frame_position_wrt_base[0], minvalue = self.zero_frame_position_limits[0][0], maxvalue = self.zero_frame_position_limits[0][1], parent = self.menus_area)  # ask the user to enter the x position of the zero frame wrt the base system
        if zero_frame_position_x != None:  # if the user enters a number
            self.zero_frame_position_wrt_base[0] = zero_frame_position_x  # change the x position of the zero frame
        zero_frame_position_y = sd.askfloat("Frame \"0\" position", "Enter the y position of frame \"0\" \nwrt the robotic manipulator base system (in meters):", initialvalue = self.zero_frame_position_wrt_base[1], minvalue = self.zero_frame_position_limits[1][0], maxvalue = self.zero_frame_position_limits[1][1], parent = self.menus_area)  # ask the user to enter the y position of the zero frame wrt the base system
        if zero_frame_position_y != None:  # if the user enters a number
            self.zero_frame_position_wrt_base[1] = zero_frame_position_y  # change the y position of the zero frame
        zero_frame_position_z = sd.askfloat("Frame \"0\" position", "Enter the z position of frame \"0\" \nwrt the robotic manipulator base system (in meters):", initialvalue = self.zero_frame_position_wrt_base[2], minvalue = self.zero_frame_position_limits[2][0], maxvalue = self.zero_frame_position_limits[2][1], parent = self.menus_area)  # ask the user to enter the z position of the zero frame wrt the base system
        if zero_frame_position_z != None:  # if the user enters a number
            self.zero_frame_position_wrt_base[2] = zero_frame_position_z  # change the z position of the zero frame
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_base_zero_frame_orientations(self, event = None):  # change the orientations of the robotic manipulator base system (wrt the world frame) and zero frame (wrt the base system)
        base_x_rotation = sd.askfloat("Base orientation", "Enter the rotation of the base around world frame's x-axis (in degrees), \nthe first rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.base_orientation_wrt_world[0]), minvalue = self.base_orientation_limits[0][0], maxvalue = self.base_orientation_limits[0][1], parent = self.menus_area)  # ask the user to enter the rotation of the base around x-axis
        if base_x_rotation != None:  # if the user enters a number
            self.base_orientation_wrt_world[0] = np.deg2rad(base_x_rotation)  # change the rotation of the base around world frame's x-axis
        base_y_rotation = sd.askfloat("Base orientation", "Enter the rotation of the base around world frame's y-axis (in degrees), \nthe second rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.base_orientation_wrt_world[1]), minvalue = self.base_orientation_limits[1][0], maxvalue = self.base_orientation_limits[1][1], parent = self.menus_area)  # ask the user to enter the rotation of the base around y-axis
        if base_y_rotation != None:  # if the user enters a number
            self.base_orientation_wrt_world[1] = np.deg2rad(base_y_rotation)  # change the rotation of the base around world frame's y-axis
        base_z_rotation = sd.askfloat("Base orientation", "Enter the rotation of the base around world frame's z-axis (in degrees), \nthe third rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.base_orientation_wrt_world[2]), minvalue = self.base_orientation_limits[2][0], maxvalue = self.base_orientation_limits[2][1], parent = self.menus_area)  # ask the user to enter the rotation of the base around z-axis of the 
        if base_z_rotation != None:  # if the user enters a number
            self.base_orientation_wrt_world[2] = np.deg2rad(base_z_rotation)  # change the rotation of the base around world frame's z-axis
        zero_frame_x_rotation = sd.askfloat("Frame \"0\" orientation", "Enter the rotation of frame \"0\" around the robotic manipulator base system's x-axis (in degrees), \nthe first rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.zero_frame_orientation_wrt_base[0]), minvalue = self.zero_frame_orientation_limits[0][0], maxvalue = self.zero_frame_orientation_limits[0][1], parent = self.menus_area)  # ask the user to enter the rotation of the zero frame around x-axis
        if zero_frame_x_rotation != None:  # if the user enters a number
            self.zero_frame_orientation_wrt_base[0] = np.deg2rad(zero_frame_x_rotation)  # change the rotation of the zero frame around the robotic manipulator base system's x-axis
        zero_frame_y_rotation = sd.askfloat("Frame \"0\" orientation", "Enter the rotation of frame \"0\" around the robotic manipulator base system's y-axis (in degrees), \nthe second rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.zero_frame_orientation_wrt_base[1]), minvalue = self.zero_frame_orientation_limits[1][0], maxvalue = self.zero_frame_orientation_limits[1][1], parent = self.menus_area)  # ask the user to enter the rotation of the zero frame around y-axis
        if zero_frame_y_rotation != None:  # if the user enters a number
            self.zero_frame_orientation_wrt_base[1] = np.deg2rad(zero_frame_y_rotation)  # change the rotation of the zero frame around the robotic manipulator base system's y-axis
        zero_frame_z_rotation = sd.askfloat("Frame \"0\" orientation", "Enter the rotation of frame \"0\" around the robotic manipulator base system's z-axis (in degrees), \nthe third rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.zero_frame_orientation_wrt_base[2]), minvalue = self.zero_frame_orientation_limits[2][0], maxvalue = self.zero_frame_orientation_limits[2][1], parent = self.menus_area)  # ask the user to enter the rotation of the zero frame around z-axis
        if zero_frame_z_rotation != None:  # if the user enters a number
            self.zero_frame_orientation_wrt_base[2] = np.deg2rad(zero_frame_z_rotation)  # change the rotation of the zero frame around the robotic manipulator base system's z-axis
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_end_effector_position(self, event = None):  # change the position of the robotic manipulator end-effector system wrt the last joint frame
        end_effector_position_x = sd.askfloat("End-effector position", "Enter the x position of the robotic manipulator end-effector \nwrt the frame \"n\" (in meters):", initialvalue = self.end_effector_position_wrt_last_frame[0], minvalue = self.end_effector_position_limits[0][0], maxvalue = self.end_effector_position_limits[0][1], parent = self.menus_area)  # ask the user to enter the x position of the robotic manipulator end-effector wrt the last joint frame
        if end_effector_position_x != None:  # if the user enters a number
            self.end_effector_position_wrt_last_frame[0] = end_effector_position_x  # change the x position of the robotic manipulator end-effector wrt the last joint frame
        end_effector_position_y = sd.askfloat("End-effector position", "Enter the y position of the robotic manipulator end-effector \nwrt the frame \"n\" (in meters):", initialvalue = self.end_effector_position_wrt_last_frame[1], minvalue = self.end_effector_position_limits[1][0], maxvalue = self.end_effector_position_limits[1][1], parent = self.menus_area)  # ask the user to enter the y position of the robotic manipulator end-effector wrt the last joint frame
        if end_effector_position_y != None:  # if the user enters a number
            self.end_effector_position_wrt_last_frame[1] = end_effector_position_y  # change the y position of the robotic manipulator end-effector wrt the last joint frame
        end_effector_position_z = sd.askfloat("End-effector position", "Enter the z position of the robotic manipulator end-effector \nwrt the frame \"n\" (in meters):", initialvalue = self.end_effector_position_wrt_last_frame[2], minvalue = self.end_effector_position_limits[2][0], maxvalue = self.end_effector_position_limits[2][1], parent = self.menus_area)  # ask the user to enter the z position of the robotic manipulator end-effector wrt the last joint frame
        if end_effector_position_z != None:  # if the user enters a number
            self.end_effector_position_wrt_last_frame[2] = end_effector_position_z  # change the z position of the robotic manipulator end-effector wrt the last joint frame
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_end_effector_orientation(self, event = None):  # change the orientation of the robotic manipulator end-effector system wrt the last joint frame
        end_effector_x_rotation = sd.askfloat("end-effector orientation", "Enter the rotation of the end-effector around \"n\" frame's x-axis (in degrees), \nthe first rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.end_effector_orientation_wrt_last_frame[0]), minvalue = self.end_effector_orientation_limits[0][0], maxvalue = self.end_effector_orientation_limits[0][1], parent = self.menus_area)  # ask the user to enter the rotation of the end-effector around x-axis
        if end_effector_x_rotation != None:  # if the user enters a number
            self.end_effector_orientation_wrt_last_frame[0] = np.deg2rad(end_effector_x_rotation)  # change the rotation of the end-effector around last joint frame's x-axis
        end_effector_y_rotation = sd.askfloat("end-effector orientation", "Enter the rotation of the end-effector around \"n\" frame's y-axis (in degrees), \nthe second rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.end_effector_orientation_wrt_last_frame[1]), minvalue = self.end_effector_orientation_limits[1][0], maxvalue = self.end_effector_orientation_limits[1][1], parent = self.menus_area)  # ask the user to enter the rotation of the end-effector around y-axis
        if end_effector_y_rotation != None:  # if the user enters a number
            self.end_effector_orientation_wrt_last_frame[1] = np.deg2rad(end_effector_y_rotation)  # change the rotation of the end-effector around last joint frame's y-axis
        end_effector_z_rotation = sd.askfloat("end-effector orientation", "Enter the rotation of the end-effector around \"n\" frame's z-axis (in degrees), \nthe third rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.end_effector_orientation_wrt_last_frame[2]), minvalue = self.end_effector_orientation_limits[2][0], maxvalue = self.end_effector_orientation_limits[2][1], parent = self.menus_area)  # ask the user to enter the rotation of the end-effector around z-axis
        if end_effector_z_rotation != None:  # if the user enters a number
            self.end_effector_orientation_wrt_last_frame[2] = np.deg2rad(end_effector_z_rotation)  # change the rotation of the end-effector around last joint frame's z-axis
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def build_robotic_manipulator_model(self, event = None):  # apply the changes to the current robotic manipulator model, shown also in the visualization
        if self.robotic_manipulator_model_name == "":  # if the robotic manipulator model name is empty
            ms.showerror("Error", "You should first give a name to the robotic manipulator model, in order to build it!", parent = self.menus_area)  # show an error message
        else:  # if the robotic manipulator model name is not empty
            robotic_manipulator_joints_parameters = []  # the parameters of the robotic manipulator joints
            for joint in range(self.joints_number):  # iterate through all joints of the robotic manipulator
                if self.joints_types[joint] == self.joints_types_list[1]:  # if the current joint is prismatic
                    robotic_manipulator_joints_parameters.append(rtb.PrismaticDH(a = self.a_den_har_parameters[joint], alpha = self.alpha_den_har_parameters[joint], theta = self.theta_den_har_parameters[joint], offset = self.d_den_har_parameters[joint], qlim = self.control_joints_variables_limits[joint][1]))  # add the prismatic joint to the list of the robotic manipulator joints parameters
                else:  # if the current joint is revolute
                    robotic_manipulator_joints_parameters.append(rtb.RevoluteDH(a = self.a_den_har_parameters[joint], alpha = self.alpha_den_har_parameters[joint], d = self.d_den_har_parameters[joint], offset = self.theta_den_har_parameters[joint], qlim = np.deg2rad(self.control_joints_variables_limits[joint][0])))  # add the revolute joint to the list of the robotic manipulator joints parameters
            robot_base_T = self.get_transformation_matrix(self.base_position_wrt_world, self.base_orientation_wrt_world) * \
                            self.get_transformation_matrix(self.zero_frame_position_wrt_base, self.zero_frame_orientation_wrt_base)  # set the base of the robotic manipulator model
            robot_end_effector_T = self.get_transformation_matrix(self.end_effector_position_wrt_last_frame, self.end_effector_orientation_wrt_last_frame)  # set the end-effector of the robotic manipulator model
            self.built_robotic_manipulator = rtb.DHRobot(links = robotic_manipulator_joints_parameters, base = robot_base_T, tool = robot_end_effector_T, name = self.robotic_manipulator_model_name)  # build the robotic manipulator model
            self.built_robotic_manipulator.q = self.get_robot_joints_variables(self.control_or_kinematics_variables_visualization)  # set the current joints variables (to be visualized) to the robotic manipulator model
            self.built_robotic_manipulator_info["name"] = self.robotic_manipulator_model_name  # set the name of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_number"] = self.joints_number  # set the number of the joints of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_types"] = copy.deepcopy(self.joints_types)  # set the types of the joints of the robotic manipulator model
            self.built_robotic_manipulator_info["base_position_wrt_world"] = np.copy(self.base_position_wrt_world)  # set the position of the robotic manipulator base wrt the world frame
            self.built_robotic_manipulator_info["base_orientation_wrt_world"] = np.copy(self.base_orientation_wrt_world)  # set the orientation of the robotic manipulator base wrt the world frame
            self.built_robotic_manipulator_info["zero_frame_position_wrt_base"] = np.copy(self.zero_frame_position_wrt_base)  # set the position of the robotic manipulator zero frame wrt the base system
            self.built_robotic_manipulator_info["zero_frame_orientation_wrt_base"] = np.copy(self.zero_frame_orientation_wrt_base)  # set the orientation of the robotic manipulator zero frame wrt the base system
            self.built_robotic_manipulator_info["end_effector_position_wrt_last_frame"] = np.copy(self.end_effector_position_wrt_last_frame)  # set the position of the robotic manipulator end-effector wrt the last joint frame
            self.built_robotic_manipulator_info["end_effector_orientation_wrt_last_frame"] = np.copy(self.end_effector_orientation_wrt_last_frame)  # set the orientation of the robotic manipulator end-effector wrt the last joint frame
            self.built_robotic_manipulator_info["a_den_har_parameters"] = copy.deepcopy(self.a_den_har_parameters)  # set the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.built_robotic_manipulator_info["alpha_den_har_parameters"] = copy.deepcopy(self.alpha_den_har_parameters)  # set the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.built_robotic_manipulator_info["d_den_har_parameters"] = copy.deepcopy(self.d_den_har_parameters)  # set the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.built_robotic_manipulator_info["theta_den_har_parameters"] = copy.deepcopy(self.theta_den_har_parameters)  # set the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.built_robotic_manipulator_info["frames_origins_colors"] = copy.deepcopy(self.frames_origins_colors)  # set the colors of the frames of the robotic manipulator model
            self.built_robotic_manipulator_info["frames_origins_sizes"] = copy.deepcopy(self.frames_origins_sizes)  # set the sizes of the frames of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_z_axis_positions"] = copy.deepcopy(self.joints_z_axis_positions)  # set the z-axis positions of the joints of the robotic manipulator model
            self.built_robotic_manipulator_info["links_colors"] = copy.deepcopy(self.links_colors)  # set the colors of the links of the robotic manipulator model
            self.built_robotic_manipulator_info["links_sizes"] = copy.deepcopy(self.links_sizes)  # set the sizes of the links of the robotic manipulator model
            self.built_robotic_manipulator_info["initial_links_lengths"] = copy.deepcopy(self.initial_links_lengths)  # set the initial lengths of the links of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_motors"] = copy.deepcopy(self.joints_motors_list[:])  # set the motors of the joints of the robotic manipulator model
            self.built_robotic_manipulator_info["joints_motors_mult_factors"] = copy.deepcopy(self.joints_motors_mult_factors[:])  # set the motors multiplication factors of the joints of the robotic manipulator model
            control_joints_variables_limits = []  # the limits of the control variables of the joints of the robotic manipulator model
            for k in range(self.joints_number):  # iterate through all joints of the robotic manipulator
                if self.joints_types[k] == self.joints_types_list[0]: control_joints_variables_limits.append(self.control_joints_variables_limits[k][0])  # if the current joint is revolute
                elif self.joints_types[k] == self.joints_types_list[1]: control_joints_variables_limits.append(self.control_joints_variables_limits[k][1])  # if the current joint is prismatic
            self.built_robotic_manipulator_info["control_joints_variables_limits"] = control_joints_variables_limits  # set the limits of the control variables of the joints of the robotic manipulator model
            self.robotic_manipulator_is_built = True  # the robotic manipulator model has been built
            self.robot_joints_control_law_output = []  # reset the robot joints control law output
            print(f"\nRobotic manipulator name: {self.robotic_manipulator_model_name}\n", self.built_robotic_manipulator)  # print the robotic manipulator model
            ms.showinfo("Robotic manipulator model built", f"The \"{self.robotic_manipulator_model_name}\" robotic manipulator model has been built successfully!", parent = self.menus_area)  # show an information message
            self.update_model_visualization_indicators()  # update the model and visualization indicators
    def reload_built_robotic_manipulator_info_data(self, event = None):  # reload the data of the built robotic manipulator model, if any
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            self.robotic_manipulator_model_name = self.built_robotic_manipulator_info["name"]  # set the name of the robotic manipulator model
            self.joints_number = self.built_robotic_manipulator_info["joints_number"]  # set the total number of joints of the robotic manipulator
            self.frames_number, self.links_number = self.set_frames_links_number(joints_number = self.joints_number)  # set the total number of frames and links of the robotic manipulator
            self.joints_types = copy.deepcopy(self.built_robotic_manipulator_info["joints_types"])  # set the types of the joints of the robotic manipulator model
            self.base_position_wrt_world = np.copy(self.built_robotic_manipulator_info["base_position_wrt_world"])  # set the position of the robotic manipulator base wrt the world frame
            self.base_orientation_wrt_world = np.copy(self.built_robotic_manipulator_info["base_orientation_wrt_world"])  # set the orientation of the robotic manipulator base wrt the world frame
            self.zero_frame_position_wrt_base = np.copy(self.built_robotic_manipulator_info["zero_frame_position_wrt_base"])  # set the position of the robotic manipulator zero frame wrt the base system
            self.zero_frame_orientation_wrt_base = np.copy(self.built_robotic_manipulator_info["zero_frame_orientation_wrt_base"])  # set the orientation of the robotic manipulator zero frame wrt the base system
            self.end_effector_position_wrt_last_frame = np.copy(self.built_robotic_manipulator_info["end_effector_position_wrt_last_frame"])  # set the position of the robotic manipulator end-effector wrt the last joint frame
            self.end_effector_orientation_wrt_last_frame = np.copy(self.built_robotic_manipulator_info["end_effector_orientation_wrt_last_frame"])  # set the orientation of the robotic manipulator end-effector wrt the last joint frame
            self.a_den_har_parameters = copy.deepcopy(self.built_robotic_manipulator_info["a_den_har_parameters"])  # set the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.alpha_den_har_parameters = copy.deepcopy(self.built_robotic_manipulator_info["alpha_den_har_parameters"])  # set the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.d_den_har_parameters = copy.deepcopy(self.built_robotic_manipulator_info["d_den_har_parameters"])  # set the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.theta_den_har_parameters = copy.deepcopy(self.built_robotic_manipulator_info["theta_den_har_parameters"])  # set the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
            self.frames_origins_colors = copy.deepcopy(self.built_robotic_manipulator_info["frames_origins_colors"])  # set the colors of the frames of the robotic manipulator model
            self.frames_origins_sizes = copy.deepcopy(self.built_robotic_manipulator_info["frames_origins_sizes"])  # set the sizes of the frames of the robotic manipulator model
            self.joints_z_axis_positions = copy.deepcopy(self.built_robotic_manipulator_info["joints_z_axis_positions"])  # set the z-axis positions of the joints of the robotic manipulator model
            self.initial_links_lengths = copy.deepcopy(self.built_robotic_manipulator_info["initial_links_lengths"])  # set the initial lengths of the links of the robotic manipulator model
            self.links_colors = copy.deepcopy(self.built_robotic_manipulator_info["links_colors"])  # set the colors of the links of the robotic manipulator model
            self.links_sizes = copy.deepcopy(self.built_robotic_manipulator_info["links_sizes"])  # set the sizes of the links of the robotic manipulator model
            self.joints_motors_list = copy.deepcopy(self.built_robotic_manipulator_info["joints_motors"])  # set the motors of the joints of the robotic manipulator model
            self.joints_motors_mult_factors = copy.deepcopy(self.built_robotic_manipulator_info["joints_motors_mult_factors"])  # set the motors multiplication factors of the joints of the robotic manipulator model
            control_joints_variables_limits = copy.deepcopy(self.control_joints_variables_limits)  # initialize the limits of the control variables of the joints of the robotic manipulator model
            for k in range(self.joints_number):  # iterate through all joints of the robotic manipulator
                if self.joints_types[k] == self.joints_types_list[0]: control_joints_variables_limits[k][0] = copy.deepcopy(self.built_robotic_manipulator_info["control_joints_variables_limits"][k])  # if the current joint is revolute
                elif self.joints_types[k] == self.joints_types_list[1]: control_joints_variables_limits[k][1] = copy.deepcopy(self.built_robotic_manipulator_info["control_joints_variables_limits"][k])  # if the current joint is prismatic
            self.control_joints_variables_limits = control_joints_variables_limits  # set the limits of the control variables of the joints of the robotic manipulator model
    def destroy_robotic_manipulator_model(self, event = None):  # destroy the current loaded and built robotic manipulator model
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            previous_robotic_manipulator_name = self.built_robotic_manipulator_info['name']
            self.built_robotic_manipulator = None  # destroy the robotic manipulator model
            self.built_robotic_manipulator_info = {}  # clear the robotic manipulator model info
            self.robotic_manipulator_is_built = False  # no robotic manipulator model has been built
            ms.showinfo("Robotic manipulator model destroyed", f"The \"{previous_robotic_manipulator_name}\" robotic manipulator model has been destroyed successfully!", parent = self.menus_area)  # show an information message
    def show_robotic_manipulator_model_info(self, event = None):  # show to the user the current robotic manipulator model
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            ms.showinfo("Robotic manipulator model info (in SI units where is needed)", \
                f"Name: {self.built_robotic_manipulator_info['name']}\n\
--- Cordinates systems ---\n\
• Base position wrt world (in m): {self.built_robotic_manipulator_info['base_position_wrt_world']}\nBase orientation (in degrees) wrt world (roll-pitch-yaw): {np.rad2deg(self.built_robotic_manipulator_info['base_orientation_wrt_world'])}\n\
• Frame \"0\" position wrt base (in m): {self.built_robotic_manipulator_info['zero_frame_position_wrt_base']}\nFrame \"0\" orientation (in degrees) wrt base (roll-pitch-yaw): {np.rad2deg(self.built_robotic_manipulator_info['zero_frame_orientation_wrt_base'])}\n\
• End-effector position wrt frame \"n\" (in m): {self.built_robotic_manipulator_info['end_effector_position_wrt_last_frame']}\nEnd-effector orientation (in degrees) wrt frame \"n\" (roll-pitch-yaw): {np.rad2deg(self.built_robotic_manipulator_info['end_effector_orientation_wrt_last_frame'])}\n\
--- Joints and frames parameters ---\n\
• Joints number: {self.built_robotic_manipulator_info['joints_number']}\n\
• Joints types: {self.built_robotic_manipulator_info['joints_types']}\n\
• Joints a parameters (in m): {self.built_robotic_manipulator_info['a_den_har_parameters']}\n\
• Joints alpha parameters (in degrees): {np.rad2deg(self.built_robotic_manipulator_info['alpha_den_har_parameters'])}\n\
• Joints d parameters (in m): {self.built_robotic_manipulator_info['d_den_har_parameters']}\n\
• Joints theta parameters (in degrees): {np.rad2deg(self.built_robotic_manipulator_info['theta_den_har_parameters'])}\n\
• Joints frames z-axis positions (in m): {self.built_robotic_manipulator_info['joints_z_axis_positions']}\n\
• Frames origins colors: {self.built_robotic_manipulator_info['frames_origins_colors']}\n\
• Frames origins sizes: {self.built_robotic_manipulator_info['frames_origins_sizes']}\n\
--- Links parameters ---\n\
• Links colors: {self.built_robotic_manipulator_info['links_colors']}\n\
• Links sizes: {self.built_robotic_manipulator_info['links_sizes']}\n\
• Links lengths initial (in m): {self.built_robotic_manipulator_info['initial_links_lengths']}\n\
--- Control parameters ---\n\
• Joints control variables limits (in degrees): {self.built_robotic_manipulator_info['control_joints_variables_limits']}\n\
• Joints motors: {self.built_robotic_manipulator_info['joints_motors']}\n\
• Joints motors multiplication factors: {self.built_robotic_manipulator_info['joints_motors_mult_factors']}", parent = self.menus_area)  # show the robotic manipulator model info
        else:  # if the robotic manipulator model has not been built
            ms.showerror("Error", "There is no robotic manipulator model built yet!", parent = self.menus_area)  # show an error message
    def save_robotic_manipulator_model_file(self, event = None):  # save the current robotic manipulator model
        if self.robotic_manipulator_model_name == "":  # if the robotic manipulator model name is empty
            ms.showerror("Error", "You should first give a name to the robotic manipulator model, in order to save it!", parent = self.menus_area)  # show an error message
        else:  # if the robotic manipulator model name is not empty
            ask_file_name = sd.askstring("Save robotic manipulator model", f"Enter the name of the file to save the robotic manipulator model (the model name -\"{self.robotic_manipulator_model_name}\"- \nyou specified will remain as it is right now, regardless of the file's name):", initialvalue = self.robotic_manipulator_model_name, parent = self.menus_area)  # ask the user to enter the name of the file to save the robotic manipulator model
            if ask_file_name != None and ask_file_name != "":  # if the user enters a name for the file
                overwrite_file_accept = False  # the user's choice to overwrite the file
                if (ask_file_name + ".txt") in os.listdir(self.saved_robotic_manipulators_folder_path):  # if there is already a file named exactly like this
                    overwrite_file_accept = ms.askyesno("Asking permission to overwrite file", "There is already a file named exactly like this. Do you want to overwrite it?")  # ask the user if they want to overwrite the file
                if (ask_file_name + ".txt") not in os.listdir(self.saved_robotic_manipulators_folder_path) or overwrite_file_accept:  # if the file does not exist or the user wants to overwrite it
                    model_file = open(self.saved_robotic_manipulators_folder_path + fr"/{ask_file_name}.txt", "w", encoding = "utf-8")  # the path of the file where the robotic manipulator model will be saved
                    model_file.write(f"Name: {self.robotic_manipulator_model_name}\n")  # write the name of the robotic manipulator model
                    model_file.write(f"Joints number: {self.joints_number}\n")  # write the number of the joints of the robotic manipulator model
                    model_file.write(f"Base position wrt world: {self.base_position_wrt_world[0]:.3f} {self.base_position_wrt_world[1]:.3f} {self.base_position_wrt_world[2]:.3f}\n")  # write the position of the robotic manipulator base wrt the world frame
                    model_file.write(f"Base orientation wrt world: {np.rad2deg(self.base_orientation_wrt_world[0]):.3f} {np.rad2deg(self.base_orientation_wrt_world[1]):.3f} {np.rad2deg(self.base_orientation_wrt_world[2]):.3f}\n")  # write the orientation of the robotic manipulator base wrt the world frame
                    model_file.write(f"Frame \"0\" position wrt base: {self.zero_frame_position_wrt_base[0]:.3f} {self.zero_frame_position_wrt_base[1]:.3f} {self.zero_frame_position_wrt_base[2]:.3f}\n")  # write the position of the robotic manipulator zero frame wrt the base system
                    model_file.write(f"Frame \"0\" orientation wrt base: {np.rad2deg(self.zero_frame_orientation_wrt_base[0]):.3f} {np.rad2deg(self.zero_frame_orientation_wrt_base[1]):.3f} {np.rad2deg(self.zero_frame_orientation_wrt_base[2]):.3f}\n")  # write the orientation of the robotic manipulator zero frame wrt the base system
                    model_file.write(f"End-effector position wrt frame \"n\": {self.end_effector_position_wrt_last_frame[0]:.3f} {self.end_effector_position_wrt_last_frame[1]:.3f} {self.end_effector_position_wrt_last_frame[2]:.3f}\n")  # write the position of the robotic manipulator end-effector wrt the last joint frame
                    model_file.write(f"End-effector orientation wrt frame \"n\": {np.rad2deg(self.end_effector_orientation_wrt_last_frame[0]):.3f} {np.rad2deg(self.end_effector_orientation_wrt_last_frame[1]):.3f} {np.rad2deg(self.end_effector_orientation_wrt_last_frame[2]):.3f}\n")  # write the orientation of the robotic manipulator end-effector wrt the last joint frame
                    for k in range(self.joints_number):  # iterate through all the joints of the robotic manipulator
                        joint_type_index = self.joints_types_list.index(self.joints_types[k])  # the index of the current joint type (0 for revolute and 1 for prismatic) for the model
                        joint_motors = ", ".join(self.joints_motors_list[k])  # the motors of the current joint
                        joint_motors_mult_factors = ", ".join([str(float(num)) for num in self.joints_motors_mult_factors[k]])  # the multiplication factors of the motors of the current joint
                        model_file.write(f"Joint {k + 1} type: {self.joints_types[k]}\n")  # write the type of the current joint
                        model_file.write(f"Joint {k + 1} Denavit - Hartenberg parameters: {self.a_den_har_parameters[k]:.3f} {np.rad2deg(self.alpha_den_har_parameters[k]):.3f} {self.d_den_har_parameters[k]:.3f} {np.rad2deg(self.theta_den_har_parameters[k]):.3f}\n")  # write the Denavit - Hartenberg parameters of the current joint
                        model_file.write(f"Joint {k + 1} variable limits: {self.control_joints_variables_limits[k][joint_type_index][0]:.3f} {self.control_joints_variables_limits[k][joint_type_index][1]:.3f}\n")  # write the variable limits of the current joint
                        model_file.write(f"Joint {k + 1} motors: {joint_motors}\n")  # write the motors of the current joint
                        model_file.write(f"Joint {k + 1} motors mult. factors: {joint_motors_mult_factors}\n")  # write the multiplication factors of the motors of the current joint
                        model_file.write(f"Joint {k + 1} z-axis position: {self.joints_z_axis_positions[k]:.3f}\n")  # write the z-axis position of the current joint (along the z-axis of its frame)
                    for k in range(self.frames_number):  # itearate through all the frames of the robotic manipulator
                        frame = (["base"] + [f"{i}" for i in range(self.links_number)] + ["end-effector"])[k]  # the name of the current frame
                        model_file.write(f"Frame \"{frame}\" origin color: {self.frames_origins_colors[k]}\n")  # write the color of the current frame origin
                        model_file.write(f"Frame \"{frame}\" origin size: {self.frames_origins_sizes[k]}\n")  # write the size of the current frame origin
                        if frame != f"{self.joints_number}" and frame != "end-effector":  # if the current frame is not the "n" frame nor the end-effector
                            model_file.write(f"Frame \"{frame}\" joint's link color: {self.links_colors[k]}\n")  # write the color of the current link
                            model_file.write(f"Frame \"{frame}\" joint's link size: {self.links_sizes[k]}\n")  # write the size of the current link
                            model_file.write(f"Frame \"{frame}\" initial joint's link length: {self.initial_links_lengths[k]:.3f}\n")  # write the initial length of the current link
                    model_file.close()  # close the file where the robotic manipulator model is saved
                    if ask_file_name not in self.saved_robots_models_files_list:  # if the name of the saved file is not in the values of the combobox that contains the names of the saved files
                        self.saved_robots_models_files_list.append(ask_file_name)  # add the name of the saved file to the list of the saved files
                    ms.showinfo("Robot model saved!", f"The robotic manipulator model has been saved successfully in the file \"{ask_file_name}.txt\"!", parent = self.menus_area)  # show an information message
                    self.update_model_visualization_indicators()  # update the model and visualization indicators
                else:  # if the file exists and the user does not want to overwrite it
                    ms.showinfo("File not saved", "The file has not been saved!", parent = self.menus_area)  # show an information message
            else:  # if the user does not enter a name for the file
                ms.showerror("Error", "You have not entered a name for the file!", parent = self.menus_area)  # show an error message
    def delete_robotic_manipulator_model_file(self, event = None):  # delete a saved robotic manipulator model
        file_to_delete = sd.askstring("Delete robotic manipulator model", "Enter the name of the file to delete the robotic manipulator model:", parent = self.menus_area)  # ask the user to enter the name of the file to delete the robotic manipulator model
        if file_to_delete != None and file_to_delete != "":  # if the user enters a name for the file
            if file_to_delete in self.saved_robots_models_files_list:  # if the chosen file exists
                delete_file_accept = ms.askyesno("Confirm deletion", f"Are you sure you want to delete the file \"{file_to_delete}.txt\"?")  # ask the user if they want to delete the file
                if delete_file_accept:  # if the user wants to delete the file
                    os.remove(self.saved_robotic_manipulators_folder_path + fr"/{file_to_delete}.txt")  # delete the file
                    self.saved_robots_models_files_list.remove(file_to_delete)  # remove the name of the deleted file from the list of the saved files
                    ms.showinfo("File deleted!", f"The file \"{file_to_delete}.txt\" has been deleted successfully!", parent = self.menus_area)  # show an information message
                    self.update_model_visualization_indicators()  # update the model and visualization indicators
            else:  # if the file does not exist
                ms.showerror("Error", "The file does not exist!", parent = self.menus_area)  # show an error message
        else:  # if the user does not enter a name for the file
            ms.showerror("Error", "You have not entered a name for the file!", parent = self.menus_area)  # show an error message
    def load_robotic_manipulator_model(self, robot_model_name, event = None):  # load a saved robotic manipulator model
        load_file_accept = False  # the user's choice to load the file
        if event != None:  # if the function is called by an event
            loaded_robot_file = self.load_model_combobox.get()  # the name of the chosen loaded file that contains the robotic manipulator model
        else:
            loaded_robot_file = robot_model_name  # the name of the chosen loaded file that contains the robotic manipulator model
        if loaded_robot_file in self.saved_robots_models_files_list:  # if the chosen file exists
            if event != None:  # if the function is called by an event
                load_file_accept = ms.askyesno("Confirm loading", f"Are you sure you want to load the file \"{loaded_robot_file}.txt\"?")  # ask the user if they want to load the chosen file
            else:
                load_file_accept = True  # the file is loaded automatically
            if load_file_accept:  # if the user wants to load the chosen file
                self.destroy_robotic_manipulator_model()  # destroy the current built robotic manipulator model
                model_file = open(self.saved_robotic_manipulators_folder_path + fr"/{loaded_robot_file}.txt", "r", encoding = "utf-8")  # open the file that contains the robotic manipulator model
                model_file_lines = model_file.readlines()  # read all the lines of the file
                model_file.close()  # close the file
                self.robotic_manipulator_model_name = model_file_lines[0].split(": ")[1].strip()  # the name of the robotic manipulator model
                self.joints_number = int(model_file_lines[1].split(": ")[1].strip())  # set the total number of joints
                systems_first_line_index = 2  # the index of the first line of the base and end-effector systems in the file
                self.frames_number, self.links_number = self.set_frames_links_number(joints_number = self.joints_number)  # set the total number of frames and links of the robotic manipulator
                self.base_position_wrt_world = np.array([float(value) for value in model_file_lines[systems_first_line_index].split(": ")[1].strip().split(" ")])  # the position of the robotic manipulator base
                self.base_orientation_wrt_world = np.array([np.deg2rad(float(value)) for value in model_file_lines[systems_first_line_index + 1].split(": ")[1].strip().split(" ")])  # the orientation of the robotic manipulator base
                self.zero_frame_position_wrt_base = np.array([float(value) for value in model_file_lines[systems_first_line_index + 2].split(": ")[1].strip().split(" ")])  # the position of the robotic manipulator zero frame
                self.zero_frame_orientation_wrt_base = np.array([np.deg2rad(float(value)) for value in model_file_lines[systems_first_line_index + 3].split(": ")[1].strip().split(" ")])  # the orientation of the robotic manipulator zero frame
                self.end_effector_position_wrt_last_frame = np.array([float(value) for value in model_file_lines[systems_first_line_index + 4].split(": ")[1].strip().split(" ")])  # the position of the robotic manipulator end-effector
                self.end_effector_orientation_wrt_last_frame = np.array([np.deg2rad(float(value)) for value in model_file_lines[systems_first_line_index + 5].split(": ")[1].strip().split(" ")])  # the orientation of the robotic manipulator end-effector
                self.joints_types = []  # the types of the joints of the robotic manipulator model
                self.a_den_har_parameters = []  # the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                self.alpha_den_har_parameters = []  # the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                self.d_den_har_parameters = []  # the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                self.theta_den_har_parameters = []  # the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                self.joints_z_axis_positions = []  # the z-axis positions of the joints of the robotic manipulator model
                self.joints_motors_list = []  # the motors of the joints of the robotic manipulator model
                self.joints_motors_mult_factors = []  # the multiplication factors of the motors of the joints of the robotic manipulator model
                self.control_joints_variables_limits = [copy.deepcopy(self.control_joints_variables_extreme_limits) for _ in range(self.joints_number)]  # the limits of the control joints variables in degrees or meters, depending on the joint's type
                joints_first_line_index = 8  # the index of the first line of the joints in the file
                lines_number_for_each_joint = 6  # the number of lines for each joint separately in the file
                for k in range(self.joints_number):  # iterate through all joints of the robotic manipulator model
                    self.joints_types.append(model_file_lines[joints_first_line_index + lines_number_for_each_joint * k].split(": ")[1].strip())  # the type of the current joint
                    self.a_den_har_parameters.append(float(model_file_lines[(joints_first_line_index + 1) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")[0]))  # the a parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                    self.alpha_den_har_parameters.append(np.deg2rad(float(model_file_lines[(joints_first_line_index + 1) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")[1])))  # the alpha parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                    self.d_den_har_parameters.append(float(model_file_lines[(joints_first_line_index + 1) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")[2]))  # the d parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                    self.theta_den_har_parameters.append(np.deg2rad(float(model_file_lines[(joints_first_line_index + 1) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")[3])))  # the theta parameters of the Denavit - Hartenberg parameters of the robotic manipulator model
                    self.control_joints_variables_limits[k][self.joints_types_list.index(self.joints_types[k])] = [float(value) for value in model_file_lines[(joints_first_line_index + 2) + lines_number_for_each_joint * k].split(": ")[1].strip().split(" ")]  # the limits of the variables of the current joint
                    self.joints_motors_list.append(model_file_lines[(joints_first_line_index + 3) + lines_number_for_each_joint * k].split(": ")[1].strip().split(", "))  # the motors of the current joint
                    self.joints_motors_mult_factors.append([float(num) for num in model_file_lines[(joints_first_line_index + 4) + lines_number_for_each_joint * k].split(": ")[1].strip().split(", ")])  # the multiplication factors of the motors of the current joint
                    self.joints_z_axis_positions.append(float(model_file_lines[(joints_first_line_index + 5) + lines_number_for_each_joint * k].split(": ")[1].strip()))  # the z-axis position of the current joint (along the z-axis of its frame)
                self.frames_origins_colors = []  # the colors of the frames origins of the robotic manipulator model
                self.frames_origins_sizes = []  # the sizes of the frames origins of the robotic manipulator model
                self.links_colors = []  # the colors of the links of the robotic manipulator model
                self.links_sizes = []  # the sizes of the links of the robotic manipulator model
                self.initial_links_lengths = []  # the initial lengths of the links of the robotic manipulator model
                frames_line_index = joints_first_line_index + self.joints_number * lines_number_for_each_joint  # the index of the first line of the frames in the file
                lines_number_for_each_frame = 5  # the number of lines for each frame separately in the file
                for k in range(self.links_number):  # iterate through the first frames (base and joints frames) of the robotic manipulator
                    self.frames_origins_colors.append(model_file_lines[frames_line_index + lines_number_for_each_frame * k].split(": ")[1].strip())  # the color of the current frame origin
                    self.frames_origins_sizes.append(int(model_file_lines[frames_line_index + lines_number_for_each_frame * k + 1].split(": ")[1].strip()))  # the size of the current frame origin
                    self.links_colors.append(model_file_lines[frames_line_index + lines_number_for_each_frame * k + 2].split(": ")[1].strip())  # the color of the current link
                    self.links_sizes.append(int(model_file_lines[frames_line_index + lines_number_for_each_frame * k + 3].split(": ")[1].strip()))  # the size of the current link
                    self.initial_links_lengths.append(float(model_file_lines[frames_line_index + lines_number_for_each_frame * k + 4].split(": ")[1].strip()))  # the initial length of the current link
                for k in range(2):  # iterate through the last frames ("n" frame end-effector frame) of the robotic manipulator
                    self.frames_origins_colors.append(model_file_lines[frames_line_index + lines_number_for_each_frame * self.links_number + 2*k].split(": ")[1].strip())  # the color of the current frame origin
                    self.frames_origins_sizes.append(int(model_file_lines[frames_line_index + lines_number_for_each_frame * self.links_number + 2*k+1].split(": ")[1].strip()))  # the size of the current frame origin
                self.control_joints_variables = [0.0 for _ in range(self.joints_number)]  # the variables of the joints of the robotic manipulator model
                self.forward_kinematics_variables = [0.0 for _ in range(self.joints_number)]  # the variables of the forward kinematics of the robotic manipulator model
                self.differential_kinematics_velocities = [0.0 for _ in range(self.joints_number)]  # the velocities of the differential kinematics of the robotic manipulator model
                self.chosen_joint_number_model = 1  # the chosen joint number of the robotic manipulator model
                self.chosen_frame_visualization = "frame 0"  # the chosen frame of the robotic manipulator for visualization
                self.chosen_link_visualization = "link 1"  # the chosen link of the robotic manipulator for visualization
                self.chosen_joint_number_control = 1  # the chosen joint number of the robotic manipulator control
                self.model_name_entrybox.delete(0, "end")  # delete the current name of the robotic manipulator model
                self.model_name_entrybox.insert(0, self.robotic_manipulator_model_name)  # insert the name of the robotic manipulator model
                self.build_robotic_manipulator_model()  # build the robotic manipulator model
        else:  # if the chosen file does not exist
            ms.showerror("Error", f"The chosen robot model \"{loaded_robot_file}.txt\" does not exist!", parent = self.menus_area)  # show an error message
    # functions for the main menu where the robotic manipulator is visualized
    def get_number_from_frame_visualization(self, frame_visualization):  # get the number from the frame visualization
        if frame_visualization == "base": return 0  # if the chosen frame is the base frame
        elif frame_visualization == "end-effector": return self.joints_number + 2  # if the chosen frame is the end-effector frame
        else: return int(frame_visualization.split(" ")[-1]) + 1  # if the chosen frame is a joint frame
    def get_number_from_link_visualization(self, link_visualization):  # get the number from the link visualization
        if link_visualization == "base": return 0  # if the chosen link is the base link
        else: return int(link_visualization.split(" ")[-1])  # if the chosen link is not the base link
    def change_chosen_frame_visualization(self, event = None):  # change the chosen joint number of the robotic manipulator visualization
        self.chosen_frame_visualization = self.choose_frame_visualization_combobox.get()  # change the chosen frame of the robotic manipulator for visualization
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_joint_position(self, event = None):  # change the position of the chosen joint of the robotic manipulator
        if self.chosen_frame_visualization != "base" and self.chosen_frame_visualization != "end-effector" and self.chosen_frame_visualization != f"frame {self.joints_number}":  # if the chosen frame is a joint frame
            chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
            joint_z_axis_position = sd.askfloat("Choose the joint position offset from its frame origin", f"Enter the z-axis position of the joint {chosen_frame_number_visualization} \n(in meters) wrt its frame (\"{self.chosen_frame_visualization}\" frame):", initialvalue = self.joints_z_axis_positions[chosen_frame_number_visualization - 1], minvalue = -self.d_parameters_limits[1], maxvalue = self.d_parameters_limits[1], parent = self.menus_area)
            if joint_z_axis_position != None:  # if the user enters a number
                self.joints_z_axis_positions[chosen_frame_number_visualization - 1] = joint_z_axis_position  # change the position offset of the chosen joint of the robotic manipulator
        else:
            ms.showerror("Error", f"This frame is not related to any joint! Choose one of the frames 0 to {self.joints_number-1}!", parent = self.menus_area)  # show an error message
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_frame_color(self, event = None):  # change the color of the chosen frame origin of the robotic manipulator
        chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
        self.frames_origins_colors[chosen_frame_number_visualization] = cc.askcolor(title = f"Choose the \"{self.chosen_frame_visualization}\" frame origin color", color = self.frames_origins_colors[chosen_frame_number_visualization], parent = self.menus_area)[1]  # change the color of the chosen frame origin of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def apply_color_to_all_frames(self, event = None):  # apply the chosen color to all the frames origins of the robotic manipulator
        chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
        for k in range(self.frames_number):  # iterate through all the frames origins of the robotic manipulator
            self.frames_origins_colors[k] = self.frames_origins_colors[chosen_frame_number_visualization]  # apply the chosen color to all the frames origins
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_frame_size(self, event = None):  # change the size of the chosen frame origin of the robotic manipulator
        chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
        frame_origin_size = sd.askinteger("Choose the frame origin width", f"Enter the relative size of the \"{self.chosen_frame_visualization}\" frame origin\n({self.min_visualization_size} for minimum and {self.max_visualization_size} for maximum):", initialvalue = self.frames_origins_sizes[chosen_frame_number_visualization], minvalue = self.min_visualization_size, maxvalue = self.max_visualization_size, parent = self.menus_area)
        if frame_origin_size != None:  # if the user enters a number
            self.frames_origins_sizes[chosen_frame_number_visualization] = frame_origin_size  # change the size of the chosen frame origin of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def apply_size_to_all_frames(self, event = None):  # apply the chosen size to all the frames origins of the robotic manipulator
        chosen_frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
        for k in range(self.frames_number):  # iterate through all the frames origins of the robotic manipulator
            self.frames_origins_sizes[k] = self.frames_origins_sizes[chosen_frame_number_visualization]  # apply the chosen size to all the frames origins
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_chosen_link_visualization(self, event = None):  # change the chosen link of the robotic manipulator for visualization
        self.chosen_link_visualization = self.choose_link_visualization_combobox.get()  # change the chosen link of the robotic manipulator for visualization
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_link_color(self, event = None):  # change the color of the chosen link of the robotic manipulator
        chosen_link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen frame number
        self.links_colors[chosen_link_number_visualization] = cc.askcolor(title = f"Choose the \"{self.chosen_link_visualization}\" link color", color = self.links_colors[chosen_link_number_visualization], parent = self.menus_area)[1]  # change the color of the chosen link of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def apply_color_to_all_links(self, event = None):  # apply the chosen color to all links of the robotic manipulator
        chosen_link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen frame number
        for k in range(self.links_number):  # iterate through all links of the robotic manipulator
            self.links_colors[k] = self.links_colors[chosen_link_number_visualization]  # apply the chosen color to all the links
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_link_size(self, event = None):  # change the size of the chosen link of the robotic manipulator
        chosen_link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen frame number
        link_size = sd.askinteger("Choose the link width", f"Enter the relative size of the \"{self.chosen_link_visualization}\" link \n({self.min_visualization_size} for minimum and {self.max_visualization_size} for maximum):", initialvalue = self.links_sizes[chosen_link_number_visualization], minvalue = self.min_visualization_size, maxvalue = self.max_visualization_size, parent = self.menus_area)
        if link_size != None:  # if the user enters a number
            self.links_sizes[chosen_link_number_visualization] = link_size  # change the size of the chosen link of the robotic manipulator
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def apply_size_to_all_links(self, event = None):  # apply the chosen size to all links of the robotic manipulator
        chosen_link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen frame number
        for k in range(self.links_number):  # iterate through all links of the robotic manipulator
            self.links_sizes[k] = self.links_sizes[chosen_link_number_visualization]  # apply the chosen size to all the links
        self.update_model_visualization_indicators()  # update the model and visualization indicators
    def change_canvas_color(self, event = None):  # change the color of the canvas where the robotic manipulator is visualized
        self.workspace_canvas_color = cc.askcolor(title = "Choose the canvas color", color = self.workspace_canvas_color, parent = self.menus_area)[1]
        self.workspace_canvas.configure(bg = self.workspace_canvas_color)  # change the color of the canvas where the robotic manipulator is visualized
        self.choose_visualized_variables_button.configure(bg = self.workspace_canvas_color)  # change the bg color of the button that allows the user to choose the visualized variables
        self.visualized_variables_values_indicator.configure(bg = self.workspace_canvas_color)  # change the bg color of the indicator that shows the values of the visualized variables
    def change_terrain_color(self, event = None):  # change the color of the terrain of the workspace
        self.up_side_terrain_color = cc.askcolor(title = "Choose the terrain up side color", color = self.up_side_terrain_color, parent = self.menus_area)[1]
        self.down_side_terrain_color = cc.askcolor(title = "Choose the terrain down side color", color = self.down_side_terrain_color, parent = self.menus_area)[1]
    def change_axis_colors(self, event = None):  # change the color of the axis of the workspace
        self.workspace_axis_colors[0] = cc.askcolor(title = "Choose the x-axis color", color = self.workspace_axis_colors[0], parent = self.menus_area)[1]
        self.workspace_axis_colors[1] = cc.askcolor(title = "Choose the y-axis color", color = self.workspace_axis_colors[1], parent = self.menus_area)[1]
        self.workspace_axis_colors[2] = cc.askcolor(title = "Choose the z-axis color", color = self.workspace_axis_colors[2], parent = self.menus_area)[1]
    def change_axis_size(self, event = None):  # change the size of the axis of the workspace
        axis_sizes = sd.askinteger("Choose the axis size", f"Enter the relative size of the axis \n({self.min_visualization_size} for minimum and {self.max_visualization_size} for maximum):", initialvalue = self.workspace_axis_sizes, minvalue = self.min_visualization_size, maxvalue = self.max_visualization_size, parent = self.menus_area)  # ask the user to enter the size of the axis
        if axis_sizes != None:  # if the user enters a number
            self.workspace_axis_sizes = axis_sizes  # change the size of the axis of the workspace
    def open_matplotlib_simulator(self, event = None):  # open the matplotlib simulator of the robotic manipulator
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            self.built_robotic_manipulator.teach(q = self.built_robotic_manipulator.q, backend = "pyplot", vellipse = True, block = True)  # open the pyplot simulator and allow the user to control the robotic manipulator
        else:
            ms.showerror("Error", "You have not built the robotic manipulator model yet!", parent = self.menus_area)  # show an error message
    def open_close_swift_simulator(self, event = None):  # open the Swift simulator of the robotic manipulator
        if not self.swift_sim_thread_flag:  # if the thread that runs the online Swift simulation is not running
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                if self.built_robotic_manipulator_info["name"] in self.swift_simulated_robots_list:  # if the robotic manipulator model can be simulated in the Swift simulator
                    self.swift_sim_env = swift.Swift()  # create the Swift environment
                    self.swift_sim_env.launch(realtime = True, headless = False, rate = 60)  # launch the Swift simulator environment
                    self.swift_sim_env.sim_time = 0.0
                    self.swift_env_load_robot()  # load the robotic manipulator model inside the Swift simulator environment
                    self.swift_info_label = swift.Label(f"Robotic manipulator name: {self.built_robotic_manipulator_info['name']}. If you want to exit the simulation, do not close the browser tab. Just press again the GUI's button that started the simulation.")
                    self.swift_robot_configuration_label = swift.Label(f"Robot configuration ({self.control_or_kinematics_variables_visualization}): {self.built_robotic_manipulator.q}")
                    self.swift_sim_env.add(self.swift_info_label)  # add a label with some instructions inside the Swift simulator environment
                    self.swift_sim_env.add(self.swift_robot_configuration_label)  # add a label with the robotic manipulator configuration inside the Swift simulator environment
                    self.start_swift_thread()  # start the thread that runs the online Swift simulation
                else:
                    ms.showerror("Error", f"The Swift simulator is available only for the following robots models! \n{self.swift_simulated_robots_list}", parent = self.menus_area)
            else:
                ms.showerror("Error", "You have not built the robotic manipulator model yet!", parent = self.menus_area)  # show an error message
        else:
            self.kill_swift_thread()  # kill the thread that runs the online Swift simulation
    def swift_env_load_robot(self, event = None):  # load the robotic manipulator model in the Swift simulator environment
        robot_name = self.built_robotic_manipulator_info["name"].lower()  # the name of the chosen robotic manipulator model
        robot_model_urdf_file_path = self.saved_robots_descriptions_folder_path + fr"/{robot_name}_description/urdf/{robot_name}.urdf"  # the path of the chosen robotic manipulator urdf/xacro file
        self.swift_robotic_manipulator = rtb.Robot.URDF(robot_model_urdf_file_path)  # load the chosen robotic manipulator model reading the URDF file
        self.swift_sim_env.add(self.swift_robotic_manipulator, robot_alpha = 1.0, collision_alpha = 0.0)  # add the chosen robotic manipulator model inside the Swift simulator environment
    def update_model_visualization_indicators(self, event = None):  # update the model and visualization indicators
        try:
            # model indicators
            self.joints_number_button.configure(text = self.joints_number)  # change the text of the button that shows the joints number of the robotic manipulator
            self.choose_joint_number_model_combobox.set(f"joint {self.chosen_joint_number_model}")  # set the chosen joint number to the chosen joint
            self.choose_joint_number_model_combobox["values"] = [f"joint {joint}" for joint in range(1, self.links_number)]  # change the values of the combobox that shows the joints number of the robotic manipulator model
            self.choose_frame_visualization_combobox.set(f"{self.chosen_frame_visualization}")  # set the chosen joint number to the chosen joint
            self.choose_frame_visualization_combobox["values"] = ["base"] + [f"frame {frame}" for frame in range(self.links_number)] + ["end-effector"]  # change the values of the combobox that shows the joints number of the robotic manipulator visualization
            self.choose_link_visualization_combobox.set(f"{self.chosen_link_visualization}")  # set the chosen link number to the chosen link
            self.choose_link_visualization_combobox["values"] = ["base"] + [f"link {link}" for link in range(1, self.links_number)]  # change the values of the combobox that shows the links number of the robotic manipulator visualization
            self.joints_types_combobox.set(self.joints_types[self.chosen_joint_number_model - 1])  # set the corresponding combobox to the current joint type
            self.a_den_har_parameter_button.configure(text = f"{self.a_den_har_parameters[self.chosen_joint_number_model - 1]:.3f}")  # change the text of the button that shows the a parameter of the Denavit - Hartenberg parameters of the robotic manipulator
            self.alpha_den_har_parameter_button.configure(text = f"{np.rad2deg(self.alpha_den_har_parameters[self.chosen_joint_number_model - 1]):.3f}")  # change the text of the button that shows the alpha parameter of the Denavit - Hartenberg parameters of the robotic manipulator
            if self.joints_types[self.chosen_joint_number_model - 1] == self.joints_types_list[0]:  # if the chosen joint is revolute
                self.d_den_har_parameter_button.configure(text = f"{self.d_den_har_parameters[self.chosen_joint_number_model - 1]:.3f}")  # change the text of the button that shows the d parameter of the Denavit - Hartenberg parameters of the robotic manipulator
                self.theta_den_har_parameter_button.configure(text = f"{np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]):.3f} " + "(var)")  # change the text of the button that shows the theta parameter of the Denavit - Hartenberg parameters of the robotic manipulator
            else:  # if the chosen joint is prismatic
                self.d_den_har_parameter_button.configure(text = f"{self.d_den_har_parameters[self.chosen_joint_number_model - 1]:.3f} " + "(var)")  # change the text of the button that shows the d parameter of the Denavit - Hartenberg parameters of the robotic manipulator
                self.theta_den_har_parameter_button.configure(text = f"{np.rad2deg(self.theta_den_har_parameters[self.chosen_joint_number_model - 1]):.3f}")  # change the text of the button that shows the theta parameter of the Denavit - Hartenberg parameters of the robotic manipulator
            joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_model - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the model
            self.var_limits_label.configure(text = f"Variable limits\n({['degrees', 'meters'][joint_type_index]}):")  # change the text of the label that shows the variable limits of the chosen joint
            self.var_min_limit_button.configure(text = f"{np.round(self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][0], [self.angles_precision, self.distances_precision][joint_type_index]):.3f}")  # change the text of the button that shows the minimum limit of the variable of the chosen joint
            self.var_max_limit_button.configure(text = f"{np.round(self.control_joints_variables_limits[self.chosen_joint_number_model - 1][joint_type_index][1], [self.angles_precision, self.distances_precision][joint_type_index]):.3f}")  # change the text of the button that shows the maximum limit of the variable of the chosen joint
            self.load_model_combobox["values"] = self.saved_robots_models_files_list  # update the values of the combobox that contains the names of the saved files
            # visualization indicators
            frame_number_visualization = self.get_number_from_frame_visualization(self.chosen_frame_visualization)  # get the chosen frame number
            link_number_visualization = self.get_number_from_link_visualization(self.chosen_link_visualization)  # get the chosen link number
            if frame_number_visualization in list(range(1, self.links_number)):  # if the chosen frame is a joint frame
                joint_number_visualization = frame_number_visualization - 1  # get the chosen joint number
                self.change_joint_position_button.configure(text = f"{self.joints_z_axis_positions[joint_number_visualization]:.3f}")  # change the text of the button that shows the position of the chosen joint
            else:
                self.change_joint_position_button.configure(text = "none")  # change the text of the button that shows the position of the chosen joint
            self.change_frame_color_button.configure(text = self.frames_origins_colors[frame_number_visualization])  # change the text of the button that shows the color of the chosen frame origin
            self.change_frame_size_button.configure(text = self.frames_origins_sizes[frame_number_visualization])  # change the text of the button that shows the size of the chosen frame origin
            self.change_link_color_button.configure(text = self.links_colors[link_number_visualization])  # change the text of the button that shows the color of the chosen link
            self.change_link_size_button.configure(text = self.links_sizes[link_number_visualization])  # change the text of the button that shows the size of the chosen link
            frames_origins, frames_orientations = self.get_all_frames_positions_orientations(np.zeros(self.joints_number,))  # get the positions and the orientations of the frames of the robotic manipulator when all the joints are at zero
            robotic_manipulator_links_points = [np.array(frames_origins[0])] + [np.array(frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]) for k in range(self.joints_number)] + [np.array(frames_origins[-1])]  # the points of the links of the robotic manipulator
            self.initial_links_lengths = np.linalg.norm(np.diff(robotic_manipulator_links_points, axis = 0), axis = 1).tolist()  # the initial lengths of the links of the robotic manipulator
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                frames_origins, frames_orientations = self.get_all_frames_positions_orientations(self.built_robotic_manipulator.q)  # get the positions and the orientations of the current frames of the robotic manipulator
            robotic_manipulator_links_points = [np.array(frames_origins[0])] + [np.array(frames_origins[k + 1] + frames_orientations[k + 1][:, 2] * self.joints_z_axis_positions[k]) for k in range(self.joints_number)] + [np.array(frames_origins[-1])]  # the points of the links of the robotic manipulator
            links_lengths = np.linalg.norm(np.diff(robotic_manipulator_links_points, axis = 0), axis = 1)  # the sizes of the links of the robotic manipulator
            self.link_length_indicator.configure(text = f"{links_lengths[link_number_visualization]:.3f}")  # change the text of the button that shows the length of the chosen link
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_model_visualization_indicators: {e}")

    # the functions used for the thread that runs the online swift simulation
    def start_swift_thread(self, event = None):  # start the thread that runs the online Swift simulation
        self.swift_sim_thread_flag = True  # let the swift thread run
        self.swift_sim_thread = threading.Thread(target = self.swift_thread_run_function)  # create the thread for the online Swift simulation
        self.swift_sim_thread.start()  # start the online Swift simulation thread
    def kill_swift_thread(self, event = None):  # kill the thread that runs the online Swift simulation
        self.swift_sim_thread_flag = False  # stop the online Swift simulation thread
    def swift_thread_run_function(self):  # the run function, it continuously runs the online Swift simulation
        while True:  # while the thread is running
            if self.swift_sim_thread_flag:  # if the thread is running
                try:  # try to run the online Swift simulation
                    # for the robotic manipulator
                    self.swift_robotic_manipulator.base = self.get_transformation_matrix(self.built_robotic_manipulator_info["base_position_wrt_world"], self.built_robotic_manipulator_info["base_orientation_wrt_world"])  # set the base pose of the robotic manipulator model
                    self.swift_robotic_manipulator.tool = self.get_transformation_matrix(self.built_robotic_manipulator_info["end_effector_position_wrt_last_frame"], self.built_robotic_manipulator_info["end_effector_orientation_wrt_last_frame"])  # set the end-effector pose of the robotic manipulator model
                    self.swift_robotic_manipulator.q = np.copy(self.built_robotic_manipulator.q)  # set the joints angles of the robotic manipulator model
                    # for the labels
                    joints_configuration_columns_indicator = 1  # the number of rows of the joints configuration indicator
                    joints_configuration = ""  # the configuration of the joints of the robotic manipulator
                    for k in range(self.joints_number):
                        joints_configuration += f"{k + 1}" + [f"(°): {np.rad2deg(self.swift_robotic_manipulator.q[k]):.1f}", f"(m): {self.swift_robotic_manipulator.q[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])]  # the joints configuration indicator of the robotic manipulator
                        if k < self.joints_number - 1: joints_configuration += ",   "
                        if (k + 1) % joints_configuration_columns_indicator == 0 and (k + 1) != self.joints_number: joints_configuration += " "
                    self.swift_robot_configuration_label.desc = f"Robot configuration ({self.control_or_kinematics_variables_visualization}): {joints_configuration}"  # update the label with the robotic manipulator configuration inside the Swift simulator environment
                    # make the Swift simulation dt step and render the scene
                    self.swift_sim_env.step(dt = self.swift_sim_dt, render = True)  # run the online Swift simulation
                except:  # if an error occurs
                    self.kill_swift_thread()  # kill the thread that runs the online Swift simulation
            else:  # if the thread is stopped
                try: self.swift_sim_env.close()  # close the online Swift simulation environment
                except: pass
                break  # break the while loop
        # self.swift_sim_env.hold()  # hold the online Swift simulation environment

    # functions for the main menu where the robotic manipulator kinematics are analyzed
    # forward kinematics
    def change_chosen_joint_number_fkine(self, event = None):  # change the chosen joint number for the forward kinematics analysis
        self.chosen_joint_number_fkine = int(self.choose_joint_number_fkine_combobox.get().split(" ")[-1])  # change the chosen control joint number of the robotic manipulator
        self.update_forward_kinematics_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_chosen_joint_number_fkine_2(self, event = None):  # change the chosen joint number for the forward kinematics analysis
        self.chosen_joint_number_fkine = self.joint_fkine_value_combobox.current() + 1  # change the chosen control joint number of the robotic manipulator
        self.update_forward_kinematics_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_chosen_fkine_variable(self, event = None):  # change the value of the chosen joint variable for the forward kinematics analysis
        try:  # try to get the entered value
            joint_fkine_value = float(self.joint_fkine_value_combobox.get())  # the fkine variable of the chosen joint
            joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_fkine - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
            self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1] = [np.deg2rad(joint_fkine_value), joint_fkine_value][joint_type_index]  # change the fkine variable of the chosen joint
            control_variable_limits = [np.deg2rad(self.control_joints_variables_limits[self.chosen_joint_number_fkine - 1][0]).tolist(), self.control_joints_variables_limits[self.chosen_joint_number_fkine - 1][1]][joint_type_index]  # the control variable limits of the chosen joint
            if joint_fkine_value < self.control_joints_variables_limits[self.chosen_joint_number_fkine - 1][joint_type_index][0]:  # if the entered value is less than the joint fkine variable minimum limit
                self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1] = control_variable_limits[0]  # change the fkine variable of the chosen joint to the minimum limit
            elif joint_fkine_value > self.control_joints_variables_limits[self.chosen_joint_number_fkine - 1][joint_type_index][1]:  # if the entered value is greater than the joint fkine variable maximum limit
                self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1] = control_variable_limits[1]  # change the fkine variable of the chosen joint to the maximum limit
            self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
        except:  # if the entered value is not a number
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
    def copy_control_to_fkine_values(self, event = None):  # copy the control variables to the forward kinematics variables
        self.forward_kinematics_variables = copy.deepcopy(self.control_joints_variables)  # copy the control variables to the forward kinematics variables
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def change_fkine_variable_slider(self, slider_num, event = None):  # change the value of the chosen joint variable (changing the corresponding slider) for the forward kinematics analysis
        joint_fkine_value = self.fkine_variables_sliders[slider_num].get()  # the fkine variable of the chosen joint
        joint_type_index = self.joints_types_list.index(self.joints_types[slider_num])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
        self.forward_kinematics_variables[slider_num] = [np.deg2rad(joint_fkine_value), joint_fkine_value][joint_type_index]  # change the fkine variable of the chosen joint
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def change_chosen_frame_fkine(self, event = None):  # change the chosen frame for the forward kinematics analysis
        self.chosen_frame_fkine = self.choose_frame_fkine_combobox.get()  # change the chosen frame for the forward kinematics analysis
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def change_orientation_representation_fkine(self, event = None):  # change the orientation representation for the forward kinematics analysis
        self.orientation_representation_fkine = self.frame_orientation_representation_combobox.get()  # change the orientation representation for the forward kinematics analysis
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def set_joints_range_divisions(self, event = None):  # set the divisions of the joints range for the calculation of the robot's reachable workspace
        self.joints_range_divisions = self.alternate_matrix_elements(self.joints_range_divisions_list, self.joints_range_divisions)  # change the divisions of the joints range for the calculation of the robot's reachable workspace
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def compute_plot_reachable_workspace(self, event = None):  # compute and plot the reachable workspace of the robotic manipulator
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            kin.find_reachable_workspace(self.built_robotic_manipulator, self.joints_range_divisions, True)  # compute and plot the reachable workspace of the robotic manipulator
        else:  # if no robotic manipulator is built yet
            ms.showerror("Error", "You have not built a robotic manipulator model yet!", parent = self.menus_area)  # show an error message
    def show_fkine_info(self, event = None):  # show the forward kinematics information
        pass
    def update_forward_kinematics_indicators(self, event = None):  # update the forward kinematics indicators
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_fkine - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
                joint_fkine_value = [f"{np.rad2deg(self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1]):.1f}", f"{self.forward_kinematics_variables[self.chosen_joint_number_fkine - 1]:.3f}"][joint_type_index]  # the fkine value of the chosen joint
                self.choose_joint_number_fkine_combobox.set(f"joint {self.chosen_joint_number_fkine}")  # set the combobox to the chosen joint number for the forward kinematics analysis
                self.joint_fkine_value_combobox.set(joint_fkine_value)  # set the combobox to the chosen joint variable for the forward kinematics analysis
                self.joint_fkine_value_combobox["values"] = [f"{k + 1}: " + [f"{np.rad2deg(self.forward_kinematics_variables[k]):.1f}", f"{self.forward_kinematics_variables[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])] for k in range(self.joints_number)]  # update the values of the joint fkine value combobox
                self.fkine_value_unit_indicator.configure(text = self.joints_types[self.chosen_joint_number_fkine - 1] + " " + ["(°)", "(m)"][joint_type_index])  # change the unit of the forward kinematics value indicator
                self.choose_frame_fkine_combobox.set(self.chosen_frame_fkine)  # set the combobox to the chosen frame for the forward kinematics analysis
                self.frame_orientation_representation_combobox.set(self.orientation_representation_fkine)  # set the combobox to the chosen orientation representation for the forward kinematics analysis
                # solve the forward kinematics of the robotic manipulator for the chosen frame and the chosen joints configuration
                self.fkine_frame_position, fkine_frame_orientation = self.get_fkine_frame_position_orientation(self.forward_kinematics_variables, self.chosen_frame_fkine)  # calculate the position and the orientation of the chosen frame
                self.frame_position_indicator.configure(text = str([np.round(self.fkine_frame_position[k], self.distances_precision) for k in range(len(self.fkine_frame_position))]))
                r = sc.spatial.transform.Rotation.from_matrix(fkine_frame_orientation)  # create a rotation object from the rotation matrix
                if self.orientation_representation_fkine == self.orientation_representations_fkine_list[0]:  # if the orientation representation is in Euler xyz extrinsic angles
                    orient = r.as_euler("xyz")
                    self.fkine_frame_orientation = np.copy(orient)  # set the orientation of the chosen frame for the forward kinematics analysis
                    self.frame_orientation_indicator.configure(text = str([np.round(np.rad2deg(orient[k]), self.angles_precision) for k in range(3)]))
                elif self.orientation_representation_fkine == self.orientation_representations_fkine_list[1]:  # if the orientation representation is in quaternion form
                    orient = r.as_quat()
                    self.frame_orientation_indicator.configure(text = str([np.round(orient[k], 3) for k in range(4)]))
                elif self.orientation_representation_fkine == self.orientation_representations_fkine_list[2]:  # if the orientation representation is in rotation matrix form
                    self.frame_orientation_indicator.configure(text = np.round(fkine_frame_orientation, 3))
                for k in range(self.joints_number):
                    self.fkine_variables_sliders[k].set([np.rad2deg(self.forward_kinematics_variables[k]), self.forward_kinematics_variables[k]][self.joints_types_list.index(self.joints_types[k])])  # set the value of the chosen joint variable (changing the corresponding slider) for the forward kinematics analysis
                self.joints_range_divisions_button.configure(text = f"{self.joints_range_divisions}")  # change the text of the button that shows the divisions of the joints range for the calculation of the robot's reachable workspace
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_forward_kinematics_indicators: {e}")
    # inverse kinematics
    def get_fkine_pose_result(self, event = None):  # get the forward kinematics pose result to the inverse kinematics analysis
        self.chosen_frame_fkine = "end-effector"  # change the chosen frame for the forward kinematics analysis
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
        self.chosen_invkine_3d_position = np.copy(self.fkine_frame_position)  # change the end-effector position for the inverse kinematics analysis
        self.chosen_invkine_orientation = np.copy(self.fkine_frame_orientation)  # change the end-effector orientation for the inverse kinematics analysis
        self.update_inverse_kinematics_indicators()  # update the inverse kinematics indicators
    def send_invkine_joints_config_result(self, event = None):  # send the inverse kinematics joints configuration result to the forward kinematics analysis
        self.chosen_frame_fkine = "end-effector"  # change the chosen frame for the forward kinematics analysis
        self.forward_kinematics_variables = np.copy(self.invkine_joints_configuration).tolist()  # change the joints configuration for the forward kinematics analysis
        self.update_forward_kinematics_indicators()  # update the forward kinematics indicators
    def choose_end_effector_position_invkine(self, event = None):  # choose the end-effector position for the inverse kinematics analysis
        end_effector_pos_x = sd.askfloat("Choose the end-effector x position", "Enter the x position of the end-effector (in meters):", initialvalue = self.chosen_invkine_3d_position[0], parent = self.menus_area)  # ask the user to enter the x position of the end-effector
        if end_effector_pos_x != None:  # if the user enters a number
            self.chosen_invkine_3d_position[0] = end_effector_pos_x  # change the x position of the end-effector
        end_effector_pos_y = sd.askfloat("Choose the end-effector y position", "Enter the y position of the end-effector (in meters):", initialvalue = self.chosen_invkine_3d_position[1], parent = self.menus_area)  # ask the user to enter the y position of the end-effector
        if end_effector_pos_y != None:  # if the user enters a number
            self.chosen_invkine_3d_position[1] = end_effector_pos_y  # change the y position of the end-effector
        end_effector_pos_z = sd.askfloat("Choose the end-effector z position", "Enter the z position of the end-effector (in meters):", initialvalue = self.chosen_invkine_3d_position[2], parent = self.menus_area)  # ask the user to enter the z position of the end-effector
        if end_effector_pos_z != None:  # if the user enters a number
            self.chosen_invkine_3d_position[2] = end_effector_pos_z  # change the z position of the end-effector
        self.update_inverse_kinematics_indicators()  # update the inverse kinematics indicators
    def choose_end_effector_orientation_invkine(self, event = None):  # choose the end-effector orientation for the inverse kinematics analysis
        end_effector_rot_x = sd.askfloat("Choose the end-effector x rotation", "Enter the rotation of the end-effector around world frame's x-axis (in degrees), \nthe first rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.chosen_invkine_orientation[0]), parent = self.menus_area)  # ask the user to enter the x rotation of the end-effector
        if end_effector_rot_x != None:  # if the user enters a number
            self.chosen_invkine_orientation[0] = np.deg2rad(end_effector_rot_x)  # change the x rotation of the end-effector
        end_effector_rot_y = sd.askfloat("Choose the end-effector y rotation", "Enter the rotation of the end-effector around world frame's y-axis (in degrees), \nthe second rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.chosen_invkine_orientation[1]), parent = self.menus_area)  # ask the user to enter the y rotation of the end-effector
        if end_effector_rot_y != None:  # if the user enters a number
            self.chosen_invkine_orientation[1] = np.deg2rad(end_effector_rot_y)  # change the y rotation of the end-effector
        end_effector_rot_z = sd.askfloat("Choose the end-effector z rotation", "Enter the rotation of the end-effector around world frame's z-axis (in degrees), \nthe third rotation that is applied (xyz extrinsic Euler angles):", initialvalue = np.rad2deg(self.chosen_invkine_orientation[2]), parent = self.menus_area)  # ask the user to enter the z rotation of the end-effector
        if end_effector_rot_z != None:  # if the user enters a number
            self.chosen_invkine_orientation[2] = np.deg2rad(end_effector_rot_z)  # change the z rotation of the end-effector
        self.update_inverse_kinematics_indicators()  # update the inverse kinematics indicators
    def change_invkine_tolerance(self, event = None):  # change the tolerance for the inverse kinematics analysis
        invkine_tolerance = sd.askfloat("Choose the inverse kinematics solver tolerance", "Enter the tolerance for the inverse\nkinematics numerical solver:", initialvalue = self.invkine_tolerance, parent = self.menus_area)  # ask the user to enter the tolerance for the inverse kinematics analysis
        if invkine_tolerance != None:  # if the user enters a number
            self.invkine_tolerance = invkine_tolerance  # change the tolerance for the inverse kinematics analysis
        self.update_inverse_kinematics_indicators()  # update the inverse kinematics indicators
    def show_invkine_info(self, event = None):  # show the inverse kinematics information
        pass
    def update_inverse_kinematics_indicators(self, event = None):  # update the inverse kinematics indicators
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                self.choose_end_effector_position_button.configure(text = str([np.round(self.chosen_invkine_3d_position[k], self.distances_precision) for k in range(len(self.chosen_invkine_3d_position))]))  # change the text of the button that allows the user to choose the end-effector position
                self.choose_end_effector_orientation_button.configure(text = str([np.round(np.rad2deg(self.chosen_invkine_orientation[k]), self.angles_precision) for k in range(len(self.chosen_invkine_orientation))]))  # change the text of the button that allows the user to choose the end-effector orientation
                self.choose_invkine_tolerance_button.configure(text = f"{np.round(self.invkine_tolerance, 10)}")  # change the text of the button that allows the user to choose the tolerance for the inverse kinematics solver
                # solve the inverse kinematics of the robotic manipulator for the chosen end-effector position and orientation
                self.invkine_joints_configuration, invkine_success = kin.compute_inverse_kinematics(self.built_robotic_manipulator, self.get_transformation_matrix(self.chosen_invkine_3d_position, self.chosen_invkine_orientation), self.invkine_tolerance)  # compute the inverse kinematics of the robotic manipulator
                joints_configuration = ""  # initialize the joints configuration indicator of the robotic manipulator
                joints_configuration_columns_indicator = 5  # the number of rows of the joints configuration indicator
                if invkine_success:  # if the inverse kinematics analysis is successful
                    for k in range(self.joints_number):
                        joints_configuration += f"{k + 1}" + [f"(°): {np.rad2deg(self.invkine_joints_configuration[k]):.1f}", f"(m): {self.invkine_joints_configuration[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])]  # the joints configuration indicator of the robotic manipulator
                        if k < self.joints_number - 1: joints_configuration += "   "
                        if (k + 1) % joints_configuration_columns_indicator == 0 and (k + 1) != self.joints_number: joints_configuration += "\n"
                else:
                    joints_configuration = "This end-effector configuration does not belong\nin the reachable workspace! No solution found!"
                self.joints_configuration_indicator.configure(text = joints_configuration)  # change the text of the joints configuration indicator
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_inverse_kinematics_indicators: {e}")
    # differential kinematics
    def change_chosen_joint_number_diffkine(self, event = None):  # change the chosen joint number for the differential kinematics analysis
        self.chosen_joint_number_diffkine = int(self.choose_joint_number_diffkine_combobox.get().split(" ")[-1])  # change the chosen joint number for the differential kinematics analysis
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def change_chosen_joint_number_diffkine_2(self, event = None):  # change the chosen joint number for the differential kinematics analysis
        self.chosen_joint_number_diffkine = self.joint_diffkine_velocity_combobox.current() + 1  # change the chosen joint number for the differential kinematics analysis
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def change_chosen_diffkine_velocity(self, event = None):  # change the value of the chosen joint velocity for the differential kinematics analysis
        try:  # try to get the entered value
            joint_diffkine_velocity = np.deg2rad(float(self.joint_diffkine_velocity_combobox.get()))  # the diffkine velocity of the chosen joint
            self.differential_kinematics_velocities[self.chosen_joint_number_diffkine - 1] = joint_diffkine_velocity  # change the diffkine velocity of the chosen joint
            self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
        except:  # if the entered value is not a number
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
    def change_diffkine_variable_slider(self, slider_num, event = None):  # change the value of the chosen joint velocity (changing the corresponding slider) for the differential kinematics analysis
        joint_diffkine_velocity = self.diffkine_variables_sliders[slider_num].get()  # the diffkine velocity of the chosen joint
        joint_type_index = self.joints_types_list.index(self.joints_types[slider_num])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the differential kinematics analysis
        self.differential_kinematics_velocities[slider_num] = [np.deg2rad(joint_diffkine_velocity), joint_diffkine_velocity][joint_type_index]  # change the diffkine velocity of the chosen joint
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def change_diffkine_wrt_frame(self, event = None):  # change the frame with respect to which the differential kinematics is computed
        self.diffkine_wrt_frame = self.alternate_matrix_elements(self.diffkine_wrt_frame_list, self.diffkine_wrt_frame)  # change the frame with respect to which the differential kinematics is computed
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def show_diffkine_info(self, event = None):  # show the differential kinematics information
        pass
    def update_differential_kinematics_indicators(self, event = None):  # update the differential kinematics indicators
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                self.joints_configuration_diffkine_indicator.configure(text = self.control_or_kinematics_variables_visualization)  # change the text of the differential kinematics robots joints indicator
                joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_diffkine - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the forward kinematics analysis
                joint_diffkine_velocity = [f"{np.rad2deg(self.differential_kinematics_velocities[self.chosen_joint_number_diffkine - 1]):.1f}", f"{self.differential_kinematics_velocities[self.chosen_joint_number_diffkine - 1]:.3f}"][joint_type_index]  # the diffkine velocity of the chosen joint
                self.choose_joint_number_diffkine_combobox.set(f"joint {self.chosen_joint_number_diffkine}")  # set the combobox to the chosen joint number for the differential kinematics analysis
                self.joint_diffkine_velocity_combobox.set(joint_diffkine_velocity)  # set the combobox to the chosen joint velocity for the differential kinematics analysis
                self.joint_diffkine_velocity_combobox["values"] = [f"{k + 1}: " + [f"{np.rad2deg(self.differential_kinematics_velocities[k]):.1f}", f"{self.differential_kinematics_velocities[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])] for k in range(self.joints_number)]  # update the values of the joint diffkine velocity combobox
                self.diffkine_value_unit_indicator.configure(text = ["(°/s)", "(m/s)"][joint_type_index])  # change the unit of the differential kinematics velocity indicator
                self.diffkine_wrt_frame_button.configure(text = self.diffkine_wrt_frame)  # change the text of the button that allows the user to change the frame with respect to which the differential kinematics is computed
                # solve the differential kinematics of the robotic manipulator for the chosen joints velocities
                end_effector_velocities = kin.compute_differential_kinematics(self.built_robotic_manipulator, self.built_robotic_manipulator.q, self.differential_kinematics_velocities, self.diffkine_wrt_frame)  # compute the differential kinematics of the robotic manipulator
                self.diffkine_linear_vel = np.array(end_effector_velocities[:3], dtype = float)  # the linear velocity of the end-effector
                self.diffkine_angular_vel = np.array(end_effector_velocities[3:], dtype = float)  # the angular velocity of the end-effector
                self.end_effector_linear_vel_indicator.configure(text = str([np.round(self.diffkine_linear_vel[k], self.distances_precision) for k in range(len(self.diffkine_linear_vel))]))  # change the text of the end-effector linear velocity indicator
                self.end_effector_angular_vel_indicator.configure(text = str([np.round(np.rad2deg(self.diffkine_angular_vel[k]), self.angles_precision) for k in range(len(self.diffkine_angular_vel))]))  # change the text of the end-effector angular velocity indicator
                for k in range(self.joints_number):
                    self.diffkine_variables_sliders[k].set([np.rad2deg(self.differential_kinematics_velocities[k]), self.differential_kinematics_velocities[k]][self.joints_types_list.index(self.joints_types[k])])  # set the value of the chosen joint velocity (changing the corresponding slider) for the differential kinematics analysis
        except: pass
        # except Exception as e: 
        #     if "invalid command name" not in str(e): print(f"Error in update_differential_kinematics_indicators: {e}")
    # inverse differential kinematics
    def get_diffkine_velocities_result(self, event = None):  # get the differential kinematics velocity result to the inverse differential kinematics analysis
        self.chosen_invdiffkine_linear_vel = np.copy(self.diffkine_linear_vel)  # change the end-effector linear velocity for the inverse differential kinematics analysis
        self.chosen_invdiffkine_angular_vel = np.copy(self.diffkine_angular_vel)  # change the end-effector angular velocity for the inverse differential kinematics analysis
        self.update_inverse_differential_kinematics_indicators()  # update the differential kinematics indicators
    def send_invdiffkine_joints_velocities_result(self, event = None):  # send the inverse differential kinematics joints velocities result to the differential kinematics analysis
        self.differential_kinematics_velocities = np.copy(self.invdiffkine_joints_velocities)  # change the joints velocities for the inverse differential kinematics analysis
        self.update_differential_kinematics_indicators()  # update the differential kinematics indicators
    def choose_end_effector_linear_velocity(self, event = None):  # choose the end-effector linear velocity for the differential kinematics analysis
        end_effector_vel_x = sd.askfloat("Choose the end-effector x linear velocity", "Enter the x linear velocity of the end-effector (in m/s):", initialvalue = self.chosen_invdiffkine_linear_vel[0], parent = self.menus_area)
        if end_effector_vel_x != None:  # if the user enters a number
            self.chosen_invdiffkine_linear_vel[0] = end_effector_vel_x  # change the x linear velocity of the end-effector
        end_effector_vel_y = sd.askfloat("Choose the end-effector y linear velocity", "Enter the y linear velocity of the end-effector (in m/s):", initialvalue = self.chosen_invdiffkine_linear_vel[1], parent = self.menus_area)
        if end_effector_vel_y != None:  # if the user enters a number
            self.chosen_invdiffkine_linear_vel[1] = end_effector_vel_y  # change the y linear velocity of the end-effector
        end_effector_vel_z = sd.askfloat("Choose the end-effector z linear velocity", "Enter the z linear velocity of the end-effector (in m/s):", initialvalue = self.chosen_invdiffkine_linear_vel[2], parent = self.menus_area)
        if end_effector_vel_z != None:  # if the user enters a number
            self.chosen_invdiffkine_linear_vel[2] = end_effector_vel_z  # change the z linear velocity of the end-effector
        self.update_inverse_differential_kinematics_indicators()  # update the differential kinematics indicators
    def choose_end_effector_angular_velocity(self, event = None):  # choose the end-effector angular velocity for the differential kinematics analysis
        end_effector_ang_vel_x = sd.askfloat("Choose the end-effector x angular velocity", "Enter the x angular velocity of the end-effector (in °/s):", initialvalue = np.rad2deg(self.chosen_invdiffkine_angular_vel[0]), parent = self.menus_area)
        if end_effector_ang_vel_x != None:  # if the user enters a number
            self.chosen_invdiffkine_angular_vel[0] = np.deg2rad(end_effector_ang_vel_x)  # change the x angular velocity of the end-effector
        end_effector_ang_vel_y = sd.askfloat("Choose the end-effector y angular velocity", "Enter the y angular velocity of the end-effector (in °/s):", initialvalue = np.rad2deg(self.chosen_invdiffkine_angular_vel[1]), parent = self.menus_area)
        if end_effector_ang_vel_y != None:  # if the user enters a number
            self.chosen_invdiffkine_angular_vel[1] = np.deg2rad(end_effector_ang_vel_y)  # change the y angular velocity of the end-effector
        end_effector_ang_vel_z = sd.askfloat("Choose the end-effector z angular velocity", "Enter the z angular velocity of the end-effector (in °/s):", initialvalue = np.rad2deg(self.chosen_invdiffkine_angular_vel[2]), parent = self.menus_area)
        if end_effector_ang_vel_z != None:  # if the user enters a number
            self.chosen_invdiffkine_angular_vel[2] = np.deg2rad(end_effector_ang_vel_z)  # change the z angular velocity of the end-effector
        self.update_inverse_differential_kinematics_indicators()  # update the differential kinematics indicators
    def change_invdiffkine_wrt_frame(self, event = None):  # change the frame with respect to which the inverse differential kinematics is computed
        self.invdiffkine_wrt_frame = self.alternate_matrix_elements(self.invdiffkine_wrt_frame_list, self.invdiffkine_wrt_frame)  # change the frame with respect to which the inverse differential kinematics is computed
        self.update_inverse_differential_kinematics_indicators()  # update the inverse differential kinematics indicators
    def show_invdiffkine_info(self, event = None):  # show the inverse differential kinematics information
        pass
    def update_inverse_differential_kinematics_indicators(self, event = None):  # update the inverse differential kinematics indicators
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                self.joints_configuration_invkine_indicator.configure(text = self.control_or_kinematics_variables_visualization)  # change the text of the inverse differential kinematics robots joints indicator
                self.end_effector_linear_velocity_button.configure(text = str([np.round(self.chosen_invdiffkine_linear_vel[k], self.distances_precision) for k in range(len(self.chosen_invdiffkine_linear_vel))]))  # change the text of the button that allows the user to choose the end-effector linear velocity
                self.end_effector_angular_velocity_button.configure(text = str([np.round(np.rad2deg(self.chosen_invdiffkine_angular_vel[k]), self.angles_precision) for k in range(len(self.chosen_invdiffkine_angular_vel))]))  # change the text of the button that allows the user to choose the end-effector angular velocity
                self.invdiffkine_wrt_frame_button.configure(text = self.invdiffkine_wrt_frame)  # change the text of the button that allows the user to change the frame with respect to which the inverse differential kinematics is computed
                # solve the inverse differential kinematics of the robotic manipulator for the chosen end-effector linear and angular velocities
                end_effector_velocity = np.concatenate((self.chosen_invdiffkine_linear_vel, self.chosen_invdiffkine_angular_vel), axis = 0)  # the end-effector velocity
                self.invdiffkine_joints_velocities, diffkine_success = kin.compute_inverse_differential_kinematics(self.built_robotic_manipulator, self.built_robotic_manipulator.q, end_effector_velocity, self.invdiffkine_wrt_frame)  # compute the inverse differential kinematics of the robotic manipulator
                joints_velocities = ""  # initialize the joints velocities indicator of the robotic manipulator
                joints_velocities_columns_indicator = 5  # the number of rows of the joints velocities indicator
                if diffkine_success:  # if the inverse differential kinematics analysis is successful
                    for k in range(self.joints_number):
                        joints_velocities += f"{k + 1}" + [f"(°/s): {np.rad2deg(self.invdiffkine_joints_velocities[k]):.1f}", f"(m/s): {self.invdiffkine_joints_velocities[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])]  # the joints velocities indicator of the robotic manipulator
                        if k < self.joints_number - 1: joints_velocities += "   "
                        if (k + 1) % joints_velocities_columns_indicator == 0 and (k + 1) != self.joints_number: joints_velocities += "\n"
                else:
                    joints_velocities = "The resulting Jacobian is singular!\nNo solution found!"
                self.joints_velocities_indicator.configure(text = joints_velocities)  # change the text of the joints configuration indicator
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_inverse_differential_kinematics_indicators: {e}")
    # functions used for the robotic manipulator manipulation
    def get_transformation_matrix(self, position, orientation, event = None):  # get the transformation matrix from the position and orientation (given in the form of roll-pitch-yaw angles) of a frame
        return sm.SE3(np.array(position, dtype = float)) * sm.SE3.RPY(np.array(orientation, dtype = float))  # return the transformation matrix
    def get_robot_joints_variables(self, joints_variables_chosen, event = None):  # get the proper values for the joints variables (control or fkine)
        if joints_variables_chosen == self.control_or_kinematics_variables_visualization_list[0]:  # if the control values are visualized
            q_config = np.array(self.control_joints_variables)  # get the control values for the joints variables
        elif joints_variables_chosen == self.control_or_kinematics_variables_visualization_list[1]:  # if the forward kinematics values are visualized
            q_config = np.array(self.forward_kinematics_variables)  # get the forward kinematics values for the joints variables
        return q_config  # return the proper values for the joints variables
    def get_all_frames_positions_orientations(self, q, event = None):  # get the positions and the orientations (as 3x3 rotation matrices) of the frames of the robotic manipulator
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            DH_parameters = np.vstack([self.built_robotic_manipulator_info["a_den_har_parameters"], self.built_robotic_manipulator_info["alpha_den_har_parameters"], self.built_robotic_manipulator_info["d_den_har_parameters"], self.built_robotic_manipulator_info["theta_den_har_parameters"]])  # the DH parameters of the robotic manipulator
            world_base_T = self.get_transformation_matrix(self.base_position_wrt_world, self.base_orientation_wrt_world)  # base wrt world transformation matrix
            base_0_T = self.get_transformation_matrix(self.zero_frame_position_wrt_base, self.zero_frame_orientation_wrt_base)  # zero frame wrt base transformation matrix
            robot_end_effector_T = self.get_transformation_matrix(self.end_effector_position_wrt_last_frame, self.end_effector_orientation_wrt_last_frame)  # end effector wrt n frame transformation matrix
            fkine_all_frames = kin.compute_forward_kinematics_all_frames(world_base_T, base_0_T, robot_end_effector_T, DH_parameters, self.built_robotic_manipulator_info["joints_types"], q)  # compute the forward kinematics for all the frames of the robotic manipulator
            frames_positions = [np.array(fkine_all_frames[frame])[:3, 3] for frame in range(len(fkine_all_frames))]  # get the positions of the frames of the robotic manipulator
            frames_orientations = [np.array(fkine_all_frames[frame])[:3, :3] for frame in range(len(fkine_all_frames))]  # get the orientations of the frames of the robotic manipulator
        else:
            frames_positions = [np.zeros(3, dtype = float) for frame in range(self.frames_number)]
            frames_orientations = [np.eye(3, dtype = float) for frame in range(self.frames_number)]
        return frames_positions, frames_orientations  # return the positions and the orientations of the frames of the robotic manipulator
    def get_fkine_frame_position_orientation(self, q, chosen_frame, event = None):  # calculate the position and the orientation of the chosen frame
        if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
            DH_parameters = np.vstack([self.built_robotic_manipulator_info["a_den_har_parameters"], self.built_robotic_manipulator_info["alpha_den_har_parameters"], self.built_robotic_manipulator_info["d_den_har_parameters"], self.built_robotic_manipulator_info["theta_den_har_parameters"]])  # the DH parameters of the robotic manipulator
            world_base_T = self.get_transformation_matrix(self.base_position_wrt_world, self.base_orientation_wrt_world)  # base wrt world transformation matrix
            base_0_T = self.get_transformation_matrix(self.zero_frame_position_wrt_base, self.zero_frame_orientation_wrt_base)  # zero frame wrt base transformation matrix
            robot_end_effector_T = self.get_transformation_matrix(self.end_effector_position_wrt_last_frame, self.end_effector_orientation_wrt_last_frame)  # end effector wrt n frame transformation matrix
            fkine_chosen_frame = kin.compute_forward_kinematics_until_frame(self.built_robotic_manipulator, world_base_T, base_0_T, robot_end_effector_T, DH_parameters, self.built_robotic_manipulator_info["joints_types"], q, chosen_frame)  # compute the forward kinematics for the chosen frame of the robotic manipulator
            pos = np.array(fkine_chosen_frame)[:3, 3]  # get the position of the chosen frame
            orient = np.array(fkine_chosen_frame)[:3, :3]  # get the orientation of the chosen frame
        else:  # if there is no robotic manipulator model built
            pos = np.zeros(3, dtype = float)  # the position of the chosen frame
            orient = np.eye(3, dtype = float)  # the orientation of the chosen frame
        return pos, orient  # return the position and the orientation of the chosen frame

    # for the main menu where the robotic manipulator is controlled
    def obtain_serial_ports(self, event = None):  # obtain the entire list of the available serial ports
        self.available_serial_ports = spf.serial_ports()  # search for the available serial ports of the computer
        if self.available_serial_ports:  # if there are available serial ports
            self.serial_port = self.available_serial_ports[0]  # the current serial port
        else:  # if there are no available serial ports
            ms.showerror("Error", "No available serial ports found!", parent = self.menus_area)  # show an error message
        self.update_serial_connection_indicators(self.serial_connection_state)  # update the serial connection indicators
    def change_serial_port(self, event = None):  # change the serial port used for the serial communication with the arduino microcontroller
        self.serial_port = self.serial_ports_combobox.get()  # change the serial port used for the serial communication with the arduino microcontroller
        self.update_serial_connection_indicators(self.serial_connection_state)  # update the serial connection indicators
    def change_baudrate(self, event = None):  # change the baudrate used for the serial communication with the arduino microcontroller
        self.baudrate = self.baudrates_combobox.get()  # change the baudrate used for the serial communication with the arduino microcontroller
        self.update_serial_connection_indicators(self.serial_connection_state)  # update the serial connection indicators
    def serial_connect_disconnect(self, event = None):  # connect to the chosen serial port to establish the serial communication with the arduino microcontroller or disconnect from the current serial port
        if self.serial_port != "":  # if the serial port is chosen
            if self.baudrate != "":  # if the baudrate is chosen
                if self.serial_connection.is_open:  # if the serial connection is open, then close it
                    current_serial_port = self.serial_connection.port  # the current serial port
                    self.close_serial_connection()  # close the serial connection and kill the serial communication thread
                    self.write_serial_monitor("Info:", "green", f"The serial connection to port \"{current_serial_port}\" has been closed successfully!")  # inform the user that the serial connection between the arduino microcontroller and the computer has been closed successfully
                    print("The serial connection has been closed successfully!")  # print a message to inform the user that the serial connection has been closed successfully
                else:  # if the serial connection is closed, then try to open it
                    try:  # try to establish the serial communication between the arduino microcontroller and the computer
                        self.serial_connection.port = self.serial_port  # change the serial port of the serial communication
                        self.serial_connection.baudrate = self.baudrate  # change the baudrate of the serial communication
                        self.serial_connection.timeout = 1  # set the timeout of the serial communication in seconds
                        self.serial_connection.close()  # close the serial connection/port
                        self.serial_connection.open()  # open the serial connection/port
                        self.start_serial_communication_thread()  # create and start a serial communication thread
                        self.write_serial_monitor("Info:", "green", f"The serial connection to port \"{self.serial_port}\" has been established successfully!")  # inform the user that the serial connection between the arduino microcontroller and the computer has been established successfully
                        self.update_serial_connection_indicators("Connected")  # update the serial connection indicator to the connected state
                        print("The serial connection has been established successfully!")  # print a message to inform the user that the serial connection has been established successfully
                    except Exception as e:  # if an error occurs
                        ms.showerror("Error", f"Error opening serial port \"{self.serial_port}\".", parent = self.menus_area)  # show an error message
                        print("Error opening serial port: " + str(e))  # print the error message
            else:  # if the baudrate is not chosen
                ms.showerror("Error", "Please choose the baudrate!", parent = self.menus_area)  # show an error message
        else:  # if the serial port is not chosen
            ms.showerror("Error", "Please choose the serial port!", parent = self.menus_area)  # show an error message
    def update_serial_connection_indicators(self, serial_connection_state):  # update the serial connection indicator
        try:
            self.serial_ports_combobox["values"] = self.available_serial_ports  # store the available serial ports as combobox values
            self.serial_ports_combobox.set(self.serial_port)  # set the value of the current serial port
            self.baudrates_combobox.set(self.baudrate)  # set the value of the current baudrate
            new_serial_connection_state = ""  # initialize the new serial connection state
            for index, string in enumerate(self.serial_connection_states_list):  # iterate through the serial connection states list
                if serial_connection_state in string and serial_connection_state != "" and serial_connection_state != " ":  # if the new serial connection state is in the current string
                    new_serial_connection_state = self.serial_connection_states_list[index]  # change the new serial connection state to the current string
            if new_serial_connection_state == "":  # if the new serial connection state is not an empty string
                new_serial_connection_state = ["Disconnected", "Connected to Port"][[True, False].index("disconnect" in serial_connection_state.lower())]  # change the new serial connection state to the disconnected or connected to port state
            self.serial_connection_state = new_serial_connection_state  # update the serial connection state
            self.serial_connection_indicator.configure(text = self.serial_connection_state, bg = self.serial_connection_indicator_colors[self.serial_connection_states_list.index(self.serial_connection_state)])  # update the serial connection indicator text and background color
            self.serial_connect_disconnect_command = ["Connect", "Disconnect"][[True, False].index("disconnect" in self.serial_connection_state.lower())]  # change the text of the serial connect/disconnect command to "Disconnect" or "Connect"
            self.serial_connect_button.configure(text = self.serial_connect_disconnect_command)  # change the text of the serial connect/disconnect button to the text of the serial connect/disconnect command
        except Exception as e:
            print(f"Error in update_serial_connection_indicators: {e}")
    def send_serial_command(self, command, event = None):  # send a command through the serial connection to be executed
        try:  # try to send the command through the serial connection
            if self.serial_connection.is_open:  # if the serial connection is open
                if self.allow_sending_all_ports or self.serial_connection_state != "Connected to Port":  # if the user is allowed to send commands to all serial ports or the user is connected to the wrong serial port
                    try:  # try to write the command to the serial monitor
                        self.write_serial_monitor(">>>", "blue", command)  # write the command to the serial monitor
                        self.command_entrybox.delete(0, "end")  # clear the console entry box
                    except: pass
                    print(">>> " + command)  # print the command
                    serial_command = command + "\n"  # add a newline character to the command to be sent
                    self.serial_connection.write(serial_command.encode("utf-8"))  # send the serial command through the opened serial port to the arduino microcontroller in order to be executed
                else:  # if the user is not allowed to send commands to all serial ports and the user is connected to the wrong serial port
                    if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[0]:  # if the robotic manipulator is controlled manually
                        try:  # try to write the command to the serial monitor
                            self.write_serial_monitor("Warn:", "brown", f"The command \"{command}\" could not be sent to the arduino microcontroller! You are connected to the wrong serial port or/and you are using the wrong baudrate!")  # inform the user that the command could not be sent to the arduino microcontroller
                        except: pass
                        print("You are connected to the wrong serial port or/and you are using the wrong baudrate!")  # print a message to inform the user that the command could not be sent to the arduino microcontroller
                        self.allow_sending_all_ports = ms.askyesno("Asking permission to send commands", f"Do you want to allow sending commands to the serial port \"{self.serial_port}\" with {self.baudrate} bps baudrate?")  # ask the user's permission to send commands to this serial port with this baudrate
                        if not self.allow_sending_all_ports and self.robot_control_thread_flag:  # if the user does not allow sending commands to this serial port with this baudrate and the robot control thread is running
                            self.kill_robot_control_thread()  # kill the robot control thread
            else:  # if the user tries to send a command when there is no serial connection established yet
                if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[0]:  # if the robotic manipulator is controlled manually
                    try:  # try to write the command to the serial monitor
                        self.write_serial_monitor("Warn:", "brown", f"The command \"{command}\" was not sent because there is no serial connection established yet!")  # inform the user that there is no serial connection established yet
                    except: pass
                    ms.showerror("Error", "There is no serial connection! You should first establish a serial connection before trying to send commands!", parent = self.menus_area)  # show an error message to the user that there is no serial connection
        except Exception as e:  # if an error occurs
            self.close_serial_connection()  # close the serial connection and kill the serial communication thread
            self.write_serial_monitor("Error:", "red", "The serial connection has been closed due to an error!")  # inform the user that the serial connection has been closed due to an error
            print("Error: " + str(e))  # print the error message
    def emit_serial_signals_to_console(self, emitted_message):  # emit the serial signals coming from the arduino microcontroller to the console
        try:  # try to emit the serial signals to the console
            if emitted_message == self.closed_serial_connection_message:  # if the emitted message is the closed serial connection message
                self.close_serial_connection()  # close the serial connection and kill the serial communication thread
                self.write_serial_monitor("Error:", "red", "The serial connection has been closed due to an error!")  # inform the user that the serial connection has been closed due to an error
            else:  # if the emitted message is not the closed serial connection message
                message_has_status = "MPos" in emitted_message  # check if the emitted message has the status of the robotic manipulator
                message_has_ok = "ok" in emitted_message  # check if the emitted message has the ok response
                if message_has_status:  # if the emitted message has the status of the robotic manipulator
                    status = emitted_message[1:].split(",")[0]  # get the status of the robotic manipulator
                    if not self.expanded_serial_monitor:  # if the serial monitor is expanded
                        self.update_serial_connection_indicators(status)  # update the serial connection indicator to the status of the robotic manipulator
                if not message_has_status and not message_has_ok:  # if the emitted message is not a status message or an ok response
                    self.write_serial_monitor("<<<", "purple", emitted_message)  # write the emitted message to the serial monitor
                elif message_has_ok and self.show_ok_responses:  # if the emitted message has the ok response and the user wants to see the ok responses
                    self.write_serial_monitor("<<<", "purple", emitted_message)  # write the emitted message to the serial monitor
                elif message_has_status and self.show_status_responses:  # if the emitted message has the status of the robotic manipulator and the user wants to see the status responses
                    self.write_serial_monitor("<<<", "purple", emitted_message)  # write the emitted message to the serial monitor
        except Exception as e:
            print("Error: " + str(e))  # print the error message
    def close_serial_connection(self, event = None):  # close the serial connection and kill the serial communication thread
        self.kill_serial_communication_thread()  # kill the existed and running serial communication thread
        self.serial_connection.close()  # close the serial connection/port
        self.update_serial_connection_indicators("Disconnected")  # update the serial connection indicator to the disconnected state
        self.allow_sending_all_ports = False  # allow the user to send commands to all serial ports
    def write_serial_monitor(self, message_tag, note_color, message):  # write a message to the console serial monitor
        self.text_pointer = self.console_serial_monitor.index('end')  # get the current text pointer of the console serial monitor
        if message_tag == ">>>":  # if the message is a command
            console_current_text = "\n" + message_tag + " " + message
        else:  # if the message is not a command (it explains something to the user, it is an info, a warning or an error message etc.)
            console_current_text = "\n" + message_tag + " " + message
        self.console_serial_monitor.insert("end", console_current_text)  # write the message to the console serial monitor
        self.console_serial_monitor.tag_add("{}".format(message_tag), str(float(self.text_pointer)), format(float(self.text_pointer) + float(len(message_tag) / 100), ".2f"))  # add a tag to the message
        self.console_serial_monitor.tag_configure("{}".format(message_tag), foreground = note_color, font = "Arial 10 bold italic")  # configure the tag
        self.serial_monitor_text = self.console_serial_monitor.get("1.0", "end")  # get the text of the console serial monitor
        self.console_serial_monitor.see("end")  # make the console serial monitor to scroll to the end
    def change_command_starting_text(self, event = None):  # change the starting text of the command to be sent to the arduino microcontroller
        self.command_starting_text = self.command_starting_text_entrybox.get()  # change the starting text of the command to be sent to the arduino microcontroller
        self.write_serial_monitor("Info:", "green", "The starting text of the command has been changed to \"{}\".".format(self.command_starting_text))  # inform the user that the starting text of the command has been changed
    def change_command_ending_text(self, event = None):  # change the ending text of the command to be sent to the arduino microcontroller
        self.command_ending_text = self.command_ending_text_entrybox.get()  # change the ending text of the command to be sent to the arduino microcontroller
        self.write_serial_monitor("Info:", "green", "The ending text of the command has been changed to \"{}\".".format(self.command_ending_text))  # inform the user that the ending text of the command has been changed
    def show_hide_ok_responses_on_console(self, event = None):  # show or hide the ok responses on the console serial monitor
        self.show_ok_responses = not self.show_ok_responses  # show or hide the ok responses on the console serial monitor
        self.show_hide_ok_button.configure(text = self.show_responses_indicators[[False, True].index(self.show_ok_responses)])  # change the text of the show/hide ok responses button
        self.write_serial_monitor("Info:", "green", "The ok responses are now {}.".format(["hidden", "shown"][[False, True].index(self.show_ok_responses)]))  # inform the user that the ok responses are now shown or hidden
    def show_hide_status_responses_on_console(self, event = None):  # show or hide the status responses on the console serial monitor
        self.show_status_responses = not self.show_status_responses  # show or hide the status responses on the console serial monitor
        self.write_serial_monitor("Info:", "green", "The status responses are now {}.".format(["hidden", "shown"][[False, True].index(self.show_status_responses)]))  # inform the user that the status responses are now shown or hidden
        self.show_hide_status_button.configure(text = ["✖", "✔"][[False, True].index(self.show_status_responses)])  # change the text of the show/hide status responses button
    def expand_serial_monitor_menu(self, event = None):  # expand the serial monitor menu
        self.expanded_serial_monitor = not self.expanded_serial_monitor  # expand or collapse the serial monitor menu
        self.build_control_robotic_manipulator_menus(self.submenus_titles[self.main_menu_choice], self.submenus_descriptions[self.main_menu_choice])  # create the control robotic manipulator menus
    def clear_serial_monitor(self, event = None):  # clear the console serial monitor
        self.console_serial_monitor.delete("1.0", "end")  # clear the console serial monitor
        self.serial_monitor_text = ""  # clear the serial monitor text
    def change_chosen_joint_number_control(self, event = None):  # change the chosen control joint number of the robotic manipulator
        self.chosen_joint_number_control = int(self.choose_joint_number_control_combobox.get().split(" ")[-1])  # change the chosen control joint number of the robotic manipulator
        self.chosen_joint_motor_number = 0  # the chosen motor number of the chosen control joint
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_chosen_joint_number_control_2(self, event = None):  # change the chosen control joint number of the robotic manipulator
        self.chosen_joint_number_control = self.joint_control_variable_combobox.current() + 1  # change the chosen control joint number of the robotic manipulator
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def copy_fkine_to_control_values(self, event = None):  # copy the forward kinematics variables to the control variables
        self.control_joints_variables = copy.deepcopy(self.forward_kinematics_variables)  # copy the forward kinematics variables to the control variables 
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_chosen_joint_motors()  # send the right command to the chosen control joint motors
    def set_joints_variables_to_zero(self, event = None):  # set the control joints variables of the robotic manipulator to zero
        for joint_number in range(1, self.joints_number + 1):  # iterate through the joints of the robotic manipulator
            self.control_joints_variables[joint_number - 1] = 0  # set the control joint variable of the chosen control joint to zero
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_all_motors()  # send the right command to all motors
    def change_control_variable_slider(self, event = None):  # change the control variable of the chosen joint of the robotic manipulator
        joint_control_variable = float(self.joint_variable_slider.get())  # the control variable of the chosen control joint
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        self.control_joints_variables[self.chosen_joint_number_control - 1] = [np.deg2rad(joint_control_variable), joint_control_variable][joint_type_index]  # change the control variable of the chosen control joint
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_chosen_joint_motors()  # send the right command to the chosen control joint motors
    def change_chosen_control_variable(self, event = None):  # change the control variable of the chosen joint of the robotic manipulator
        try:  # try to get the entered value
            joint_control_variable = float(self.joint_control_variable_combobox.get())  # the control variable of the chosen control joint
        except:  # if the entered value is not a number
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
            return  # return to the main loop
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        self.control_joints_variables[self.chosen_joint_number_control - 1] = [np.deg2rad(joint_control_variable), joint_control_variable][joint_type_index]  # change the control variable of the chosen control joint
        control_variable_limits = [np.deg2rad(self.control_joints_variables_limits[self.chosen_joint_number_control - 1][0]).tolist(), self.control_joints_variables_limits[self.chosen_joint_number_control - 1][1]][joint_type_index]  # the control variable limits of the chosen control joint
        if joint_control_variable < self.control_joints_variables_limits[self.chosen_joint_number_control - 1][joint_type_index][0]:  # if the entered value is less than the joint variable minimum limit
            self.control_joints_variables[self.chosen_joint_number_control - 1] = control_variable_limits[0]  # change the control variable of the chosen control joint to the minimum limit
        elif joint_control_variable > self.control_joints_variables_limits[self.chosen_joint_number_control - 1][joint_type_index][1]:  # if the entered value is greater than the joint variable maximum limit
            self.control_joints_variables[self.chosen_joint_number_control - 1] = control_variable_limits[1]  # change the control variable of the chosen control joint to the maximum limit
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_chosen_joint_motors()  # send the right command to the chosen control joint motors
    def increase_decrease_control_variable(self, change_type, event = None):  # increase or decrease the control variable of the chosen joint of the robotic manipulator
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        self.control_joints_variables[self.chosen_joint_number_control - 1] += np.sign(change_type) * [(10**(-self.angles_precision) * np.deg2rad([1, 10, 100])).tolist(), (10**(-self.distances_precision) * np.array([1, 10, 100])).tolist()][joint_type_index][np.abs(change_type) - 1]  # increase or decrease the control variable of the chosen control joint by the correct ammount
        control_variable_limits = [np.deg2rad(self.control_joints_variables_limits[self.chosen_joint_number_control - 1][0]).tolist(), self.control_joints_variables_limits[self.chosen_joint_number_control - 1][1]][joint_type_index]  # the control variable limits of the chosen control joint
        if self.control_joints_variables[self.chosen_joint_number_control - 1] < control_variable_limits[0]:  # if the entered value is less than the joint variable minimum limit
            self.control_joints_variables[self.chosen_joint_number_control - 1] = control_variable_limits[0]  # change the control variable of the chosen control joint to the minimum limit
        elif self.control_joints_variables[self.chosen_joint_number_control - 1] > control_variable_limits[1]:  # if the entered value is greater than the joint variable maximum limit
            self.control_joints_variables[self.chosen_joint_number_control - 1] = control_variable_limits[1]  # change the control variable of the chosen control joint to the maximum limit
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_chosen_joint_motors()  # send the right command to the chosen control joint motors
    def change_chosen_joint_motors(self, event = None):  # change the chosen control joint motors
        self.chosen_joint_motor_number = self.joints_motors_list[self.chosen_joint_number_control - 1].index(self.choose_joint_motors_combobox.get())  # the current chosen control joint motor
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def increase_chosen_joint_motors(self, event = None):  # increase the chosen control joint motors
        ask_added_joint_motor = sd.askstring(f"Increase the joint {self.chosen_joint_number_control} motors", "Enter the motor's name to be added:", initialvalue = "", parent = self.menus_area)  # ask the user to enter the new motor's name
        if ask_added_joint_motor != None:  # if the user enters a name
            if ask_added_joint_motor not in self.joints_motors_list[self.chosen_joint_number_control - 1]:  # if the new motor's name is not already in the list of the chosen control joint motors
                self.joints_motors_list[self.chosen_joint_number_control - 1].append(ask_added_joint_motor)  # add the new motor's name to the list of the chosen control joint motors
                self.joints_motors_mult_factors[self.chosen_joint_number_control - 1].append(1)  # add the new motor's multiplication factor to the list of the chosen control joint motors
                self.chosen_joint_motor_number = 0  # initialize the chosen motor number of the chosen control joint
            else:
                ms.showinfo("Info", f"The motor's name \"{ask_added_joint_motor}\" is already in the list of the chosen control joint motors.", parent = self.menus_area)
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def decrease_chosen_joint_motors(self, event = None):  # decrease the chosen control joint motors
        if len(self.joints_motors_list[self.chosen_joint_number_control - 1]) > 1:  # if there are more than one motors in the list of the chosen control joint motors
            ask_deleted_joint_motor = sd.askstring(f"Decrease the joint {self.chosen_joint_number_control} motors", "Enter the motor's name to be deleted:", initialvalue = "", parent = self.menus_area)  # ask the user to enter the motor's name to be deleted
            if ask_deleted_joint_motor != None:  # if the user enters a name
                if ask_deleted_joint_motor in self.joints_motors_list[self.chosen_joint_number_control - 1]:  # if the motor's name is in the list of the chosen control joint motors and there are more than one motors
                    self.joints_motors_mult_factors[self.chosen_joint_number_control - 1].pop(self.joints_motors_list[self.chosen_joint_number_control - 1].index(ask_deleted_joint_motor))  # remove the motor's multiplication factor from the list of the chosen control joint motors
                    self.joints_motors_list[self.chosen_joint_number_control - 1].pop(self.joints_motors_list[self.chosen_joint_number_control - 1].index(ask_deleted_joint_motor))  # remove the motor's name from the list of the chosen control joint motors
                    self.chosen_joint_motor_number = 0  # initialize the chosen motor number of the chosen control joint
                else:
                    ms.showinfo("Info", f"The motor's name \"{ask_deleted_joint_motor}' is not in the list of the chosen control joint motors.", parent = self.menus_area)
        else:
            ms.showinfo("Info", "There is only one motor in the list of the chosen control joint motors, so it can not be deleted. I you want to change its name, you should first add a new motor with the desired name and then delete the old one.", parent = self.menus_area)
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_motors_mult_factors_entrybox(self, event = None):  # change the multiplication factors of the chosen control joint motors
        try:  # try to change the multiplication factors of the chosen control joint motors
            self.joints_motors_mult_factors[self.chosen_joint_number_control - 1][self.chosen_joint_motor_number] = float(self.motors_mult_factors_entrybox.get())  # change the multiplication factors of the chosen control joint motors
        except:  # if an error occurs
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_control_end_effector_slider(self, event = None):  # change the control end-effector state of the robotic manipulator
        self.control_end_effector_variable = float(self.end_effector_slider.get())  # the control end-effector variable of the robotic manipulator
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_end_effector()  # send the right command to the end-effector motors
    def set_end_effector_to_zero(self, event = None):  # set the control end-effector state of the robotic manipulator to zero
        self.control_end_effector_variable = 0  # set the control end-effector variable to zero
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
        if self.robotic_manipulator_control_mode == self.robotic_manipulator_control_modes_list[1]:  # if the robotic manipulator control mode is automatic
            self.send_command_to_end_effector()  # send the right command to all motors
    def change_end_effector_motor(self, event = None):  # change the chosen control end-effector motor
        self.end_effector_motor = self.end_effector_motor_entrybox.get()  # change the chosen control end-effector motor
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_end_effector_mult_factor(self, event = None):  # change the multiplication factor of the chosen control end-effector motor
        try:  # try to change the multiplication factor of the control end-effector motor
            self.end_effector_motor_mult_factor = float(self.end_effector_mult_factor_entrybox.get())  # change the multiplication factor of the chosen control end-effector motor
        except:  # if an error occurs
            ms.showerror("Error", "Please enter a number!", parent = self.menus_area)  # show an error message
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def change_robotic_manipulator_control_mode(self, event = None):  # change the control mode of the robotic manipulator
        self.robotic_manipulator_control_mode = self.alternate_matrix_elements(self.robotic_manipulator_control_modes_list, self.robotic_manipulator_control_mode)  # change the control mode of the robotic manipulator
        self.update_control_variables_indicators()  # update the indicators of the control variables of the robotic manipulator
    def send_command_to_chosen_joint_motors(self, event = None):  # send the control joints values (multiplied by some factors) to the chosen control joint motors
        command = ""  # the command to be sent to the console
        command_dict = {}  # initialize a dictionary for the command motors
        joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
        for joint_number in range(1, self.joints_number + 1):  # iterate through the joints of the robotic manipulator
            joint_value = self.control_joints_variables[joint_number - 1]  # the value of the joint
            for k in range(len(self.joints_motors_list[joint_number - 1])):  # iterate through all the motors of the chosen control joint
                motor_name = self.joints_motors_list[joint_number - 1][k]  # the name of the motor
                motor_value = np.round([np.rad2deg(joint_value), joint_value][joint_type_index] * self.joints_motors_mult_factors[joint_number - 1][k], 2)  # the value of the motor
                if motor_name in self.joints_motors_list[self.chosen_joint_number_control - 1]:  # if the motor name is in the list of the chosen control joint motors
                    command_dict[motor_name] = command_dict.get(motor_name, 0) + motor_value  # combine the values of the same motors
        command = " ".join([motor + str(np.round(value, 2)) for motor, value in command_dict.items()])  # rewrite the command string
        command = self.command_starting_text + command + self.command_ending_text  # add the starting and ending texts to the command
        self.send_serial_command(command)  # send the command to the console to be executed
    def send_command_to_all_motors(self, event = None):  # send the control joints values (multiplied by some factors) to all motors
        command_dict = {}  # initialize a dictionary for the command motors
        for joint_number in range(1, self.joints_number + 1):  # iterate through the joints of the robotic manipulator
            joint_value = self.control_joints_variables[joint_number - 1]  # the value of the joint
            joint_type_index = self.joints_types_list.index(self.joints_types[joint_number - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
            for k in range(len(self.joints_motors_list[joint_number - 1])):  # iterate through all the motors of the chosen control joint
                motor_name = self.joints_motors_list[joint_number - 1][k]  # the name of the motor
                motor_value = np.round([np.rad2deg(joint_value), joint_value][joint_type_index] * self.joints_motors_mult_factors[joint_number - 1][k], 2)  # the value of the motor
                command_dict[motor_name] = command_dict.get(motor_name, 0) + motor_value  # combine the values of the same motors
        command = " ".join([motor + str(np.round(value, 2)) for motor, value in command_dict.items()])  # rewrite the command string for the joints motors
        # command += " " + self.end_effector_motor + str(np.round(self.control_end_effector_variable * self.end_effector_motor_mult_factor, 2))  # add the end-effector motor command to the joints motors command
        command = self.command_starting_text + command + self.command_ending_text  # add the starting and ending texts to the command
        self.send_serial_command(command)  # send the command to the console to be executed
    def send_command_to_end_effector(self, event = None):  # send the control end-effector values (multiplied by some factors) to the end-effector motors
        command = self.end_effector_motor + str(np.round(self.control_end_effector_variable * self.end_effector_motor_mult_factor, 2))  # the command to be sent to the console
        command = self.command_starting_text + command + self.command_ending_text  # add the starting and ending texts to the command
        self.send_serial_command(command)  # send the command to the console to be executed
    def update_control_variables_indicators(self, event = None):  # update the indicators of the control variables of the robotic manipulator
        try:
            if self.robotic_manipulator_is_built:  # if a robotic manipulator model is built
                self.choose_joint_number_control_combobox.set(f"joint {self.chosen_joint_number_control}")  # set the value of the chosen joint number for the control
                joint_type_index = self.joints_types_list.index(self.joints_types[self.chosen_joint_number_control - 1])  # the index of the chosen joint type (0 for revolute and 1 for prismatic) for the control
                joint_control_variable = [f"{np.rad2deg(self.control_joints_variables[self.chosen_joint_number_control - 1]):.1f}", f"{self.control_joints_variables[self.chosen_joint_number_control - 1]:.3f}"][joint_type_index]  # the control variable of the chosen control joint
                self.joint_type_indicator.configure(text = self.joints_types[self.chosen_joint_number_control - 1] + [" (degrees)", " (meters)"][joint_type_index])  # change the joint type indicator label to the chosen joint type
                self.joint_variable_slider.configure(resolution = [10**(-self.angles_precision), 10**(-self.distances_precision)][joint_type_index], \
                                                    from_ = self.control_joints_variables_limits[self.chosen_joint_number_control - 1][joint_type_index][0], to = self.control_joints_variables_limits[self.chosen_joint_number_control - 1][joint_type_index][1])  # set the slider limits to the current control joint variable limits
                self.joint_variable_slider.set(joint_control_variable)  # set the slider to the control variable of the chosen control joint
                self.joint_control_variable_combobox.set(joint_control_variable)  # set the combobox to the chosen joint variable for the control
                self.joint_control_variable_combobox["values"] = [f"{k + 1}: " + [f"{np.rad2deg(self.control_joints_variables[k]):.1f}", f"{self.control_joints_variables[k]:.3f}"][self.joints_types_list.index(self.joints_types[k])] for k in range(self.joints_number)]  # update the values of the joint control variable combobox
                self.increase_joint_var_1_button.configure(text = ["+0.1", "+0.001"][joint_type_index])
                self.decrease_joint_var_1_button.configure(text = ["-0.1", "-0.001"][joint_type_index])
                self.increase_joint_var_2_button.configure(text = ["+1", "+0.01"][joint_type_index])
                self.decrease_joint_var_2_button.configure(text = ["-1", "-0.01"][joint_type_index])
                self.increase_joint_var_3_button.configure(text = ["+10", "+0.1"][joint_type_index])
                self.decrease_joint_var_3_button.configure(text = ["-10", "-0.1"][joint_type_index])
                self.choose_joint_motors_combobox["values"] = self.joints_motors_list[self.chosen_joint_number_control - 1]  # store the available control joint motors as combobox values
                self.choose_joint_motors_combobox.set(self.joints_motors_list[self.chosen_joint_number_control - 1][self.chosen_joint_motor_number])  # set the combobox to the current control joint motor
                self.motors_mult_factors_entrybox.delete(0, "end")  # clear the motors multiplication factors entry box
                self.motors_mult_factors_entrybox.insert(0, f"{self.joints_motors_mult_factors[self.chosen_joint_number_control - 1][self.chosen_joint_motor_number]:.3f}")  # insert the motors multiplication factors of the chosen control joint to the entry box
                self.end_effector_slider.set(self.control_end_effector_variable)  # set the slider to the control variable of the chosen control joint
                self.end_effector_motor_entrybox.delete(0, "end")  # clear the end-effector motor entry box
                self.end_effector_motor_entrybox.insert(0, self.end_effector_motor)  # set the combobox to the current control joint motor
                self.end_effector_mult_factor_entrybox.delete(0, "end")  # clear the end-effector motor multiplication factor entry box
                self.end_effector_mult_factor_entrybox.insert(0, f"{self.end_effector_motor_mult_factor:.3f}")  # insert the end-effector motor multiplication factor to the entry box
                self.choose_control_mode_button.configure(text = self.robotic_manipulator_control_mode)  # change the text of the appropriate button to the chosen control mode
        except: pass
        # except Exception as e:
        #     if "invalid command name" not in str(e): print(f"Error in update_control_variables_indicators: {e}")

    # the functions used for the serial communication thread between the arduino microcontroller and the computer
    def start_serial_communication_thread(self, event = None):  # create and start a serial communication thread
        self.serial_connection_thread_flag = True  # let the serial communication thread run
        self.serial_communication_thread = threading.Thread(target = self.serial_communication_thread_run_function)  # create the thread for the serial communication with the arduino microcontroller
        self.serial_communication_thread.start()  # start the serial communication thread
    def kill_serial_communication_thread(self, event = None):  # kill the existed and running serial communication thread
        self.serial_connection_thread_flag = False  # stop the serial communication thread
    def serial_communication_thread_run_function(self):  # the run function, it continuously checks for the data sent from the serial connection and emits signals for the messages and errors
        self.serial_connection_elapsed_time = time.time()  # the elapsed time since the serial communication thread started
        while True:  # while the thread is running
            if self.serial_connection.is_open:  # if the serial connection is open
                try:
                    bytes_waiting = self.serial_connection.in_waiting  # the number of bytes waiting to be read from the serial connection
                    if time.time() - self.serial_connection_elapsed_time > 1.0:  # if the elapsed time since the thread started is greater than 1.0 seconds
                        self.serial_connection_elapsed_time = time.time()  # update the elapsed time since the thread started
                        self.serial_connection.write("?\n".encode("utf-8"))  # send the command to show the status and position information
                    data_read = str(self.serial_connection.readline().decode("utf-8")).strip()  # read the data from the serial connection
                    if data_read != "":  # if there is data read from the serial connection
                        self.emit_serial_signals_to_console(data_read)  # emit a message containing the data read from the serial connection
                    print("<<< " + data_read)  # print the data read from the serial connection
                except Exception as e:  # if an error occurs
                    self.emit_serial_signals_to_console(self.closed_serial_connection_message)  # emit a message that the serial connection is closed
                    print(self.closed_serial_connection_message + " Error: " + str(e))  # print that the serial connection is closed along with the error message
            if not self.serial_connection_thread_flag:  # if the thread is stopped
                break  # break the while loop


# the main function of the program
if __name__ == "__main__":
    # windows_number = int(input("How many windows (program instances) do you want to create? "))
    windows_number = 1
    roots_list = []
    guis_list = []
    for window in range(windows_number):
        roots_list.append(tk.Tk())
        guis_list.append(robotic_manipulators_playground_window(roots_list[window], window))
    for window in range(windows_number):
        roots_list[window].mainloop()
