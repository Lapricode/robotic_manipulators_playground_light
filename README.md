# A playground for serial robotic manipulators

## Introduction

This is a python tkinter GUI for designing, simulating and controlling serial robotic manipulators (with open kinematic chains). It's a light edition of a bigger project located in the repository https://github.com/Lapricode/robotic_manipulators_playground. The inspiration for this work came from the open source project "Thor (3D Printable 6DOF Robotic Arm)", that you may explore going to the link http://thor.angel-lm.com. As such, the GUI was first built to support mainly the Thor robotic arm, but it evolved to a more generic program for the simulation of any serial robotic manipulator up to 12 DOFs. It is supported on both Windows and Linux systems.
Thor robotic arm is an open-source project, with the entire construction process and control code freely available (http://thor.angel-lm.com). Its supporting structure (or body) consists of 3D-printable parts. It is a serial robotic manipulator with 6 degrees of freedom (6 DOF). All of its joints are rotational, arranged in a yaw-pitch-pitch-yaw-pitch-yaw configuration (or yaw-roll-roll-yaw-roll-yaw, depending on the perspective of the x and y axes), starting from the base and extending to the end-effector. In the picture below you can see the fully constructed Thor robotic arm, its joints frames used for the kinematics analysis and the corresponding classic DH parameters.

![image](https://github.com/user-attachments/assets/f9536b42-0e1a-4463-8bb5-6335701df5cf)

## Running instructions

To start the program, run the python file "robotic_manipulators_playground_light.py". The GUI can be run in both Windows and Linux. The Python libraries needed are written below, along with their suggested versions that :

- To be installed (using pip for example):
    - roboticstoolbox
    - swift
    - matplotlib (matplotlib.pyplot, matplotlib.colors)
    - mpl_toolkits (mpl_toolkits.mplot3d)
    - numpy
    - scipy (scipy.ndimage)
    - spatialmath
    - pyserial
- Native:
    - tkinter (tkinter.ttk, tkinter.simpledialog, tkinter.messagebox, tkinter.colorchooser)
    - string
    - copy
    - os
    - shutil
    - time
    - threading

There is a problem with the swift library that may has been solved by now, or not :). The issue has been identified in the robotics-toolbox-python repository on GitHub: https://github.com/petercorke/robotics-toolbox-python/issues/383. The problem arises when running the swift library on Windows. Specifically, there is a discrepancy in how file paths are handled between Linux and Windows systems. The root of the issue lies in how the self.path variable is processed within the SwiftRoute.py file. The current implementation attempts to adjust the path by retaining the initial / character, which works fine on Linux but leads to incorrect path formatting on Windows. To address the problem on Windows, a simple adjustment can be made in the SwiftRoute.py file of the swift library. Specifically, update the block of code by modifying self.path[9:] to self.path[10:].

So, for linux it must be like this:
```python
elif self.path.startswith("/retrieve/"):
    # print(f"Retrieving file: {self.path[10:]}")
    self.path = urllib.parse.unquote(self.path[9:])
    self.send_file_via_real_path()
    return
```

And for windows it must be like this:
```python
elif self.path.startswith("/retrieve/"):
    # print(f"Retrieving file: {self.path[10:]}")
    self.path = urllib.parse.unquote(self.path[10:])
    self.send_file_via_real_path()
    return
```

## GUI features

### Visualizing the robotic manipulators and their environment - canvas area

When the program starts, the visualization canvas and the first basic menu appear on the main screen. The visualization canvas initially displays the world coordinate system, along with the floor. This is where the basic design of the skeletal model of each robotic arm is carried out. Simple geometric objects (dots, lines, and planes) are used for this representation, with the significant drawback that the depth of objects is not accurately depicted, as no z-buffer is considered, unlike typical graphics. For more realistic simulations, there is also the online Swift simulator.

![image](https://github.com/user-attachments/assets/c0b0a2a0-c37f-4db5-876b-8c223946f196)

### Designing and simulating the robotic manipulators - 1st menu

In the first menu, all the basic tools for designing and modeling a serial robotic manipulator with an open kinematic chain (first submenu) are available, as well as for visualizing the robot and its environment (second submenu) on the canvas or in the online Swift simulator. Users have flexibility in defining the joints (number, type, range of motion), the structure of the robotic manipulator (DH parameters), and in determining the configuration of the base and the end-effector. There is also an important feature that allows for saving and loading robotic models, enabling them to be reused at any time.

![image](https://github.com/user-attachments/assets/9b0d6aa5-9686-4df7-af13-19c497f77515)

### Analysing the kinematics of the robotic manipulators - 2nd and 3rd menus

The second and third menus are used for the forward and inverse kinematics analysis, as well as for the differential and inverse differential kinematics analysis, respectively, of the manipulator. The program uses the classic Denavit - Hartenberg parameterization. In forward kinematics, users can control the variables of all the joints, and then the configurations (position and orientation) of all the frames of the robot are calculated. Also, the accessible workspace can be visualized. In inverse kinematics, the user specifies the configuration of the end-effector, and then the variables of all the joints are calculated. In differential kinematics, the velocities of all the joints are controlled, and then the corresponding velocity (linear and angular) of the end-effector is determined, either in its own frame or in the world frame. In inverse differential kinematics, the user provides the velocity of the end-effector (again in one of the two mentioned frames), and then the joints velocities are returned.

![image](https://github.com/user-attachments/assets/f87f2005-1aa6-4041-8070-1a22ca7fc524)

### Controlling the robotic manipulators - 4th menu

In the picture below, there is the original Asgard GUI from the "Thor" open source project (the left side, it can be found here: http://thor.angel-lm.com/documentation/control-software) and my suggestion (the right side). The menu is heavily inspired from the Asgard structure.

![image](https://github.com/user-attachments/assets/4da05154-e8e3-43e2-ae6d-dde2ab3aeadb)

