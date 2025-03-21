# A Playground for Serial Robotic Manipulators

## Introduction

This is a python tkinter GUI for designing, simulating and controlling serial robotic manipulators (with open kinematic chains). It's a light edition of a bigger project located in the repository https://github.com/Lapricode/robotic_manipulators_playground. The inspiration for this work came from the open source project "Thor (3D Printable 6DOF Robotic Arm)", that you may explore going to the link http://thor.angel-lm.com. As such, the GUI was first built to support mainly the Thor robotic arm, but it evolved to a more generic program for the simulation of any serial robotic manipulator up to 12 DOFs. It is supported on both Windows and Linux systems.

Thor is an open-source project, with the entire construction process and control code freely available (http://thor.angel-lm.com). Its supporting structure (or body) consists of 3D-printable parts. It is a serial robotic manipulator with 6 degrees of freedom (6 DOF). All of its joints are rotational, arranged in a yaw-pitch-pitch-yaw-pitch-yaw configuration (or yaw-roll-roll-yaw-roll-yaw, depending on the perspective of the x and y axes), starting from the base and extending to the end-effector. In the picture below you can see the fully constructed Thor robotic arm, its joints frames used for the kinematics analysis and the corresponding classic DH parameters.

![image](https://github.com/user-attachments/assets/f9536b42-0e1a-4463-8bb5-6335701df5cf)

## Running instructions

To launch the program, run the Python script "robotic_manipulators_playground_light.py." The graphical user interface (GUI) is compatible with both Windows and Linux systems. Below is a list of the required Python libraries, along with the versions that have been tested and confirmed to work on my setup (although other newer versions may also be fine):

- To be installed (using pip for example):
    - roboticstoolbox (roboticstoolbox-python), 1.1.0
    - swift (swift-sim), 1.1.0
    - matplotlib, 3.7.0
    - numpy, 1.26.4
    - scipy, 1.11.3
    - spatialmath (spatialmath-python), 1.1.9
    - pyserial, 3.5
- Native: tkinter, string, copy, os, shutil, time, threading

There is a problem with the swift library that may has been solved by now, or not :). The issue has been identified in the robotics-toolbox-python repository on GitHub: https://github.com/petercorke/robotics-toolbox-python/issues/383. The problem arises when running the swift library on Windows. Specifically, there is a discrepancy in how file paths are handled between Linux and Windows systems. The root of the issue lies in how the self.path variable is processed within the SwiftRoute.py file. The current implementation attempts to adjust the path by retaining the initial / character, which works fine on Linux but leads to incorrect path formatting on Windows. To address the problem on Windows, a simple adjustment can be made in the SwiftRoute.py file of the swift library. Specifically, update the block of code by modifying self.path[9:] to self.path[10:].

So, for linux it must be like this:
```python
elif self.path.startswith("/retrieve/"):
    self.path = urllib.parse.unquote(self.path[9:])
    self.send_file_via_real_path()
    return
```

And for windows it must be like this:
```python
elif self.path.startswith("/retrieve/"):
    self.path = urllib.parse.unquote(self.path[10:])
    self.send_file_via_real_path()
    return
```

## GUI Description

### Visualizing the robotic manipulators and their environment - canvas area

When the program starts, the visualization canvas and the first basic menu appear on the main screen. The visualization canvas initially displays the world coordinate system, along with the floor. This is where the basic design of the skeletal model of each robotic arm is carried out. Simple geometric objects (dots, lines, and planes) are used for this representation, with the significant drawback that the depth of objects is not accurately depicted, as no z-buffer is considered, unlike typical graphics. For more realistic simulations, there is also the online Swift simulator. In the left side bar there are some controls for the world frame movement and the visualization of the scene objects.

![image](https://github.com/user-attachments/assets/c0b0a2a0-c37f-4db5-876b-8c223946f196)

### Designing and simulating the robotic manipulators - 1st menu

In the first menu, all the basic tools for designing and modeling a serial robotic manipulator with an open kinematic chain (first submenu) are available, as well as for visualizing the robot and its environment (second submenu) on the canvas or in the online Swift simulator. Users have flexibility in defining the joints (number, type, range of motion), the structure of the robotic manipulator (DH parameters), and in determining the configuration of the base and the end-effector. There is also an important feature that allows for saving and loading robotic models, enabling them to be reused at any time.

![image](https://github.com/user-attachments/assets/9b0d6aa5-9686-4df7-af13-19c497f77515)

### Analysing the kinematics of the robotic manipulators - 2nd and 3rd menus

The second and third menus are used for the forward and inverse kinematics analysis, as well as for the differential and inverse differential kinematics analysis, respectively, of the manipulator. The program uses the classic Denavit - Hartenberg parameterization. In forward kinematics, users can control the variables of all the joints, and then the configurations (position and orientation) of all the frames of the robot are calculated. Also, the accessible workspace can be visualized. In inverse kinematics, the user specifies the configuration of the end-effector, and then the variables of all the joints are calculated. In differential kinematics, the velocities of all the joints are controlled, and then the corresponding velocity (linear and angular) of the end-effector is determined, either in its own frame or in the world frame. In inverse differential kinematics, the user provides the velocity of the end-effector (again in one of the two mentioned frames), and then the joints velocities are returned.

![image](https://github.com/user-attachments/assets/f87f2005-1aa6-4041-8070-1a22ca7fc524)

### Controlling the robotic manipulators - 4th menu

In the picture below, there is the original Asgard GUI from the "Thor" open source project (the right side, it can be found here: http://thor.angel-lm.com/documentation/control-software) and mine suggested GUI (the left side). It is the fourth menu of the program, with a structure heavily inspired from the Asgard software. It includes three submenus: the first sets up a serial communication with the Arduino, allowing users to choose available ports and transmission rates, and monitor connection status. The second is a serial monitor displaying commands sent to and from the Arduino, with options for automated messages, shortcuts, and clearing or expanding the monitor window. The third submenu controls the robot manipulator’s joints and end-effector, enabling flexible command inputs and adjustments, with the ability to send commands individually or in batches.

![image](https://github.com/user-attachments/assets/4da05154-e8e3-43e2-ae6d-dde2ab3aeadb)

### How to get started

_**1. Launch the Program**_

When the application starts, you will see the canvas visualization area and the main menu. This is where you can begin designing your robotic manipulator.

_**2. Build Your Robot**_

Use the menu options to construct your robot model. Ensure to build the model first, as this is necessary for accessing other features. You can also load a pre-configured model, such as the Thor robotic arm, for quick setup.

_**3. Save and Load Models**_

Save your robotic models for future projects, or load existing ones to resume work.

_**4. Analyze Kinematics**_

Once the robotic model is built, explore kinematics options, including forward, inverse, and differential kinematics, via the analysis menus.

_**5. Connect and Control**_

Connect your physical robotic arm (e.g., Thor robotic arm) through a serial link to an Arduino microcontroller. Use the control menu to set the serial port and the baud rate, adjust the motor settings, and send commands, with the serial communication visible in the integrated serial monitor. Fine-tune the movements of the real robotic manipulator with sliders, buttons, and other controls.
