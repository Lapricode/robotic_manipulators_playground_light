# robotic_manipulators_playground_light

## Introduction

This is a python tkinter GUI for designing, simulating and controlling serial robotic manipulators (with open kinematic chains). It's a light edition of a bigger project located in the repository https://github.com/Lapricode/robotic_manipulators_playground. The inspiration for this work came from the open source project "Thor (3D Printable 6DOF Robotic Arm)", that you may explore in the link http://thor.angel-lm.com. As such, the GUI was first built to support mainly the Thor robotic arm, but it evolved to a more generic program for the simulation of any serial robotic arm up to 12 DOFs. It is supported on both Windows and Linux systems.

The program uses the classic Denavit - Hartenberg parameterization for the robots kinematics.

## Runnning instructions

To start the program, run the python file "robotic_manipulators_playground_light.py". The GUI can be run in both Windows and Linux. The Python libraries needed are written below:

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

There is a problem with the swift library that may have been solved by now, or not. The issue has been identified in the robotics-toolbox-python repository on GitHub: https://github.com/petercorke/robotics-toolbox-python/issues/383. The problem arises when running the swift library on Windows. Specifically, there is a discrepancy in how file paths are handled between Linux and Windows systems. The root of the issue lies in how the self.path variable is processed within the SwiftRoute.py file. The current implementation attempts to adjust the path by retaining the initial / character, which works fine on Linux but leads to incorrect path formatting on Windows. To address the problem on Windows, a simple adjustment can be made in the SwiftRoute.py file of the swift library. Specifically, update the block of code by modifying self.path[9:] to self.path[10:].

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

## Thor robotic arm

The Thor robotic arm is an open-source project, with the entire construction process and control code freely available (http://thor.angel-lm.com/). It was fully designed by Spanish robotics engineer Ángel Larrañaga Muro (https://www.linkedin.com/in/angellarranagamuro/) and has been continuously developed since 2016, both by the creator himself and through contributions from the global community. Its supporting structure (or body) consists of 3D-printable parts. Thor is an open kinematic chain (serial robotic manipulator) with 6 degrees of freedom (6 DOF). All of its joints are rotational, arranged in a yaw-pitch-pitch-yaw-pitch-yaw configuration (or yaw-roll-roll-yaw-roll-yaw, depending on the perspective of the x and y axes), starting from the base and extending to the end-effector.

![image](https://github.com/user-attachments/assets/dabc133e-9def-4f9e-9445-99ecb8fac507)

## GUI features

### Canvas area - visualization of the robotic arms

![image](https://github.com/user-attachments/assets/c0b0a2a0-c37f-4db5-876b-8c223946f196)

### Designing and simulating the robotic manipulators - 1st menu

![image](https://github.com/user-attachments/assets/9b0d6aa5-9686-4df7-af13-19c497f77515)

### Analysing the kinematics of the robotic manipulators - 2nd and 3rd menus

![image](https://github.com/user-attachments/assets/f87f2005-1aa6-4041-8070-1a22ca7fc524)

### Controlling the robotic manipulators - 4th menu

In the picture below, there is the original Asgard GUI from the "Thor" open source project (to the left) and my suggestion (to the right). The menu is heavily inspired from the Asgard structure, but I tried to

![image](https://github.com/user-attachments/assets/4da05154-e8e3-43e2-ae6d-dde2ab3aeadb)

