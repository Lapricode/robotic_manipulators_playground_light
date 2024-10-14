# robotic_manipulators_playground_light

## Introduction

This is a python tkinter GUI for designing, simulating and controlling serial robotic manipulators (with open kinematic chains). It's a light edition of a bigger project located in the repository https://github.com/Lapricode/robotic_manipulators_playground. The inspiration for this work came from the open source project "Thor (3D Printable 6DOF Robotic Arm)", that you may explore in the link http://thor.angel-lm.com. As such, the GUI was first built to support mainly the Thor robotic arm, but it evolved to a more generic program for the simulation of any serial robotic arm up to 12 DOFs. It is supported on both Windows and Linux systems.

The program uses the classic Denavit - Hartenberg parameterization for the robots kinematics.

## Runnning instructions

The program can be run in both Windows and Linux.

Python libraries needed:
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

To start the program run the python file "robotic_manipulators_playground.py".

## Thor robotic arm

