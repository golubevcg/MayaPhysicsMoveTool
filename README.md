# PhysicsMoveTool for Autodesk Maya

## Overview
PhysicsMoveTool is a preliminary plugin for Autodesk Maya, offering a glimpse into real-time collision detection in the viewport. Ideal for basic usage, this early iteration enables users to move objects within the viewport with dynamic collision responses. While it provides a novel approach to interaction in Maya, it's best suited for simple scenes with limited geometry, as performance may vary with more complex environments. This tool represents a foundational step towards more robust future developments.

## System Requirements
- **Operating System:** Windows 10 x64
- **IDE:** Visual Studio 2019 (version 7.8 or later)
- **Windows SDK:** Version 10.0.10586.0
- **Physics Engine:** bullet3 (version 3.25 or later)

## Building the Plugin
### Prerequisites
- Autodesk Maya Developer Kit (devkitBase) placed in the `external` folder
- bullet3 library (either a compiled version or source code) in the `external` folder

### Build Instructions
1. Run `build.bat` to initiate the build process. This script generates a `build` folder containing the Visual Studio solution.
2. Open the solution in Visual Studio and compile the plugin.

## Installation and Launch
1. Build the release version of the plugin.
2. Start Autodesk Maya.
3. Open `maya_launch.py` from the root of this repository. Update the path in the script to point to your repository location.
4. Execute the script in Maya's console to launch the tool.

## Usage
PhysicsMoveTool enhances Maya's viewport by adding real-time collision detection for moving objects. Test the functionality using the `simple_test_scene.mb` located in the `test` directory. As you move objects around, they will exhibit realistic physical interactions, avoiding overlap and responding to collisions appropriately.
![difftool_gif](readme_images/collision_detection_example.gif)