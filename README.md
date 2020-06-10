# Deep Learning Classification using 3D Point Clouds 

## Objective

Base of the work below, but use Stereo image for depth estimation and simulate a LiDAR

## Table of Contents
- [Objective](#objective)
- [Dataset](#dataset)
- [Software](#software)
- [Testing](#testing)
- [Visualization of point clouds](#visualization-of-point-clouds)
- [Workflow for gathering point cloud data](#workflow-for-gathering-point-cloud-data)
- [TODO](#todo)
- [Resources](#resources)

## Objective

In autonomous driving, the vehicle needs to have a clear understanding about its surroundings in order to avoid collisions, to follow the traffic rules and to safely drive the passengers to their destinations. To achieve these goals, there's a need for a system that can detect and classify objects, with a high level of accuracy, from multiple types of sensors. 

This project aims to process and use the data gathered from a camera and LiDAR system to provide the detection and classification of 3D objects. 

For more details check the [wiki](https://github.com/Diogo525/DL-Classification-using-3D-Point-Clouds/wiki).

## Dataset

The dataset of colored 3D point clouds is obtained through the simulation of a camera and LiDAR sensor in the GTA V overworld. The resolution of the resulting point clouds is specified in the [LiDAR GTA V.cfg](https://github.com/Diogo525/DL-Classification-using-3D-Point-Clouds/blob/master/Data%20processing%20scripts/LiDAR%20GTA%20V.cfg) file. The resulting GTA V mod file that accomplishes the task of data gathering is **LiDAR GTA V.asi** (found in the output directory of the project: bin/Release/LiDAR GTA V.asi).

## Software

- GTA V  
- Blender 2.8
- Microsoft Visual Studio 2017
- Point Cloud Visualizer addon for Blender


## Testing:

In order to test this mod to generate colored 3D point cloud data, the follow steps must be followed:

1. Install the Grand Theft Auto V game.
2. Setup [ScriptHookV by Alexander Blade](http://www.dev-c.com/gtav/scripthookv/)
3. Copy the **LiDAR GTA V.asi** file to the **Rockstar Games\Grand Theft Auto V** directory
4. In the same path, create a directory called **LiDAR GTA V** and add the scripts in the [Data processing scripts](https://github.com/Diogo525/DL-Classification-using-3D-Point-Clouds/tree/master/Data%20processing%20scripts) directory to it
5. Start the game and load the **Story Mode**
6. After the game finishes loading, the following keyboard shortcuts need to be changed (the key remapping menu can be accessed by selecting **Esc** > **SETTINGS** > **General**):
    - "Switch to GTA Online" to **Numpad +**
    - "Start / Stop recording" to **Numpad -**
    - "Start / Stop Action Replay Recording" to **Numpad &ast;**
    - "Cancel Recording" to **Numpad /**
7. Close the menu and use the following keyboard shortcuts to execute the mod functionality:
    - **F1**: show a notification in the lower-left corner of the screen information about the mod key mappings
    - **F2**: perform a single lidar scan in the current character position and rotation
    - **F3**: start/stop the recording of the player position
    - **F5**: start/stop the automatic data gathering with LiDAR and camera
8. After finishing the automatic LiDAR scanning, exit from GTA V and open the directory previously created in step 4. A sequence of directories by the names:
    - LiDAR_PointCloud1
    - LiDAR_PointCloud2
    - (...)
    - LiDAR_PointCloudX
    
-will be found here. Each directory holds the 360º pictures (of the day, night and cloudy weather) and point clouds (the ideal and the point cloud with added errors). 

9. In order to obtain the colored point clouds, execute the batch script **\_JoinAllDataIntoAFolder.bat**, which will generate them and create a new directory where all of them will reside.

## Visualization of point clouds

To visualize a point cloud file with a **.ply** extension, the software **Blender v2.8** was used with the **Point Cloud Visualizer addon** activated. A video tutorial on how to use this addon can be found [here](https://www.youtube.com/watch?v=dTP1Sv7CkjQ).

## Workflow for gathering point cloud data

Here is a demonstration of the process involved in obtaining sequential point clouds from GTA V:

[![Watch the video](https://github.com/Diogo525/DL-Classification-using-3D-Point-Clouds/blob/master/_resources/thumbnail_workflow.PNG)](https://www.youtube.com/watch?v=r82ptMSAvQM)

## TODO

- Align camera with player direction
- Complete the slope correction solution

## Resources

- Original LiDAR simulation mod for GTA V
     - https://github.com/UsmanJafri/LiDAR-GTA-V
- Point Cloud Visualizer addon for Blender
     - https://github.com/uhlik/bpy
     - https://www.youtube.com/watch?v=dTP1Sv7CkjQ
