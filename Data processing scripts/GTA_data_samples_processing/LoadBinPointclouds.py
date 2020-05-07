
from pathlib import Path
import numpy as np
import os.path
import math

def loadKittiVelodyneFile(file_path, include_luminance = False):
    '''
    Loads a kitti velodyne file (ex: 000000.bin) into a list of tuples, where each tuple has (x, y, z) or (x, y, z, l)
    Right now it discards the 4th vaule of each point, i.e. the luminance
    Argument:
        - include_luminance: if the function should also store the pont intensisty value in the list of points
    '''
    # Source: https://github.com/hunse/kitti/blob/master/kitti/velodyne.py
    points = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    points = points[:, :3]  # exclude luminance

    point_tuple_list = []
    for i in range(len(points)):
        point_tuple_list.append((points[i][0], points[i][1], points[i][2],))

    return point_tuple_list

def savePlyFile(filepath, tuple_list, attributes = None, color_for_every_point = (0, 255, 0)):
    '''
    For testing in the Main.py file
    Save list of points (possibly with attributes such as color) into a .PLY formated file
    Arguments: 
        - tuple_list: list of points and their attributes
        - attributes: to indicate what type of attributes are included in the points:
            - c: each point has position + color (r, g, b)
    '''
    with open(filepath, "w") as the_file:
        header_lines = ["ply", "format ascii 1.0"]
        header_lines.append("element vertex " + str(len(tuple_list)))
        header_lines.append("property float x")
        header_lines.append("property float y")
        header_lines.append("property float z")

        # if point have color 
        if attributes == "c":
            header_lines.append("property uchar red")
            header_lines.append("property uchar green")
            header_lines.append("property uchar blue")
        
        header_lines.append("end_header")
        
        for i in range(0, len(header_lines)):
            the_file.write(header_lines[i] + "\n")

        for i in range(0, len(tuple_list)):
            if attributes == "c" and len(tuple_list[i]) <= 3:    # if the points dont have color, but the attributes is set to "c"
                new_tuple = (tuple_list[i][0], tuple_list[i][1], tuple_list[i][2], color_for_every_point[0], color_for_every_point[1], color_for_every_point[2])
                the_file.write(tupleToStr(new_tuple) + "\n")
            else:
                the_file.write(tupleToStr(tuple_list[i]) + "\n")

def tupleToStr(tuple):
    '''
    Converts a tuple of N size into a string, where each element is separated by a space.
    Arguments:
    - tuple: tuple to be converted into string
    Returns:
        - string with the tuple values
    '''
    tuple_string = ""
    for i in range(0, len(tuple)):
        if i == (len(tuple)-1):
            tuple_string += str(tuple[i])
        else:
            tuple_string += str(tuple[i]) + " "

    return tuple_string