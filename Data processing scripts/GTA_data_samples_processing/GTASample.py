import math
import json
import numpy as np
import random
import cv2
import os.path
import numpy as np
from PCRaw import PCRaw
from PCLabeledObject import PCLabeledObject
from GTAView import GTAView

class GTASample:
    '''
    Class representing all the information belonging to a sample
    generated in GTA V.
    Abreviations:
        - pc: point cloud
        - fn: filename
        - fv: front view
    '''
    # .ply file with the point cloud (stores the positions of the points)
    pc_ply_fn = "LiDAR_PointCloud.ply"
    # file with a label for each point of the point cloud (background, pedestrian, vehicle, props)
    pc_labels_fn = "LiDAR_PointCloud_labels.txt"
    # file with an id for every gameobject in the point cloud
    pc_labels_detailed_fn = "LiDAR_PointCloud_labelsDetailed.txt"
    # file that contains point position (x, y, z), point projected (x, y) pixels, and the index of the view that it's projected on (0, 1, or 2)
    # the index 0 means front view image
    pc_projected_points_fn = "LiDAR_PointCloud_points.txt"
    # original front view image file (res equal to the screen resolution - 1920x1080 in my case)
    fv_img_fn = "LiDAR_PointCloud_Camera_Print_Day_0.bmp"

    # rotation of the character when lidar scan happened
    rotation_fn = "LiDAR_PointCloud_rotation.txt"

    # PCraw instance containing all the raw point cloud information
    pc_raw_data = None
    # PCRaw instance containing all the points thar are projected onto the front view image
    pc_fv_raw_data = None

    # GTAView object
    imageView = None

    def __init__(self, sample_directory_path):
        '''
        Constructor
        Arguments:
        - sample_directory_path: path to the directory where the ppoint cloud sample files are located
        - character_rotation: rotation (in degrees) of the in-game character when the lidar scanner happened
        '''
        self.directory_path = sample_directory_path

        # a float
        char_rotation = float(self.load_txt_file_into_str_list(self.rotation_fn)[0].split(' ')[2]) # retrieves "rotx roty rotz" from file

        self.imageView = GTAView(sample_directory_path, self.fv_img_fn)

        #### Core calculations over the point cloud ####

        # tuple list with all the points of the point cloud, each point is a tuple (x, y, z)
        # fill in the point cloud list
        original_pc = self.load_ply_file_into_tuple_list(self.pc_ply_fn)

        # load list of integers with the labels (background (0), pedestrian (1), vehicle (2), game props (3))
        point_labels = self.load_txt_file_into_int_list(self.pc_labels_fn)

        # load list of integers with detailed labels, i.e., each point is associated to the id of the correspondent object
        point_labels_detailed = self.load_txt_file_into_int_list(self.pc_labels_detailed_fn)

        # load file with the points + projected coords and view index
        point_projections = self.load_txt_file_into_tuple_float_list(self.pc_projected_points_fn, [3, 4, 5], [0, 1, 2])   # 3, 4, 5 correspond to integer values of projx,, projy, view_index

        self.pc_raw_data = PCRaw(original_pc, point_labels, point_labels_detailed, point_projections, character_rot=char_rotation, debug_mode=True, pc_name="Original")

        # eliminate all points that are not projected onto the first view (index 0)
        # and store the remaining points in a list of tuples
        fv_pc, fv_pc_labels, fv_pc_labels_detailed, fv_pc_projected = self.create_frontview_pc(self.pc_raw_data.list_raw_pc, self.pc_raw_data.list_raw_labels, self.pc_raw_data.list_raw_detailed_labels, self.pc_raw_data.list_raw_projected_points)

        self.pc_fv_raw_data = PCRaw(fv_pc, fv_pc_labels, fv_pc_labels_detailed, fv_pc_projected,character_rot=char_rotation, debug_mode=True, pc_name="Front view")

    def load_ply_file_into_tuple_list(self, filename):
        '''
        Ignores the .PLY header and only returns the list of points within the file.
        Arguments:
            - filename: name of the .ply to load
        Returns:
            - list of tuples, where each tuple has the point attributtes present in the file
        '''
        # list of strings
        tmp_ply_content = self.load_txt_file_into_str_list(filename)
        tuple_list = []

        for i in range(0, len(tmp_ply_content)):
            string_values_list = tmp_ply_content[i].rstrip().split(" ")

            if self.is_number(string_values_list[0]):
                tuple_list.append(self.str_to_tuple(tmp_ply_content[i]))

        return tuple_list

    def load_txt_file_into_str_list(self, filename):
        '''
        Loads file into a list of strings. Each line of the file will be an element of the list.
        Arguments:
            - filename of a file where each line is to be treated as a string
        Return:
            - list of strings containing the file lines
        '''
        lines = []
        with open(os.path.join(self.directory_path, filename)) as file_in:
            for line in file_in:
                lines.append(line)
        return lines

    def load_txt_file_into_int_list(self, filename):
        '''
        Loads file into a list of integer values
        Arguments: 
            - filename of the file where each line is to be treated as an int
        Returns:
            - list of ints
        '''
        lines = []
        with open(os.path.join(self.directory_path, filename)) as file_in:
            for line in file_in:
                lines.append(int(line))
        return lines

    def load_txt_file_into_tuple_float_list(self, filename, integer_indices_list = [], ignore_indices = []):
        '''
        Loads file into a list of strings, where each line corresponds to an element of the list
        Arguments:
            - filename of a file where each line has float values separated by spaces
            - integer_indices_list: indices of the values that are to be stored as integers
            - ignore_indices: indices of the values not to be included in the tuples of the new list
        Return:
            - list of tuples containing the float values
        '''
        lines = []
        with open(os.path.join(self.directory_path, filename)) as file_in:
            for line in file_in:
                tmp_tuple = self.str_to_tuple(line) # contains all values as floats
                tuple = ()  # can contain integer values
                for i in range(0, len(tmp_tuple)):
                    if i in integer_indices_list:
                        tuple += (int(tmp_tuple[i]), )
                    else:
                        include = True
                        for j in range(0, len(ignore_indices)):
                            if i == j:
                                include = False
                        if include:
                            tuple += (tmp_tuple[i], )

                lines.append(tuple)

        return lines

    def load_kitti_velodyne_file(self, file_path, include_luminance = False):
        '''
        Loads a kitti velodyne file (ex: 000000.bin) into a list of tuples, where each tuple has (x, y, z) or (x, y, z, l)
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

    def save_ply_file(self, filename, tuple_list, attributes = None):
        '''
        Save list of points (possibly with attributes such as color) into a .PLY formated file
        Arguments: 
            - tuple_list: list of points and their attributes
            - attributes: to indicate what type of attributes are included in the points:
                - c: each point has position + color (r, g, b)
        '''
        with open(os.path.join(self.directory_path, filename), "w") as the_file:
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
                the_file.write(self.tuple_to_str(tuple_list[i]) + "\n")

    def save_ply_file_from_dict(self, filename, dict, attributes = None):
        '''
        Save dictionary of lists of points (possibly with attributes such as color) into a .PLY formated file
        Arguments: 
            - dict: dictionary of list of points and their attributes
            - attributes: to indicate what type of attributes are included in the points:
                - c: each point has position + color (r, g, b)
        '''
        with open(os.path.join(self.directory_path, filename), "w") as the_file:

            count_points = 0
            for key in dict.keys():
                count_points += len(dict[key])

            header_lines = ["ply", "format ascii 1.0"]
            header_lines.append("element vertex " + str(count_points))
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

            for key in dict.keys():
                for i in range(0, len(dict[key])):
                    the_file.write(self.tuple_to_str(dict[key][i]) + "\n")

    def is_number(self, s):
        '''
        https://stackoverflow.com/questions/354038/how-do-i-check-if-a-string-is-a-number-float
        '''
        try:
            float(s)
            return True
        except ValueError:
            return False

    def str_to_tuple(self, string):
        '''
        Converts a string with float values separated by spaces.
        Arguments:
            - string with float values separated by a space
        Returns:
            - tuple with float values
        '''
        string_values_list = string.rstrip().split(" ")
        t = ()
        for i in range(0, len(string_values_list)):
            t += (float(string_values_list[i]),)

        return t

    def tuple_to_str(self, tuple):
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

    def create_frontview_pc(self, point_cloud, point_cloud_labels, point_cloud_detailed_labels, point_projections_list):
        '''
        Creates a point cloud only with the points that are projected onto the front view (index 0).
        It takes the view indices from the point_projections_list, and the points from the rotated point cloud.
        It also creates a tuple list for the labels and the labels_detailed of the new front view point cloud.
        Arguments: 
            - list of tuples containing the points position, projx, projy and view index. The view index is the sixth value of each line in the file.
        Returns:
            - a new point cloud containing only the points that are projected onto the front view image
            - and respective tuple list of labels and labels_detailed, and the list of tuples containing projx, ptojy
        '''
        trimmed_pc = []
        trimmed_pc_labels = []
        trimmed_pc_labels_detailed = []
        trimmed_projected = []
        for i in range(0, len(point_projections_list)):
            if point_projections_list[i][2] == 0:
                trimmed_pc.append(point_cloud[i])
                trimmed_pc_labels.append(point_cloud_labels[i])
                trimmed_pc_labels_detailed.append(point_cloud_detailed_labels[i])
                trimmed_projected.append(point_projections_list[i]) # stores (projx, projy) tuples

        return trimmed_pc, trimmed_pc_labels, trimmed_pc_labels_detailed, trimmed_projected

   
        







