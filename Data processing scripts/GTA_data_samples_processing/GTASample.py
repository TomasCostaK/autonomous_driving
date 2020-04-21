import math
import json
import numpy as np
import random
import cv2
import os.path
import numpy as np
from GtaView import GtaView
from PcRaw import PcRaw
from PcLabeledObject import PcLabeledObject

class GtaSample:
    '''
    Class representing all the information belonging to a sample
    generated in GTA V.
    Abreviations:
        - pc: point cloud
        - Fn: filename
        - fv: front view
    '''
    ##### Input dara files #####

    # .ply file with the point cloud (stores the positions of the points)
    pcPlyFn = "LiDAR_PointCloud.ply"
    # file with pointwise labels (background, pedestrian, vehicle, props)
    pcLabelsFn = "LiDAR_PointCloud_labels.txt"
    # file with an id for every gameobject in the point cloud
    pcLabelsDetailedFn = "LiDAR_PointCloud_labelsDetailed.txt"
    # file that contains points position (x, y, z), point projected (x, y) pixels, and the index of the view that it's projected onto (0, 1, or 2)
    pcProjectedPointsFn = "LiDAR_PointCloud_points.txt"
    # original front view image file (resolution equal to the screen resolution - 1920x1080 in my case)
    fvImgFn = "LiDAR_PointCloud_Camera_Print_Day_0.bmp"
    # rotation of the character (and camera) when the lidar scan happened
    rotationFn = "LiDAR_PointCloud_rotation.txt"
    # vehicle information file to produce their bounding boxes
    vehiclesInfoFn = "LiDAR_PointCloud_vehicles_dims.txt"

    ##### Data structures for holding point cloud information ######

    # PcRaw instance containing all the raw point cloud information
    pcData = None
    # PcRaw instance containing all the points that are projected onto the front view image
    pcFvData = None
    # GtaView instance
    imageView = None

    def __init__(self, sampleDirPath):
        '''
        Constructor
        Arguments:
        - sample_directory_path: path to the directory where the ppoint cloud sample files are located
        - character_rotation: rotation (in degrees) of the in-game character when the lidar scanner happened
        '''

        self.directory_path = sampleDirPath

        # -camRot makes point cloud facing the y direction in the right handed coord system, and -90 makes the point cloud face de x direction
        # get Z rotation of the camera (character) stored in file
        self.camRotation = - (float(self.loadTxtFileIntoStrList(self.rotationFn)[0].split(' ')[2])) - 90

        self.imageView = GtaView(sampleDirPath, self.fvImgFn)

        #### Core calculations over the point cloud ####

        # tuple list with all the points of the point cloud, each point is a tuple (x, y, z)
        originalPc = self.loadPlyFileIntoTupleList(self.pcPlyFn)

        # load list of integers with the labels (background (0), pedestrian (1), vehicle (2), game props (3))
        pointLabels = self.loadTxtFileIntoIntList(self.pcLabelsFn)

        # load list of integers with detailed labels, i.e., each point is associated to the id of the correspondent object
        pointLabelsDetailed = self.loadTxtFileIntoIntList(self.pcLabelsDetailedFn)

        # load file with the points + projected coords and view index
        pointProjections = self.loadTxtFileIntoTupleFloatList(self.pcProjectedPointsFn, [3, 4, 5], [0, 1, 2])   # 3, 4, 5 correspond to integer values of projx, projy, view_index

        self.pcData = PcRaw(originalPc, pointLabels, pointLabelsDetailed, pointProjections, camRot=self.camRotation, debugMode=True, pcName="Original")

        # eliminate all points that are not projected onto the first view (index 0) and store the remaining points in a list of tuples
        frontviewPc, fvPcLabels, fvPcLabelsDetailed, fvPcProjected = \
            self.createFrontviewPc(self.pcData.list_rotated_raw_pc, self.pcData.list_raw_labels, self.pcData.list_raw_detailed_labels, self.pcData.list_raw_projected_points, orientedToXdirection=True)

        self.pcFvData = PcRaw(frontviewPc, fvPcLabels, fvPcLabelsDetailed, fvPcProjected, camRot=0, debugMode=True, pcName="Front view")


    def loadTxtFileIntoStrList(self, filename):
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

    def loadPlyFileIntoTupleList(self, filename):
        '''
        Ignores the .PLY header and only returns the list of points within the file.
        Arguments:
            - filename: name of the .ply to load
        Returns:
            - list of tuples, where each tuple has the point attributtes present in the file
        '''
        # list of strings
        tmp_ply_content = self.loadTxtFileIntoStrList(filename)
        tuple_list = []

        for i in range(0, len(tmp_ply_content)):
            string_values_list = tmp_ply_content[i].rstrip().split(" ")

            if self.isNumber(string_values_list[0]):
                tuple_list.append(self.strToTuple(tmp_ply_content[i]))

        return tuple_list

    def loadTxtFileIntoIntList(self, filename):
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

    def loadTxtFileIntoTupleFloatList(self, filename, integer_indices_list = [], ignore_indices = []):
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
                tmp_tuple = self.strToTuple(line) # contains all values as floats
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

    def createFrontviewPc(self, point_cloud, point_cloud_labels, point_cloud_detailed_labels, point_projections_list, orientedToXdirection = False):
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
                if orientedToXdirection:    
                    if point_cloud[i][0] < 0:   # also remove the points with x < 0
                        continue

                trimmed_pc.append(point_cloud[i])
                trimmed_pc_labels.append(point_cloud_labels[i])
                trimmed_pc_labels_detailed.append(point_cloud_detailed_labels[i])
                trimmed_projected.append(point_projections_list[i]) # stores (projx, projy) tuples

        return trimmed_pc, trimmed_pc_labels, trimmed_pc_labels_detailed, trimmed_projected

    def loadTxtFileToDict(self, filename):
        '''
        The first value in each line becomes the key and the rest the value
        Returns a dictionary list values
        '''
        dict = {}
        with open(os.path.join(self.directory_path, filename)) as file_in:
            for line in file_in:
                list = []
                line_list = line.rstrip().split(' ')
                #print(line_list)
                key = line_list[0]
                
                for i in range(1, len(line_list)):
                    list.append(line_list[i])
                
                dict[key] = list

        return dict

    def savePlyFile(self, filename, tuple_list, attributes = None):
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
                the_file.write(self.tupleToStr(tuple_list[i]) + "\n")

    def savePlyFileFromDict(self, filename, dict, attributes = None):
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
                    the_file.write(self.tupleToStr(dict[key][i]) + "\n")

    def saveListIntoTxtFile(self, list_of_str, dirname, filename):
        '''
        Store list of strings into a file
        '''
        with open(os.path.join(dirname, filename), "w") as the_file:
            for i in range(0, len(list_of_str)):
                the_file.write(list_of_str[i] + "\n")

    def isNumber(self, s):
        '''
        https://stackoverflow.com/questions/354038/how-do-i-check-if-a-string-is-a-number-float
        '''
        try:
            float(s)
            return True
        except ValueError:
            return False

    def strToTuple(self, string):
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

    def tupleToStr(self, tuple):
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










































