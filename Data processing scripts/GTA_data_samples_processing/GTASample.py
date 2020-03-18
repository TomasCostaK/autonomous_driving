import math
import json
import numpy as np
import random
import cv2
import os.path

'''
Class representing all the information belonging to a sample
generated in GTA V.
Abreviations:
    - pc: point cloud
    - fn: filename
    - fv: front view
'''
class GTASample:
    # .ply file with the point cloud (stores the positions of the points)
    pc_ply_fn = "LiDAR_PointCloud.ply"
    # file with a label for each point of the point cloud (background, pedestrian, vehicle, props)
    pc_labels_fn = "LiDAR_PointCloud_labels.txt"
    # file with an id for every gameobject in the point cloud
    pc_labels_detailed_fn = "LiDAR_PointCloud_labelsDetailed.txt"
    # file that contains point position (x, y, z), point projected (x, y) pixels, and the index of the view that it's projected on (0, 1, or 2)
    # the index 0 means front view image
    pc_projected_points_fn = "LiDAR_PointCloud_points.txt"
    # original front view image file (res equal to the screen resolution - 1920x1080)
    fv_img_fn = "LiDAR_PointCloud_Camera_Print_Day_0.bmp"
    # kitti size: 1224x370
    fv_img_kitti_fn = "LiDAR_PointCloud_Camera_Print_Day_0_rect.png"

    def __init__(self, sample_directory_path, character_rotation):
        '''
        Constructor
        Arguments:
            - sample_directory_path: path to the directory where the ppoint cloud sample files are located
            - character_rotation: rotation (in degrees) of the in-game character when the lidar scanner happened
        '''
        self.directory_path = sample_directory_path
        self.rotation_amount = self.degrees_to_rad(-character_rotation)  # rotation around z axis, in radians

        # tuple list with all the points of the point cloud, each point is a tuple (x, y, z)
        # fill in the point cloud list
        self.original_pc = self.load_ply_file_into_tuple_list(self.pc_ply_fn)

        # get rotated point cloud in order to be oriented towards the character's forward direction
        self.rotated_pc = self.rotate_pc_around_z_axis(self.original_pc)

        # load list of integers with the labels (background (0), pedestrian (1), vehicle (2), game props (3))
        self.point_labels = self.load_txt_file_into_int_list(self.pc_labels_fn)
        
        # load list of integers with detailed labels, i.e., each point is associated to the id of the correspondent object
        self.point_labels_detailed = self.load_txt_file_into_int_list(self.pc_labels_detailed_fn)
        
        # load file with the points + projected coords and view index
        self.point_projections = self.load_txt_file_into_tuple_float_list(self.pc_projected_points_fn, [3, 4, 5])   # 3, 4, 5 correspond to integer values of projx,, projy, view_index
        
        # eliminate all points that are not projected onto the first view (index 0)
        # and store the remaining points in a list of tuples
        self.fv_pc, self.fv_pc_labels, self.fv_pc_labels_detailed, self.fv_pc_projected = self.create_frontview_pc(self.point_projections)

        # obtain a point cloud with only vehicles, correspondent detailed_labels list and projection list
        self.vehicles_pc, self.vehicles_detailed_labels, self.vehicles_projected = self.get_points_with_label(self.fv_pc, self.fv_pc_labels, self.fv_pc_labels_detailed, self.fv_pc_projected, 2)

        # obtain list with all different object ids in the vehicles point cloud
        self.vehicle_ids = self.get_individual_objects_ids(self.vehicles_detailed_labels)

        # dictionary where each value is a list of point (with position + rgb) belonging to each vehicle
        self.dict_with_vehicle_colored_points, self.dict_vehicle_projected_points = self.get_individual_objects_dictionaries(self.vehicles_pc, self.vehicles_detailed_labels, self.vehicle_ids, self.vehicles_projected)

        # obtain dict where each vehicle will have a list of 4 values: mixX, maxX, minY, maxY
        self.dict_vehicles_2d_bb = self.calculate_2d_bb_around_same_labeled_objs(self.dict_with_vehicle_colored_points, self.vehicle_ids, self.dict_vehicle_projected_points)

        print(self.dict_vehicles_2d_bb)

    # https://stackoverflow.com/questions/354038/how-do-i-check-if-a-string-is-a-number-float
    def is_number(self, s):
        try:
            float(s)
            return True
        except ValueError:
            return False

    def create_frontview_pc(self, point_projections_list):
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
            if point_projections_list[i][5] == 0:
                trimmed_pc.append(self.rotated_pc[i])
                trimmed_pc_labels.append(self.point_labels[i])
                trimmed_pc_labels_detailed.append(self.point_labels_detailed[i])
                trimmed_projected.append((point_projections_list[i][3], point_projections_list[i][4])) # stores (projx, projy) tuples

        return trimmed_pc, trimmed_pc_labels, trimmed_pc_labels_detailed, trimmed_projected

    def get_points_with_label(self, point_list, labels_list, labels_detailed_list, point_projections_list, label):
        '''
        Creates a list with all the points with a label.
        Arguments:
            - point_list: list of tuples containing the point cloud point positions
            - labels_list: list of ints containing the labels
            - labels_detailed_list: list of ints containing the detailed labels
            - projections_list: list of tuples (projX, projY)
            - label: label of the objects to finds (0: backgrouns, 1: pedestrians, 2: vehicles, 3: gameprops)
        Returns:
            - list of tuples with point positions, i.e., point cloud only containing points with the indicated label
            - list of ints containing the detailed labels that diferentiates between different objects of the same label
            - list of tuples (projx, projy) corresponding to the object of interest (with the labe specified)
        '''
        points = []
        detailed_labels_list = []
        projected_point_list = []
        for i in range(0, len(point_list)):
            if labels_list[i] == label:
                points.append(point_list[i])
                detailed_labels_list.append(labels_detailed_list[i])
                projected_point_list.append(point_projections_list[i])

        return points, detailed_labels_list, projected_point_list

    def get_individual_objects_ids(self, labels_detailed_list):
        '''
        Creates a list with all the different ids in a point cloud with objects of the same label.
        Arguments:
            - labels_detailed_list: list of ints (detailed_labels) of a point cloud only with points of the same label
        Returns:
            - list of integers with thte different ids found
        '''
        
        new_list = []
        for i in range(0, len(labels_detailed_list)):
            if labels_detailed_list[i] not in new_list:
                new_list.append(labels_detailed_list[i])

        return new_list

    def get_individual_objects_dictionaries(self, point_list, labels_detailed_list, object_ids_list, obj_projected_coords):
        '''
        Creates a dictionary with positions + color per vehicle, and a dictionary of the correpondent projected coordinates.
        Arguments:
            - point_list: point cloud with points belongin to the same label
            - labels_detailed_list: list of ints (detailed_labels) of a point cloud only with points of the same label
            - object_ids_list: list of the different detailed_labels of the in the point_list
        Returns:
            - dictionary with lists of tuples (x, y, z, r, g, b) corresponding to various cars in thte point cloud. The keys are the objects ids (detailed labels).
            - dictionary of projected points, where each key is an object id and the value is a list of 2 values (projx, projy)
        '''

        # create random colors for the different object ids
        color_per_vehicle_dict = {}     # dicionario onde cada key é um gameobject id, e onde cada valor é uma string a indicar a cor rgb. 
        for i in range(0, len(object_ids_list)):
            # generate random rgb color for the id of the current iteration
            r = random.randint(0, 256)   # random value between [0, 255]
            g = random.randint(0, 256)
            b = random.randint(0, 256)
            color_per_vehicle_dict[object_ids_list[i]] = (r, g, b) 

        # dictionary
        dict_of_points_per_obj = {}
        dict_projected_coords_per_obj = {}

        for i in range(0, len(point_list)):
            if labels_detailed_list[i] not in dict_of_points_per_obj.keys():
                dict_of_points_per_obj[labels_detailed_list[i]] = []
                dict_projected_coords_per_obj[labels_detailed_list[i]] = []

            dict_of_points_per_obj[labels_detailed_list[i]].append((point_list[i][0], point_list[i][1], point_list[i][2], 
                                                                    color_per_vehicle_dict[labels_detailed_list[i]][0],
                                                                    color_per_vehicle_dict[labels_detailed_list[i]][1], 
                                                                    color_per_vehicle_dict[labels_detailed_list[i]][2]))
            
            dict_projected_coords_per_obj[labels_detailed_list[i]].append((obj_projected_coords[i][0], obj_projected_coords[i][1]))

        return dict_of_points_per_obj, dict_projected_coords_per_obj

    def calculate_2d_bb_around_same_labeled_objs(self, dict_with_same_labeled_objs, object_ids, dict_objs_projected_points):
        '''
        Determines the minX, maxX, minY, maxY for a 2d bounding box for each object.
        Arguments:
            - dict_with_same_labeled_objs: dictionary containing the same labeled objects (has position + projX, projY + view index)
            - object_ids: list of ids of the objects in the dictionary
            Returns:
                - a dictionary where each key is an object id, and value is a list of 4 values in the order: mixX, maxX, minY, maxY
        '''
        # cada carro vai ter uma lista de 4 integers, correspondentes às coordenadas mixX, maxX, minY, maxY, que vao originar os cantos da bounding box correspondente
        bounding_box_2d_for_each_car = {}
        
        for i in range(0, len(object_ids)):
            object_point_list = dict_with_same_labeled_objs[object_ids[i]]    # lista de tuplos de 5 elementos  (coordx, coordy, coord, projx, projy)
            object_proj_point_list = dict_objs_projected_points[object_ids[i]]

            tmp_minX = 0
            tmp_maxX = 0
            tmp_minY = 0
            tmp_minY = 0

            # get the min X projected value of the current object
            for j in range(0, len(object_point_list)):
                if j == 0:  # first iteration, no previous points to compare with
                    tmp_minX = object_proj_point_list[j][0]
                    continue
                
                if object_proj_point_list[j][0] < tmp_minX:   # se o x for menor q o do ponto atualmente selecionado
                    tmp_minX = object_proj_point_list[j][0]

            bounding_box_2d_for_each_car[object_ids[i]] = []
            bounding_box_2d_for_each_car[object_ids[i]].append(tmp_minX)

            # get the max X projected value of the current object
            for j in range(0, len(object_point_list)):
                if j == 0:  # first iteration, no previous points to compare with
                    tmp_maxX = object_proj_point_list[j][0]
                    continue
                
                if object_proj_point_list[j][0] > tmp_maxX:   # se o x for menor q o do ponto atualmente selecionado
                    tmp_maxX = object_proj_point_list[j][0]

            bounding_box_2d_for_each_car[object_ids[i]].append(tmp_maxX)

            # get the min Y projected value of the current object
            for j in range(0, len(object_point_list)):
                if j == 0:  # first iteration, no previous points to compare with
                    tmp_minY = object_proj_point_list[j][1]
                    continue
                
                if object_proj_point_list[j][1] < tmp_minY:   # se o x for menor q o do ponto atualmente selecionado
                    tmp_minY = object_proj_point_list[j][1]

            bounding_box_2d_for_each_car[object_ids[i]].append(tmp_minY)

            # get the max Y projected value of the current object
            for j in range(0, len(object_point_list)):
                if j == 0:  # first iteration, no previous points to compare with
                    tmp_maxY = object_proj_point_list[j][1]
                    continue
                
                if object_proj_point_list[j][1] > tmp_maxY:   # se o x for menor q o do ponto atualmente selecionado
                    tmp_maxY = object_proj_point_list[j][1]

            bounding_box_2d_for_each_car[object_ids[i]].append(tmp_maxY)

        return bounding_box_2d_for_each_car

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

        # store ply header

    def save_ply_file(self, filename, tuple_list, attributes = None):
        '''
        Save list of points (possibly with attributes such as color) into a .PLY formated file
        Arguments: 
            - tuple_list: list of points and their attributes
            - attributes: to indicate what type of attributes are included in the points:
                - c: each point has position + color (r, g, b)
        '''
        with open(filename, "w") as the_file:
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
        with open(filename, "w") as the_file:

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

    def degrees_to_rad(self, angle_degrees):
        '''
        Converts degrees into radians.
        Returns:
            - Float angle in radian
        '''
        return angle_degrees * (math.pi/180)

    def load_txt_file_into_str_list(self, filename):
        '''
        Loads file into a list of strings. Each line of the file will be an element of the list.
        Arguments:
            - filename of a file where each line is to be treated as a string
        Return:
            - list of strings containing the file lines
        '''
        lines = []
        with open(filename) as file_in:
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
        with open(filename) as file_in:
            for line in file_in:
                lines.append(int(line))
        return lines

    def load_txt_file_into_tuple_float_list(self, filename, integer_indices_list = []):
        '''
        Loads file into a list of strings, where each line corresponds to an element of the list
        Arguments:
            - filename of a file where each line has float values separated by spaces
            - integer_indices_list: indices of the values that are to be stored as integers
        Return:
            - list of tuples containing the float values
        '''
        lines = []
        with open(filename) as file_in:
            for line in file_in:
                tmp_tuple = self.str_to_tuple(line) # contains all values as floats
                tuple = ()  # can contain integer values
                for i in range(0, len(tmp_tuple)):
                    if i in integer_indices_list:
                        tuple += (int(tmp_tuple[i]), )
                    else:
                        tuple += (tmp_tuple[i], )

                lines.append(tuple)

        return lines

    def rotate_point_around_z_axis(self, point, angle_rad):
        '''
        Rotate a point around the z axis.
        Arguments:
            - point: tuple with 3 elements (x, y, z) corresponding the point to be rotated
            - angle_rad: angle to rotate the point around the z (up) axis
        Returns:
            - tuple with the rotated point coordinates (x, y, z)
        '''
        p_x, p_y, p_z = point

        r_x = (p_x)*math.cos(angle_rad) - (p_y)*math.sin(angle_rad)
        r_y = (p_x)*math.sin(angle_rad) + (p_y)*math.cos(angle_rad)
        r_z = (p_z)

        return (r_x, r_y, r_z) 

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

    def rotate_pc_around_z_axis(self, point_tuple_list):
        '''
        Rotates the entire point cloud around the z (up) axis.
        Arguments:
            - tuple list with all the point cloud points.
        Returns:
            - tuple list with the rotated point cloud points
        '''
        rot_point_tuple_list = []
        for i in range(0, len(point_tuple_list)):
            rot_point_tuple = self.rotate_point_around_z_axis(point_tuple_list[i], self.rotation_amount)
            rot_point_tuple_list.append(rot_point_tuple)

        return rot_point_tuple_list

    def show_objects_bounding_boxes(self, dict_bounding_box_2d_coords, image_view_fn, object_ids):
        image  = cv2.imread(image_view_fn)  #np.zeros((1080, 1920, 3), np.uint8)

        for i in object_ids:
            minx = dict_bounding_box_2d_coords[i][0]
            maxx = dict_bounding_box_2d_coords[i][1]
            miny = dict_bounding_box_2d_coords[i][2]
            maxy = dict_bounding_box_2d_coords[i][3]
            cv2.rectangle(image, (int(minx), int(miny)), (int(maxx), int(maxy)), (0, 0, 255), 3)

        cv2.imshow("Image", image)
        print("Press any key to continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()



sample = GTASample('.', -133.792633)

sample.save_ply_file("test_rotated.ply", sample.rotated_pc)

sample.save_ply_file("test_trimmed.ply", sample.fv_pc)

sample.save_ply_file("test_vehicles.ply", sample.vehicles_pc)

sample.save_ply_file_from_dict("test_vehicles_colored.ply", sample.dict_with_vehicle_colored_points, attributes = 'c')

sample.show_objects_bounding_boxes(sample.dict_vehicles_2d_bb, sample.fv_img_fn, sample.vehicle_ids)





















