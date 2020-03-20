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
    # original front view image file (res equal to the screen resolution - 1920x1080 in my case)
    fv_img_fn = "LiDAR_PointCloud_Camera_Print_Day_0.bmp"
    # image taken in gta, resolution equal to the screen
    gta_img = None
    # original image resolution taken by the kitti camera
    kitti_cam_image = None
    # properly transformed image view to be equal to the images present in the kitti dataset
    kitti_image = None
    # percentage of resize used to shrink the original image view resolution down to the resolution of the kitti camera
    self.resize_percentage = None

    def __init__(self, sample_directory_path, character_rotation):
        '''
        Constructor
        Arguments:
            - sample_directory_path: path to the directory where the ppoint cloud sample files are located
            - character_rotation: rotation (in degrees) of the in-game character when the lidar scanner happened
        '''
        self.directory_path = sample_directory_path
        self.rotation_amount = self.degrees_to_rad(-character_rotation)  # rotation around z axis, in radians

        #### Core calculations over image view ####
        self.transform_image_as_kitti_dataset()

        #### Core calculations over the point cloud ####

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

        #### Optional calculations over targeted objects/labels ####

        # calculate bounding boxes around vehicles
        self.calculate_vehicles_bounding_boxes()
        
        # calculate bounding boxes around pedestrians
        self.calculate_pedestrians_bounding_boxes()
        
    #### Point cloud functions ####

    def calculate_vehicles_bounding_boxes(self):
        '''
        Executes the pipeline for calculating the vehicles bounding boxes, 
        the (mixX, maxX, minY, maxY) for every vehicle detected in the point cloud
        '''
        # obtain a point cloud with only vehicles, correspondent detailed_labels list and projection list
        self.vehicles_pc, self.vehicles_detailed_labels, self.vehicles_projected = self.get_points_with_label(self.fv_pc, self.fv_pc_labels, self.fv_pc_labels_detailed, self.fv_pc_projected, 2)

        # obtain list with all different object ids in the vehicles point cloud
        self.vehicle_ids = self.get_individual_objects_ids(self.vehicles_detailed_labels)

        # dictionary where each value is a list of point (with position + rgb) belonging to each vehicle
        self.dict_with_vehicle_colored_points, self.dict_vehicle_projected_points = self.get_individual_objects_dictionaries(self.vehicles_pc, self.vehicles_detailed_labels, self.vehicle_ids, self.vehicles_projected)

        # obtain dict where each vehicle will have a list of 4 values: mixX, maxX, minY, maxY
        self.dict_vehicles_2d_bb = self.calculate_2d_bb_around_same_labeled_objs(self.dict_with_vehicle_colored_points, self.vehicle_ids, self.dict_vehicle_projected_points)

        # dictionary with 2d bounding box coordinates for the kitti resize imgae view
        self.dict_kitti_vehicles_2d_bb = self.calculate_new_2d_bounding_boxes_for_kitti_img(self.dict_vehicles_2d_bb)

    def calculate_pedestrians_bounding_boxes(self):
        '''
        Executes the pipeline for calculating the pedestrians bounding boxes, 
        the (mixX, maxX, minY, maxY) for every pedestrian detected in the point cloud
        '''
        # obtain a point cloud with only pedestrians, correspondent detailed_labels list and projection list
        self.pedestrians_pc, self.pedestrians_detailed_labels, self.pedestrians_projected = self.get_points_with_label(self.fv_pc, self.fv_pc_labels, self.fv_pc_labels_detailed, self.fv_pc_projected, 1)

        # obtain list with all different object ids in the pedestrians point cloud
        self.pedestrian_ids = self.get_individual_objects_ids(self.pedestrians_detailed_labels)

        # dictionary where each value is a list of point (with position + rgb) belonging to each pedestrian
        self.dict_with_pedestrian_colored_points, self.dict_pedestrian_projected_points = self.get_individual_objects_dictionaries(self.pedestrians_pc, self.pedestrians_detailed_labels, self.pedestrian_ids, self.pedestrians_projected)

        # obtain dict where each pedestrian will have a list of 4 values: mixX, maxX, minY, maxY
        self.dict_pedestrians_2d_bb = self.calculate_2d_bb_around_same_labeled_objs(self.dict_with_pedestrian_colored_points, self.pedestrian_ids, self.dict_pedestrian_projected_points)

    def is_number(self, s):
        '''
        https://stackoverflow.com/questions/354038/how-do-i-check-if-a-string-is-a-number-float
        '''
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
        with open(os.path.join(self.directory_path, filename)) as file_in:
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

    def show_image_view_with_2d_bounding_boxes(self, dict_bounding_box_2d_coords, image_opencv, object_ids, color = (0, 0, 255), window_title = "Bounding box results", window_size = 0.5):
        '''
        Open a window showing the 2d bounding boxes over the given image view.
        '''
        #image  = cv2.imread(os.path.join(self.directory_path, image_view_fn))  #np.zeros((1080, 1920, 3), np.uint8)
        # copy opencv image variable to make the 2d bounding boxes not persistent
        image_copy = image_opencv.copy()

        for i in object_ids:
            minx = dict_bounding_box_2d_coords[i][0]
            maxx = dict_bounding_box_2d_coords[i][1]
            miny = dict_bounding_box_2d_coords[i][2]
            maxy = dict_bounding_box_2d_coords[i][3]
            cv2.rectangle(image_copy, (int(minx), int(miny)), (int(maxx), int(maxy)), color, 3)

        # the bounding boxes are already drawn in the image, so they will also be resized according to the given amount
        resized_image = cv2.resize(image_copy, (0,0), fx=window_size, fy=window_size) 

        cv2.imshow(window_title + ", " + str(int(window_size*100)) + "% zoom", resized_image)
        print("Press any key to continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    #### Image view functions ####

    def image_resize(self, image, width = None, height = None, inter = cv2.INTER_AREA):
        '''
        From https://stackoverflow.com/questions/44650888/resize-an-image-without-distortion-opencv
        '''
        # initialize the dimensions of the image to be resized and
        # grab the image size
        dim = None
        (h, w) = image.shape[:2]

        # if both the width and height are None, then return the
        # original image
        if width is None and height is None:
            return image

        # check to see if the width is None
        if width is None:
            # calculate the ratio of the height and construct the
            # dimensions
            r = height / float(h)
            dim = (int(w * r), height)

        # otherwise, the height is None
        else:
            # calculate the ratio of the width and construct the
            # dimensions
            r = width / float(w)
            dim = (width, int(h * r))

        # resize the image
        resized = cv2.resize(image, dim, interpolation = inter)

        # return the resized image
        return resized

    # TODO
    def point_cloud_scaling(self, scaling_factor):
        pass

    def transform_image_as_kitti_dataset(self):
        '''
        Make the image capture in gta the same dimensions as the images of the kitii dataset
        '''
        # load original image view
        self.gta_img = cv2.imread(os.path.join(self.directory_path, self.fv_img_fn), cv2.IMREAD_UNCHANGED)

        h_gta, w_gta, c_gta = self.gta_img.shape

        self.kitti_cam_image = self.image_resize(self.gta_img, width = 1392)
        h_kitti, w_kitti, c_kitti = self.kitti_cam_image.shape
        print(self.kitti_cam_image.shape)

        # cut height to 512 pixels (obtained the region of interest (roi)); maintain the same width
        roi_desired_middle_height = 512
        start_row = int((h_kitti-roi_desired_middle_height)/2)
        roi_image = self.kitti_cam_image[start_row:start_row+roi_desired_middle_height, 0:w_kitti-1]
        h_roi, w_roi, c_roi = roi_image.shape
        
        # kitti size: 1224x370
        desired_rect_middle_width = 1224
        desired_rect_middle_height = 370

        start_rect_row = int((h_roi-desired_rect_middle_height)/2)
        start_rect_column = int((w_roi-desired_rect_middle_width)/2)

        self.kitti_image = roi_image[start_rect_row:start_rect_row+desired_rect_middle_height, start_rect_column:start_rect_column+desired_rect_middle_width]

        calculate_shrink_percente(self)


    def calculate_shrink_percente(self)
        '''
        Calculate how much the original (GTA) image shrinks to become the size of the kitti camera resolution
        ''' 
        original_height, original_width, original_channels = self.gta_img.shape
        kitti_height, kitti_width, kitti_channels = self.kitti_cam_image.shape

        self.resize_percentage = kci_w/original_width # 0.725


    def calculate_new_2d_bounding_boxes_for_kitti_img(self, dict_2d_bounding_boxes, discard_trucated_boxes = True):
        '''
        Calculate 2d bounding boxes for the resized and cropped image views that match kitti's images.

        TODO: Support for discard_truncated_boxes = false
        TODO: discard the list of points belonging to objects where their 2d bounding boxes are out of bounds
        '''
        original_height, original_width, original_channels = self.gta_img.shape
        kitti_height, kitti_width, kitti_channels = self.kitti_image.shape

        '''
        lr_bar_width
        <-->
         _____________________________
        |      . p2                   |
        |    _____________________    |
        |   |   . p1              | . |
        |   |_____________________| p3|
        |                             |  | ub_bar_width
        |_____________________________|  |
        '''
        print("original_width: " + str(original_width))
        print("kitti_width: " + str(kitti_width))
        kci_h, kci_w, kci_c = self.kitti_cam_image.shape

        lr_bar = int((kci_w-kitti_width) / 2)
        ub_bar = int((kci_h-kitti_height) / 2)
        print("lr_bar_width: " + str(lr_bar))
        print("ub_bar_height: " + str(ub_bar))

        resize_percentage = kci_w/original_width # 0.725

        dict_kitti_2d_bounding_boxes = {}
        for i in dict_2d_bounding_boxes.keys():
            print("before: minX=" + str(dict_2d_bounding_boxes[i][0]) + ", maxX=" + str(dict_2d_bounding_boxes[i][1]) + ", minY=" + str(dict_2d_bounding_boxes[i][2]) + ", maxY=" + str(dict_2d_bounding_boxes[i][3]))
            # minX
            minX = math.ceil(dict_2d_bounding_boxes[i][0]*resize_percentage) - lr_bar
            # maxX
            maxX = math.ceil(dict_2d_bounding_boxes[i][1]*resize_percentage) - lr_bar
            # minY
            minY = math.ceil(dict_2d_bounding_boxes[i][2]*resize_percentage) - ub_bar
            # maxY
            maxY = math.ceil(dict_2d_bounding_boxes[i][3]*resize_percentage) - ub_bar
            print("after: minX=" + str(minX) + ", maxX=" + str(maxX) + ", minY=" + str(minY) + ", maxY=" + str(maxY))
            # check if out of bound (not inside the kitti dimensions)
            #if  (minX < lr_bar_width or minX >= lr_bar_width+kitti_width) or (maxX < lr_bar_width or maxX >= lr_bar_width+kitti_width) or (minY < ub_bar_height or minY >= ub_bar_height+kitti_height) or (maxY < ub_bar_height or maxY >= ub_bar_height+kitti_height):
            #    if discard_trucated_boxes: # discard/ignore bounding box because it's out of bounds
            #        continue
            dict_kitti_2d_bounding_boxes[i] = []
            dict_kitti_2d_bounding_boxes[i].append(minX)
            dict_kitti_2d_bounding_boxes[i].append(maxX)
            dict_kitti_2d_bounding_boxes[i].append(minY)
            dict_kitti_2d_bounding_boxes[i].append(maxY)

        return dict_kitti_2d_bounding_boxes


    def show_kitti_image(self, window_title = "Image view with the kitti resolution", window_size = 0.5):
        '''
        Open window showing the transformed GTA image view into the resolution used in the kitti dataset
        '''
        resized_image = cv2.resize(self.kitti_image, (0,0), fx=window_size, fy=window_size) 

        cv2.imshow(window_title + ", " + str(int(window_size*100)) + "% zoom", resized_image)
        print("Press any key to continue...")
        cv2.waitKey(0)
        

    def save_kitti_image(self, filename = None):
        '''
        Save kitti image.
        '''
        if filename is None:
            filename = self.fv_img_fn.split(".")[0] + "_kitti.bmp"
            
        cv2.imwrite(os.path.join(self.directory_path, filename), self.kitti_image)




sample = GTASample('./Sample1/', -133.792633)

#sample.save_ply_file("test_rotated.ply", sample.rotated_pc)

#sample.save_ply_file("test_trimmed.ply", sample.fv_pc)

#sample.save_ply_file("test_vehicles.ply", sample.vehicles_pc)

#sample.save_ply_file_from_dict("test_vehicles_colored.ply", sample.dict_with_vehicle_colored_points, attributes = 'c')

# open window with image + bounding boxes
sample.show_image_view_with_2d_bounding_boxes(sample.dict_vehicles_2d_bb, sample.gta_img, sample.vehicle_ids, color = (0, 0, 255), window_title = "Vehicles bounding boxes in gta image", window_size=0.6)

sample.show_image_view_with_2d_bounding_boxes(sample.dict_kitti_vehicles_2d_bb, sample.kitti_image, sample.vehicle_ids, color = (0, 0, 255), window_title = "Vehicles bounding boxes in kitti image", window_size=0.6)

#sample.show_image_view_with_2d_bounding_boxes(sample.dict_pedestrians_2d_bb, sample.fv_img_fn, sample.pedestrian_ids, window_title = "Pedestrians bounding boxes")

# show image without bounding boxes
#sample.show_kitti_image()

sample.save_kitti_image()


'''
        shrink_percent_width = kitti_width / original_width
        shrink_percent_height = kitti_height / original_height

        dict_kitti_2d_bounding_boxes = {}
        for i in dict_2d_bounding_boxes.keys():
            dict_kitti_2d_bounding_boxes[i] = []
            dict_kitti_2d_bounding_boxes[i].append(int(dict_2d_bounding_boxes[i][0] * shrink_percent_width))
            dict_kitti_2d_bounding_boxes[i].append(int(dict_2d_bounding_boxes[i][1] * shrink_percent_width))
            dict_kitti_2d_bounding_boxes[i].append(int(dict_2d_bounding_boxes[i][2] * shrink_percent_height))
            dict_kitti_2d_bounding_boxes[i].append(int(dict_2d_bounding_boxes[i][3] * shrink_percent_height))
        
        print(dict_kitti_2d_bounding_boxes)
        '''












