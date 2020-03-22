import math
import json
import numpy as np
import random
import cv2
import os.path

class PCLabeledObject():
    '''
    This object represents a data of a point cloud containing only one type object (pedestrians, vehicles, ...).
    It can represent vehicles, pedestrians, and so on...
    '''

    # category/label id and name
    category_id = -1
    category_name = ""

    #### Each key of the dictionaries is an id present in the categories_ids list   ####

    # list of all the different gameobject ids (detailed labels) within the point cloud
    object_ids_list = []

    # color for each detailed label
    color_per_detailed_label_dict = {}
    
    # each value is a list of positions [(x, y, z), ...] of the points belonging to a gameobject
    dict_of_positions_per_obj = {}
    # each value is a list of colors [(r, g, b), ...] of the points belonging to a gameobject
    dict_of_colors_per_obj = {}
    # each value is a list of projected points [(projx, projy), ...] of the points belonging to a gameobject
    dict_projected_coords_per_obj = {}

    # each value is a list [minX, maxX, minY, maxY] of the 2D bounding box belonging to a gameobject, in the original image resolution (taken from gta)
    dict_original_view_2d_bb = {}
    # each value is a list [minX, maxX, minY, maxY] of the 2D bounding box belonging to a gameobject, in the kitti image resolution (after the original image being resized and cut)
    dict_kitti_view_2d_bb = {}

    def __init__(self, list_raw_pc, list_raw_detailed_labels, list_raw_projected_points, category_id, category_name = "", debug_mode = False):
        '''
        The argument's point cloud data only correspond to a single label/category
        '''
        self.category_id = category_id
        self.category_name = category_name

        self.object_ids_list = self.get_individual_object_ids(list_raw_detailed_labels)

        self.dict_of_positions_per_obj, self.dict_of_colors_per_obj, self.dict_projected_coords_per_obj, self.color_per_detailed_label_dict = self.create_dicts_to_separate_individual_objects(list_raw_pc, list_raw_detailed_labels, list_raw_projected_points, self.object_ids_list)

    def generate_random_colors_for_objects(self, object_ids_list):
        # create random colors for the different object ids
        print(object_ids_list)
        color_per_object_id_dict = {}     # dicionario onde cada key é um gameobject id, e onde cada valor é uma string a indicar a cor rgb. 
        for i in range(0, len(object_ids_list)):
            # generate random rgb color for the id of the current iteration
            r = random.randint(0, 256)   # random value between [0, 255]
            g = random.randint(0, 256)
            b = random.randint(0, 256)
            color_per_object_id_dict[object_ids_list[i]] = (r, g, b) 
        
        return color_per_object_id_dict

    def create_dicts_to_separate_individual_objects(self, list_raw_pc, list_raw_detailed_labels, list_raw_projected_points, object_ids_list):
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

        color_per_object_dict = self.generate_random_colors_for_objects(object_ids_list)

        # dictionary
        dict_of_positions_per_obj = {}
        dict_of_colors_per_obj = {}
        dict_projected_coords_per_obj = {}

        for i in range(0, len(list_raw_pc)):
            if list_raw_detailed_labels[i] not in dict_of_positions_per_obj.keys():
                dict_of_positions_per_obj[list_raw_detailed_labels[i]] = []
                dict_of_colors_per_obj[list_raw_detailed_labels[i]] = []
                dict_projected_coords_per_obj[list_raw_detailed_labels[i]] = []

            dict_of_positions_per_obj[list_raw_detailed_labels[i]].append((list_raw_pc[i][0], list_raw_pc[i][1], list_raw_pc[i][2]))

            
            dict_of_colors_per_obj[list_raw_detailed_labels[i]].append((color_per_object_dict[list_raw_detailed_labels[i]][0],
                                                                    color_per_object_dict[list_raw_detailed_labels[i]][1], 
                                                                    color_per_object_dict[list_raw_detailed_labels[i]][2]))
                        
            dict_projected_coords_per_obj[list_raw_detailed_labels[i]].append((list_raw_projected_points[i][0], list_raw_projected_points[i][1]))

        return dict_of_positions_per_obj, dict_of_colors_per_obj, dict_projected_coords_per_obj, color_per_object_dict

    def get_individual_object_ids(self, labels_detailed_list):
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

    def get_uncolored_point_cloud_dict(self):
        return self.dict_of_positions_per_obj

    def get_colored_point_cloud_dict_by_detailed_labels(self):
        '''
        Returns a list of tuples (x, y, z, r, g, b)
        '''
        new_dict = {} # will contain, per detailed label, a list of tuples (x, y, z, r, g, b)
        
        for i in range(0, len(self.object_ids_list)):
            new_dict[self.object_ids_list[i]] = []

            for j in range(0, len(self.dict_of_positions_per_obj[self.object_ids_list[i]])):
                new_dict[self.object_ids_list[i]].append((self.dict_of_positions_per_obj[self.object_ids_list[i]][j][0],
                                                    self.dict_of_positions_per_obj[self.object_ids_list[i]][j][1],
                                                    self.dict_of_positions_per_obj[self.object_ids_list[i]][j][2],
                                                    self.dict_of_colors_per_obj[self.object_ids_list[i]][j][0],
                                                    self.dict_of_colors_per_obj[self.object_ids_list[i]][j][1],
                                                    self.dict_of_colors_per_obj[self.object_ids_list[i]][j][2]))

        return new_dict
                
            





