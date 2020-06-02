import math
import json
import numpy as np
import random
import cv2
import os.path
from PcLabeledObject import PcLabeledObject

class PcRaw:
    '''
    Contains raw information about a point cloud.
    It can be an entire point cloud, or a point cloud with points associated to a given label (vehicles, pedestrians, ...). 
    '''
    pc_name = ""
    # Raw point cloud containing the points of all the object in the category. It is a list of tuples representing the position (x, y, z) of each the points
    list_raw_pc = []
    # roation in radians to aligne point cloud with the direction that the character is facing
    rotation_amount = 0
    # Rotate raw point cloud, list of (x, y, z) tuples
    list_rotated_raw_pc = []
    # List that associates each point of list_raw_pc to the correspondent labels. Each label is an integer
    list_raw_labels = []
    # List that associates each point of list_raw_pc to the correspondent gameobject id. Each id is represented by an integer.
    list_raw_detailed_labels = []
    # List of tuples (projx, projy, viewID) storing the projection of each point of the point cloud onto the front view image
    list_raw_projected_points = []
    # List of all the different labels within the point cloud
    list_labels = []

    # dict of PCLabeledObject's, where each key is a label/category integer 
    single_category_pcs_list = {}

    def __init__(self, list_raw_pc, list_raw_labels, list_raw_detailed_labels, list_raw_projected_points, camRot = 0, debugMode = False, pcName = ""):
        self.pc_name = pcName
        self.rotation_amount = self.degreesToRad(camRot)  # rotation around z axis, in radians
        self.list_labels = self.getListLabelsWithinPc(list_raw_labels)

        self.list_raw_pc = list_raw_pc
        self.list_raw_labels = list_raw_labels
        self.list_raw_detailed_labels = list_raw_detailed_labels
        self.list_raw_projected_points = list_raw_projected_points

        self.list_rotated_raw_pc = self.rotatePcToAlignWithRectCamCoordSystem(self.list_raw_pc, self.rotation_amount)
        
        self.debug(debugMode)

    def rotatePcToAlignWithRectCamCoordSystem(self, point_list, rotation_rad):
        '''
        Rotates the entire point cloud to align with the rectified camera coordinate system
        Arguments:
            - tuple list with all the point cloud points.
            - angle_rad: rotation in radians
        Returns:
            - tuple list with the rotated point cloud points
        '''
        rot_point_list = []
        for i in range(0, len(point_list)):
            rot_point_tuple = self.rotatePointAroundZaxis(point_list[i], rotation_rad)
            #rot_point_tuple = self.rotatePointAroundYaxis(rot_point_tuple, self.degreesToRad(-90))
            #rot_point_tuple = self.scalePoint(rot_point_tuple, 1, -1, 1)
            rot_point_list.append(rot_point_tuple)

        return rot_point_list
    
    def rotatePointAroundZaxis(self, point, angle_rad):
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

    def rotatePointAroundXaxis(self, point, angle_rad):
        '''
        Rotate a point around the x axis.
        Arguments:
            - point: tuple with 3 elements (x, y, z) corresponding the point to be rotated
            - angle_rad: angle to rotate the point around the x axis
        Returns:
            - tuple with the rotated point coordinates (x, y, z)
        '''
        p_x, p_y, p_z = point

        r_x = (p_x)
        r_y = (p_y)*math.cos(angle_rad) - (p_z)*math.sin(angle_rad)
        r_z = (p_y)*math.sin(angle_rad) + (p_z)*math.cos(angle_rad)

        return (r_x, r_y, r_z)

    def rotatePointAroundYaxis(self, point, angle_rad):
        '''
        Rotate a point around the y axis.
        Arguments:
            - point: tuple with 3 elements (x, y, z) corresponding the point to be rotated
            - angle_rad: angle to rotate the point around the y axis
        Returns:
            - tuple with the rotated point coordinates (x, y, z)
        '''
        p_x, p_y, p_z = point

        r_x = (p_x)*math.cos(angle_rad) + (p_z)*math.sin(angle_rad)
        r_y = (p_y)
        r_z = -(p_x)*math.sin(angle_rad) + (p_z)*math.cos(angle_rad)

        return (r_x, r_y, r_z)

    def scalePoint(self, point, vx, vy, vz):
        p_x, p_y, p_z = point
        
        s_x = vx*p_x
        s_y = vy*p_y
        s_z = vz*p_z

        return (s_x, s_y, s_z)


    def degreesToRad(self, angle_degrees):
        '''
        Converts degrees into radians.
        Returns:
            - Float angle in radian
        '''
        return angle_degrees * (math.pi/180)

    def getListLabelsWithinPc(self, point_cloud_labels):
        '''
        Search for all different labels within a point cloud.
        '''
        list = []
        for i in range(0, len(point_cloud_labels)):
            if point_cloud_labels[i] not in list:
                list.append(point_cloud_labels[i])
        return list

    def debug(self, debug_mode):
        if debug_mode:
            print("\n==== PointCloud: " + self.pc_name + " ====")
            print("Rotation amount: " + str(self.rotation_amount) + " rad")
            print("list_raw_pc:\t\t\t " + str(len(self.list_raw_pc)) + " elements;\t element: " + str(type(self.list_raw_pc[0])) + ";\t element length: " + str(len(self.list_raw_pc[0])) + "; \tinfo: (x, y, z)")
            print("list_raw_labels:\t\t " + str(len(self.list_raw_labels)) + " elements;\t element: " + str(type(self.list_raw_labels[0])))
            print("list_raw_detailed_labels:\t " + str(len(self.list_raw_detailed_labels)) + " elements;\t element: " + str(type(self.list_raw_detailed_labels[0])))
            print("list_raw_projected_points:\t " + str(len(self.list_raw_projected_points)) + " elements;\t element: " + str(type(self.list_raw_projected_points[0])) + ";\t element length: " + str(len(self.list_raw_projected_points[0])) + "; \tinfo: (projX, projY, viewID)")
            print("list_rotated_raw_pc:\t\t " + str(len(self.list_rotated_raw_pc)) + " elements;\t element: " + str(type(self.list_rotated_raw_pc[0])) + ";\t element length: " + str(len(self.list_rotated_raw_pc[0])) + "; \tinfo: (x, y, z)")
            print("list_labels:\t\t\t " + str(len(self.list_labels)) + " elements;\t\t element: " + str(type(self.list_labels[0])) + "\t\t printed list: " +  str(self.list_labels))

    def generateSingleCategoryPointCloud(self, category_id, category_name = "", debug_mode = False):
        '''
        Create a point cloud with points belonging to the same label/category.
        '''
        category_exists = False
        for i in range(0, len(self.list_labels)):
            print("i: " + str(self.list_labels[i]))
            if self.list_labels[i] == category_id:
                category_exists = True

        if not category_exists:
            print("ERROR: label " + str(category_id)  + " does not exist!")
            return None

        # get all points with the given label/category
        points = []
        detailed_labels_list = []
        projected_point_list = []
        for i in range(0, len(self.list_rotated_raw_pc)):
            if self.list_raw_labels[i] == category_id:
                points.append(self.list_rotated_raw_pc[i])
                detailed_labels_list.append(self.list_raw_detailed_labels[i])
                projected_point_list.append(self.list_raw_projected_points[i])

        category_pc = PcLabeledObject(points, detailed_labels_list, projected_point_list, debug_mode, category_id, category_name)

        self.single_category_pcs_list[category_id] = category_pc

    