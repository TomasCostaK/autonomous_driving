from GtaSample import GtaSample
from pathlib import Path
import numpy as np
import os.path
import math
import struct

class KittiSample:

    dict_2d_bb_NEW = {}

    def __init__(self, gtaSample, outputRootDir, outputLabelsDir, outputVelDir, outputViewsDir, outputCalDir, sampleCounter):
        self.kittiOutputSamplesDir = outputRootDir
        self.kittiLabelsDir = outputRootDir + outputLabelsDir
        self.kittiVelodyneDir = outputRootDir + outputVelDir
        self.kittiViewsDir = outputRootDir + outputViewsDir
        self.kittiCalibDir = outputRootDir + outputCalDir

        self.gtaSample = gtaSample

        self.outputKittiLabelFile(sampleCounter, ignore_truncated_bbs = True)

    @staticmethod
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

    @staticmethod
    def loadGtaVelodyneBinFile(file_path, include_luminance = False):
        '''
        Loads a kitti velodyne file (ex: 000000.bin) into a list of tuples, where each tuple has (x, y, z) or (x, y, z, l)
        Argument:
            - include_luminance: if the function should also store the pont intensisty value in the list of points
        '''
        f = open(file_path, mode='rb')
        dt=np.dtype('float,float,float')
        numpyList = np.fromfile(f, dt)

        point_tuple_list = []
        for i in range(len(numpyList)):
            point_tuple_list.append((numpyList[i][0], numpyList[i][1], numpyList[i][2],))

        return point_tuple_list

    @staticmethod
    def saveKittiVelodyneFile(tuple_list, filename, directory, output_luminance = False):
        '''
        Saves pointcloud without luminance in binary
        For frustum pointnet, the point cloud must contain luminance
        '''
        #dt = None
        #if output_luminance:
            #dt=np.dtype('float,float,float,float')
        #else:
            #dt=np.dtype('float,float,float')

        '''for point in tuple_list:
            s = struct.pack('f'*len(point), *point)
            f = open(directory + filename, 'ab')    # append and binary
            f.write(s)
            f.close()'''

        with open(directory + filename, "wb") as f:
            for point in tuple_list:
                s = struct.pack('f'*len(point), *point)
                f.write(s)
                #newline = "\n"
                #f.write(newline.encode("utf-8").replace(newline, os.linesep))
        

    def addDummyLuminenceValuesToPointCloud(self, tuple_list):
        new_tuple_list = []
        for t in tuple_list:
            new_tuple_list.append((t[0], t[1], t[2], 1))

        return new_tuple_list

    def outputKittiLabelFile(self, sampleCounter, ignore_truncated_bbs = True):
        # generate file name
        n_digits = len(str(sampleCounter))
        output_file_name = ""               # ex: 000000.txt
        for i in range(0, 6-n_digits):
            output_file_name = output_file_name + "0"
        
        if len(output_file_name) is not 6:
            output_file_name = output_file_name + str(sampleCounter)

        # create the hierarchy of directories
        Path(self.kittiOutputSamplesDir).mkdir(parents=True, exist_ok=True)
        Path(self.kittiViewsDir).mkdir(parents=True, exist_ok=True)
        Path(self.kittiVelodyneDir).mkdir(parents=True, exist_ok=True)
        Path(self.kittiLabelsDir).mkdir(parents=True, exist_ok=True)
        Path(self.kittiCalibDir).mkdir(parents=True, exist_ok=True)

        # save image
        self.gtaSample.imageView.saveImage(self.gtaSample.imageView.kittiImage, self.kittiViewsDir, output_file_name + ".png")
        # save point cloud - the full rotated point cloud
        KittiSample.saveKittiVelodyneFile(self.addDummyLuminenceValuesToPointCloud(self.gtaSample.pcData.list_rotated_raw_pc), output_file_name + ".bin", self.kittiVelodyneDir, output_luminance = True)
        # save calibration info
        self.saveCalibInfo(self.kittiCalibDir, output_file_name + ".txt")
        # labels info
        self.saveLabelInfo(self.kittiLabelsDir, output_file_name + ".txt")

    def degreesToRad(self, angle_degrees):
        '''
        Converts degrees into radians.
        Returns:
            - Float angle in radian
        '''
        return angle_degrees * (math.pi/180)

    def saveCalibInfo(self, dirname, filename):
        img_width = 1224
        img_height = 370

        Cu = img_width/2 # half screen width
        Cv = img_height/2  # half screen height
        hor_fov = 50.0  
        vert_fov = 2*math.atan(math.tan(hor_fov/2)*(img_width/img_height))

        fx = img_width / (2.0 * math.tan(hor_fov * math.pi / 360.0))  # focus length

        fy = img_height / (2.0 * math.tan(vert_fov * math.pi / 360.0))  # focus length

        #f = 1224 / math.tan((hor_fov * math.pi / 180.0) / 2)  # focus length
        
        #print("2.0 * math.tan(hor_fov * math.pi / 360.0: " + str(2.0 * math.tan(hor_fov * math.pi / 360.0)))
        print("hor_fov: " + str(hor_fov))
        print("pi: " + str(math.pi))
        print("Cu: " + str(Cu))
        print("Cv: " + str(Cv))
        #print("f: " + str(f))

        # only one camera means p1=p2=p3=p0 === GOOD ===
        #p0_mat = [[fx, 0, Cu, 0],
        #          [0, fy, Cv, 0],
        #          [0, 0, 1, 0]]

        p0_mat = [[850, 0, Cu, 0],
                  [ 0, 900, Cv, 0],
                  [ 0, 0, 1, 0]]

        p1_mat=p2_mat=p3_mat=p0_mat

        # rot x 90                          <<<<<<<<<<<<<<<<<<<<<<<<
        #r0_rect = [[1, 0, 0],
        #           [0, 0, -1],
        #           [0, 1, 0]]

        # rot z 90                                      <<<<<<<<<<<<<<<<<<<<<<<<
        #tr_velo_to_cam = [[0, -1, 0, 0],
        #                  [1, 0, 0, 0],
        #                  [0, 0, 1, 0]]    
        
        # identity
        r0_rect = [[1, 0, 0],
                  [0, 1, 0],
                   [0, 0, 1]]

        # rotZ(90) * rotX(90)
        tr_velo_to_cam = [[0, -1, 0, 0],
                          [0, 0, -1, 0],
                          [1, 0, 0, 0]]

        tr_imu_to_velo = [[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0]]

        line = KittiSample.matToStringKitti("P0", p0_mat)
        line += KittiSample.matToStringKitti("P1", p1_mat)
        line += KittiSample.matToStringKitti("P2", p2_mat)
        line += KittiSample.matToStringKitti("P3", p3_mat)
        line += KittiSample.matToStringKitti("R0_rect", r0_rect)
        line += KittiSample.matToStringKitti("Tr_velo_to_cam", tr_velo_to_cam)
        line += KittiSample.matToStringKitti("Tr_imu_to_velo", tr_imu_to_velo)

        KittiSample.saveStrInTxtFile(dirname, filename, line)
        
    @staticmethod
    def matToStringKitti(name, mat):
        '''
            Arguments:
                - name: name of the matrix
                - mat: list of lists of int or float values
            Returns:
                - string
        '''
        line = name + ": "
        for i in range(0, len(mat)):
            for j in range(0, len(mat[i])):
                line += str(mat[i][j]) + " "

        line += "\n"
        
        return line

    @staticmethod
    def saveStrInTxtFile(dirPath, filename, line):
        with open(os.path.join(dirPath, filename), "w") as the_file:
            the_file.write(line)

    def saveLabelInfo(self, dirname, filename):        
        '''
            (1 value) label_name: 'Car', 'Van', 'Truck', 'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram', 'Misc' (qq outro tipo de vehiculo) or 'DontCare' (quando se pertende ignorar um veiculo)

            (1 value) trucanted: [0, 1], evaluates how much an object  isn't within the image boundaries

            (1 value) occluded: {0, 1, 2, 3}, fully-visible, partly occluded, largely-occuled, unknown

            (1 value) alpha: [-pi, pi], observation angle of the object

            (4 values) bbox: {left, top, right, bottom}, 2d bounding box of the object in the image

            (3 values) dimensions: {height, width, length}, 3d object dimensions in meters

            (3 values) location: {x, y, z}, 3d object location in camera coordinates in meters

            (1 value) rotation_y: [-pi, pi], rotation around Y-axis (up axis) in camera coordinates
        '''

        '''
            Values per vehicle: [Entity] 
            | Hash 
            | minCornerX | minCornerY | minCornerZ | projMinCornerX | projMinCornerX 
            | maxCornerX | maxCornerY | maxCornerZ | projMaxCornerX | projMaxCornerY 
            | Posx | Posy | Posz | Rotx | Roty | Rotz 
            | projCenterX | projCenterY 
            | dimX | dimY | dimZ
            | objectType | truncated"
        '''
        vehicleInfoDict = self.gtaSample.loadTxtFileToDict(self.gtaSample.vehiclesInfoFn)

        contents_list = []

        # for visualization purposes
        boundingBoxList = []

        for key in vehicleInfoDict.keys():
            label_line = ""

            if int(vehicleInfoDict[key][4]) < 0 or int(vehicleInfoDict[key][5]) < 0 or int(vehicleInfoDict[key][9]) < 0 or int(vehicleInfoDict[key][10]) < 0: # if one projection coordinate is negative, the vehicle isn't in the image view
                continue

            #kitti_height, kitti_width, kitti_channels = self.gtaSample.imageView.getKittiImageDimensions()

            # object type: car
            label_line += vehicleInfoDict[key][22] + " "
            
            # truncated
            label_line += "0 "

            # occluded
            label_line += "0 "

            # alpha
            label_line += "0 "
            
            # make sure that the projected points correspond to the min and max
            minx = -1
            miny = -1
            maxx = -1
            maxy = -1
            if int(vehicleInfoDict[key][4]) < int(vehicleInfoDict[key][9]):
                minx = vehicleInfoDict[key][4]
                maxx = vehicleInfoDict[key][9]
            else:
                minx = vehicleInfoDict[key][9]
                maxx = vehicleInfoDict[key][4]
            
            if int(vehicleInfoDict[key][5]) < int(vehicleInfoDict[key][10]):
                miny = vehicleInfoDict[key][5]
                maxy = vehicleInfoDict[key][10]
            else:
                miny = vehicleInfoDict[key][10]
                maxy = vehicleInfoDict[key][5]

            # get projected coordinates for resized (kitti) image
            minx, miny, maxx, maxy = self.gtaSample.imageView.calculate2dBoundingBoxesForKittiImgSize((int(minx), int(miny), int(maxx), int(maxy)))

            boundingBoxList.append((minx, miny, maxx, maxy))

            # minx, miny, maxx, maxy
            label_line += str(minx) + " " + str(miny) + " " + str(maxx) + " " + str(maxy) + " "
            
            # height, width, length
            #label_line += vehicleInfoDict[key][19] + " " + vehicleInfoDict[key][20] + " " + vehicleInfoDict[key][21] + " "
            label_line += vehicleInfoDict[key][21] + " " + vehicleInfoDict[key][20] + " " + vehicleInfoDict[key][19] + " "

            # location
            # rotate location point around z axis according to the angle that the point cloud was rotated (- Z angle of the camera - 90º)
            originalVehiclePoint = (float(vehicleInfoDict[key][11]), float(vehicleInfoDict[key][12]), float(vehicleInfoDict[key][13]) - float(vehicleInfoDict[key][21])/2)

            print("ORIGINAL CENTER: " + str(originalVehiclePoint))

            # because of the point cloud is aditionally transformed to be pointing in the direction of x axis instead of the y axis
            rotatedVehiclePos = self.gtaSample.pcData.rotatePointAroundZaxis(originalVehiclePoint, self.gtaSample.pcData.rotation_amount) #self.degreesToRad(-90))

            print("ORIGINAL CENTER RotZ -90º: " + str(rotatedVehiclePos))

            # transform from lidar coordinate system to camera coordinate system
            rotatedVehiclePos = self.gtaSample.pcData.rotatePointAroundZaxis(rotatedVehiclePos, self.degreesToRad(90))
            rotatedVehiclePos = self.gtaSample.pcData.rotatePointAroundXaxis(rotatedVehiclePos, self.degreesToRad(90))

            label_line += str(rotatedVehiclePos[0]) + " " + str(rotatedVehiclePos[1]) + " " + str(rotatedVehiclePos[2]) + " "
            
            # [-180, 180]
            print("Rotation in degrees: " + vehicleInfoDict[key][16])

            print("Rotation in degrees: " + str(-float(vehicleInfoDict[key][16]))) # + 40

            # [-pi, pi]
            obj_rot_rads = self.degreesToRad(float(vehicleInfoDict[key][16]) * -1)

            #print("Rotation in radians: " + str(self.degreesToRad(float(vehicleInfoDict[key][16]))))

            #obj_rotation = float(vehicleInfoDict[key][16])

            #if obj_rotation < 0:
            #    obj_rotation = 180 + (180 - abs(obj_rotation))
            #    print("Rotation in degrees: " + str(obj_rotation))
                
            obj_rot_rads = obj_rot_rads - self.degreesToRad(90) - self.gtaSample.pcData.rotation_amount

            # keep the angle between [-pi, pi]
            if obj_rot_rads > math.pi:
                obj_rot_rads = abs(math.pi - obj_rot_rads) - math.pi
            if obj_rot_rads < -math.pi:
                obj_rot_rads = math.pi - (abs(obj_rot_rads) - math.pi)

            print("--- Camera rotation: " + str(self.gtaSample.rawCamRotation))
            print("--- Vehicle rotation in velocyne coords: " + vehicleInfoDict[key][15])
            print("--- Vehicle rotation in cam coords: " + str(obj_rot_rads*180/math.pi))

            #print("Rotation in degrees: " + str(self.degreesToRad(obj_rotation + 90) + self.gtaSample.pcData.rotation_amount))
            print("--- Final rotation: " + str(obj_rot_rads))
            # rotation_y in radians
            #label_line += str(self.degreesToRad(float(vehicleInfoDict[key][16]))) + " "# + 90) + self.gtaSample.pcData.rotation_amount) + " "
            label_line += str(obj_rot_rads) + " "

            contents_list.append(label_line)

        # show resulting bounding boxes in kitti images
        #self.gtaSample.imageView.showViewWith2dBoundingBoxes(boundingBoxList, self.gtaSample.imageView.kittiImage, color = (0, 0, 255), window_title = "Bounding box results", window_size = 0.5)

        self.gtaSample.saveListIntoTxtFile(contents_list, dirname, filename)

    def is_bb_truncated(self, list_coords):
        '''
            Checks if the 2d bounding box of the object was cut when the image view was resized to the kitti resolution
        '''
        kitti_height, kitti_width, kitti_channels = self.imageView.get_kitti_image_dimensions()
        #print("height: " + str(kitti_height))
        #print("width: " + str(kitti_width))
        #print("list: " + str(list_coords))

        if list_coords[0] < 0 or list_coords[1] > kitti_width or list_coords[2] < 0 or list_coords[3] > kitti_height:
            return True 

        return False

    def testProjection(self):
        '''
            Calculate bounding box with projections taken from gtav (not working as desired)
        '''
        
        dict_vehicles_dim = self.gtaSample.loadTxtFileToDict(self.gtaSample.vehiclesInfoFn)

        print(dict_vehicles_dim)
        dict_vehicle_projected_center = {}
        for key in self.gtaSample.imageView.dict_2d_bb_of_kitti_image.keys():
            self.gtaSample.dict_2d_bb_NEW[key] = []
            dict_vehicle_projected_center[key] = []
            # ignore bounding boxes that have coordinates out of bounds
            if int(float(dict_vehicles_dim[str(key)][9])) < 0 \
                or int(float(dict_vehicles_dim[str(key)][4])) < 0 \
                or int(float(dict_vehicles_dim[str(key)][10])) < 0 \
                or int(float(dict_vehicles_dim[str(key)][5])) < 0:
                #or int(float(dict_vehicles_dim[str(key)][9])) > 1392 \
                #or int(float(dict_vehicles_dim[str(key)][4])) > 1392 \
                #or int(float(dict_vehicles_dim[str(key)][10])) > 783 \
                #or int(float(dict_vehicles_dim[str(key)][5])) > 783:

                self.gtaSample.dict_2d_bb_NEW[key].append(-1)
                self.gtaSample.dict_2d_bb_NEW[key].append(-1)
                self.gtaSample.dict_2d_bb_NEW[key].append(-1)
                self.gtaSample.dict_2d_bb_NEW[key].append(-1)
                dict_vehicle_projected_center[key].append(-1)
                dict_vehicle_projected_center[key].append(-1)
                continue

            self.gtaSample.dict_2d_bb_NEW[key].append(int(float(dict_vehicles_dim[str(key)][4])))
            self.gtaSample.dict_2d_bb_NEW[key].append(int(float(dict_vehicles_dim[str(key)][9])))
            self.gtaSample.dict_2d_bb_NEW[key].append(int(float(dict_vehicles_dim[str(key)][10])))
            self.gtaSample.dict_2d_bb_NEW[key].append(int(float(dict_vehicles_dim[str(key)][5])))
            print("HELLO: " + dict_vehicles_dim[str(key)][17] + " " + dict_vehicles_dim[str(key)][18])
            dict_vehicle_projected_center[key].append(int(float(dict_vehicles_dim[str(key)][17])))
            dict_vehicle_projected_center[key].append(int(float(dict_vehicles_dim[str(key)][18])))

            print("New projection: " + str(self.dict_2d_bb_NEW[key][0]) + " " + str(self.dict_2d_bb_NEW[key][1]) + " " + str(self.dict_2d_bb_NEW[key][2]) + " " + str(self.dict_2d_bb_NEW[key][3]))
        
        #print(self.dict_2d_bb_NEW)
        print("Dictionary projection points: ")
        print(self.gtaSample.dict_2d_bb_NEW)
        
        self.gtaSample.imageView.showViewWith2dBoundingBoxes(self.gtaSample.dict_2d_bb_NEW, self.gtaSample.imageView.gtaImage, self.gtaSample.imageView.dict_2d_bb_of_kitti_image.keys(), window_size = 0.7, object_centers = dict_vehicle_projected_center)
        pass












