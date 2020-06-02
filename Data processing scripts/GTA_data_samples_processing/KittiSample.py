from GtaSample import GtaSample
from pathlib import Path
import numpy as np
import os.path
import math
import struct
from kitti_util import compute_box_3d

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

    # Ref: https://github.com/charlesq34/frustum-pointnets/blob/master/kitti/kitti_util.py
    def inverse_rigid_trans(self, Tr):
        ''' Inverse a rigid body transform matrix (3x4 as [R|t])
            [R'|-R't; 0|1]
        '''
        inv_Tr = np.zeros_like(Tr) # 3x4
        inv_Tr[0:3,0:3] = np.transpose(Tr[0:3,0:3])
        inv_Tr[0:3,3] = np.dot(-np.transpose(Tr[0:3,0:3]), Tr[0:3,3])
        return inv_Tr

    def saveCalibInfo(self, dirname, filename):
        # https://towardsdatascience.com/inverse-projection-transformation-c866ccedef1c
        # https://github.com/darylclimb/cvml_project/blob/master/projections/inverse_projection/geometry_utils.py
        img_width = 1224
        img_height = 370

        fov = 75
        Cu = img_width/2 # half screen width
        Cv = img_height/2  # half screen height

        hor_fov = fov / 360. * 2. * np.pi       # 50.0  
        fx = img_width / (2. * np.tan(hor_fov / 2.))# img_width / (2.0 * math.tan(hor_fov * math.pi / 360.0))  # focus length

        vert_fov = 2. * np.arctan(np.tan(hor_fov / 2) * img_height / img_width)
        fy = img_height / (2. * np.tan(vert_fov / 2.))
        
        print("hor_fov: " + str(hor_fov))
        print("pi: " + str(math.pi))
        print("Cu: " + str(Cu))
        print("Cv: " + str(Cv))

        self.p0_mat =[[fx, 0, Cu, 0.],
                      [0, fy, Cv, 0.],
                      [0, 0, 1., 0.]]
        
        # store in a 3x4 np matrix
        self.p0_mat = np.array(self.p0_mat)
        self.p0_mat = np.reshape(self.p0_mat, [3,4])

        p1_mat=p2_mat=p3_mat=self.p0_mat

        # identity
        r0_rect = [[1, 0, 0],
                  [0, 1, 0],
                   [0, 0, 1]]

        # Rotation from reference camera coord to rect camera coord
        self.R0 = np.array(r0_rect)
        self.R0 = np.reshape(self.R0,[3,3])

        tr_velo_to_cam = [[0, -1, 0, 0],
                          [0, 0, -1, 0],
                          [1, 0, 0, 0]]

        self.V2C = np.array(tr_velo_to_cam)
        self.V2C = np.reshape(self.V2C, [3,4])
        self.C2V = self.inverse_rigid_trans(self.V2C)

        tr_imu_to_velo = [[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0]]

        line = KittiSample.matToStringKitti("P0", self.p0_mat)
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

        print("Number of keys: " + str(vehicleInfoDict.keys()))

        for key in vehicleInfoDict.keys():
            label_line = ""

            kitti_height, kitti_width, kitti_channels = self.gtaSample.imageView.getKittiImageDimensions()

            #### Calculate object location (the center of its base plane ###)
            # rotate location point around z axis according to the angle that the point cloud was rotated (- Z angle of the camera - 90º)
            originalVehiclePoint = (float(vehicleInfoDict[key][11]), float(vehicleInfoDict[key][12]), float(vehicleInfoDict[key][13]) - float(vehicleInfoDict[key][21])/2)

            # because of the point cloud is aditionally transformed to be pointing in the direction of x axis instead of the y axis
            rotatedVehiclePos = self.gtaSample.pcData.rotatePointAroundZaxis(originalVehiclePoint, self.gtaSample.pcData.rotation_amount)

            # transform from lidar coordinate system to camera coordinate system
            rotatedVehiclePos = self.gtaSample.pcData.rotatePointAroundZaxis(rotatedVehiclePos, self.degreesToRad(90))
            rotatedVehiclePos = self.gtaSample.pcData.rotatePointAroundXaxis(rotatedVehiclePos, self.degreesToRad(90))

            # check if the vehicle is in front of the camera if not, ignore it
            objectPosition = np.array([originalVehiclePoint[0], originalVehiclePoint[1], originalVehiclePoint[2]])
            print("Object position: " + str(objectPosition))

            camPosition = np.zeros(3)
            
            # car to cam instead of cam to car because the gta is in left handed coords and the projection matrix is in right handed coords
            vecCamToObj = objectPosition - camPosition
            vecCamToObj.dot(vecCamToObj)
            # normalize vector       
            mag = np.sqrt(math.pow(vecCamToObj[0], 2) + math.pow(vecCamToObj[1], 2) + math.pow(vecCamToObj[2], 2))
            vecCamToObj = np.array([vecCamToObj[0]/mag, vecCamToObj[1]/mag, vecCamToObj[2]/mag])

            # # dot product
            dotCamObj = self.gtaSample.camForwardDir.dot(vecCamToObj)
            
            angle = math.acos(dotCamObj) * 180 / math.pi

            if angle < 0 or angle > 90:
                continue

            #### 3D bounding box dimensions ####
            # height (dimz), width (dimx), length (dimy)
            bb3d_height = float(vehicleInfoDict[key][21])
            bb3d_width = float(vehicleInfoDict[key][19])
            bb3d_length = float(vehicleInfoDict[key][20])

            #### Calculate 3D and 2D bounding boxes through the object's rotation and forward vector, and camera rotation 
            obj_rot_rads = 0
            if float(vehicleInfoDict[key][24]) < 0 and float(vehicleInfoDict[key][25]) < 0: #DONE
                print("1")
                obj_rot_rads = float(vehicleInfoDict[key][16]) + self.gtaSample.rawCamRotation + 90
            elif float(vehicleInfoDict[key][24]) < 0 and float(vehicleInfoDict[key][25]) > 0: #DONE
                print("4")
                obj_rot_rads = -float(vehicleInfoDict[key][16]) + self.gtaSample.rawCamRotation - 90
            elif float(vehicleInfoDict[key][24]) > 0 and float(vehicleInfoDict[key][25]) > 0: # almost
                #if (float(vehicleInfoDict[key][16]) > -45):
                if float(vehicleInfoDict[key][26]) < 0:
                    print("5")
                    #obj_rot_rads = float(vehicleInfoDict[key][16]) + self.gtaSample.rawCamRotation
                    obj_rot_rads = -float(vehicleInfoDict[key][16]) + self.gtaSample.rawCamRotation - 90
                else:
                    print("6")
                    obj_rot_rads = -float(vehicleInfoDict[key][16]) + self.gtaSample.rawCamRotation - 90
            elif float(vehicleInfoDict[key][24]) > 0 and float(vehicleInfoDict[key][25]) < 0:
                #if float(vehicleInfoDict[key][16]) >= -45:
                if float(vehicleInfoDict[key][26]) < 0:
                    print("7")
                    obj_rot_rads = float(vehicleInfoDict[key][16]) + self.gtaSample.rawCamRotation - 90 + 180
                    print("ROT: " + str(obj_rot_rads))
                    #obj_rot_rads = -float(vehicleInfoDict[key][16]) + self.gtaSample.rawCamRotation
                else:
                    print("8")
                    obj_rot_rads = float(vehicleInfoDict[key][16]) + self.gtaSample.rawCamRotation - 90 - 180

            obj_rot_rads = self.degreesToRad(obj_rot_rads)

            # make sure the ry is between [-pi, pi]
            if obj_rot_rads > math.pi:
                obj_rot_rads = -(math.pi + (math.pi - obj_rot_rads)) 
            elif obj_rot_rads < -math.pi:
                obj_rot_rads = math.pi + (math.pi + obj_rot_rads)

            #### Calculate 2D and 3D bounding boxes ####
            box3d_pts_2d, box3d_pts_3d = compute_box_3d(bb3d_length, bb3d_width, bb3d_height, obj_rot_rads, rotatedVehiclePos, self.p0_mat, self.R0, self.C2V)
            
            xmin = 0
            ymin = 0
            xmax = 0
            ymax = 0
            print(len(box3d_pts_2d))
            for i in range(0, len(box3d_pts_2d)):
                if i == 0:
                    xmin = box3d_pts_2d[i,0]
                    ymin = box3d_pts_2d[i,1]
                    xmax = box3d_pts_2d[i,0]
                    ymax = box3d_pts_2d[i,1]
                else:
                    if box3d_pts_2d[i,0] < xmin:
                        xmin = box3d_pts_2d[i,0]
                    elif box3d_pts_2d[i,0] > xmax:
                        xmax = box3d_pts_2d[i,0]
                    
                    if box3d_pts_2d[i,1] < ymin:
                        ymin = box3d_pts_2d[i,1]
                    elif box3d_pts_2d[i,1] > ymax:
                        ymax = box3d_pts_2d[i,1]

            if xmin < 0 and xmax > kitti_width and ymin < 0 and ymax > kitti_height: # ignore object
                continue
            
            # make sure that the uv coorsinates are within image bounds
            if xmin < 0: 
                xmin = 0
            if xmax >= kitti_width:
                xmax = kitti_width-1
            if ymin < 0:
                ymin = 0
            if ymax >= kitti_height:
                ymax = kitti_height-1

            boundingBoxList.append((xmin, ymin, xmax, ymax))

            # object type: car
            label_line += vehicleInfoDict[key][22] + " "
            
            # truncated
            label_line += "0 "

            # occluded
            label_line += "0 "

            # alpha
            label_line += "0 "

            # minx, miny, maxx, maxy
            label_line += str(int(xmin)) + " " + str(int(ymin)) + " " + str(int(xmax)) + " " + str(int(ymax)) + " "

            label_line += str(bb3d_height) + " " + str(bb3d_width) + " " + str(bb3d_length) + " "
            
            label_line += str(rotatedVehiclePos[0]) + " " + str(rotatedVehiclePos[1]) + " " + str(rotatedVehiclePos[2]) + " "

            label_line += str(obj_rot_rads) + " "

            contents_list.append(label_line)

        # show resulting bounding boxes in kitti images
        #self.gtaSample.imageView.showViewWith2dBoundingBoxes(boundingBoxList, self.gtaSample.imageView.kittiImage, color = (0, 0, 255), window_title = "Bounding box results", window_size = 0.8)
        
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
            
            dict_vehicle_projected_center[key].append(int(float(dict_vehicles_dim[str(key)][17])))
            dict_vehicle_projected_center[key].append(int(float(dict_vehicles_dim[str(key)][18])))

            #print("New projection: " + str(self.dict_2d_bb_NEW[key][0]) + " " + str(self.dict_2d_bb_NEW[key][1]) + " " + str(self.dict_2d_bb_NEW[key][2]) + " " + str(self.dict_2d_bb_NEW[key][3]))
        
        self.gtaSample.imageView.showViewWith2dBoundingBoxes(self.gtaSample.dict_2d_bb_NEW, self.gtaSample.imageView.gtaImage, self.gtaSample.imageView.dict_2d_bb_of_kitti_image.keys(), window_size = 0.7, object_centers = dict_vehicle_projected_center)
        pass












