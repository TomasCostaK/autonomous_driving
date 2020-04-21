from GtaSample import GtaSample
from pathlib import Path
import numpy as np
import os.path

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
    def saveKittiVelodyneFile(tuple_list, filename, directory):
        '''
        Saves pointcloud without luminance
        '''
        dt=np.dtype('float,float,float')
        numpyList = np.array(tuple_list,dtype=dt)
        
        f = open(directory + filename, 'w+b')
        numpyList.tofile(f)
        f.close()

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
        # save point cloud
        KittiSample.saveKittiVelodyneFile(self.gtaSample.pcFvData.list_raw_pc, output_file_name + ".bin", self.kittiVelodyneDir)
        # save calibration info
        self.saveCalibInfo(self.kittiCalibDir, output_file_name + ".txt")
        # labels info
        self.saveLabelInfo(self.kittiLabelsDir, output_file_name + ".txt")


    def saveCalibInfo(self, dirname, filename):

        # only one camera means p1=p2=p3=p0
        p0_mat = [[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0]]

        p1_mat=p2_mat=p3_mat=p0_mat

        r0_rect = [[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0]]
        
        tr_velo_to_cam = [[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0]]

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
        line = name + ":\n"
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
            label_line += "-1 "

            # occluded
            label_line += "-1 "

            # alpha
            label_line += "-10 "
            
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
            label_line += vehicleInfoDict[key][19] + " " + vehicleInfoDict[key][20] + " " + vehicleInfoDict[key][21] + " "

            # location
            # rotate location point around z axis according to the angle that the point cloud was rotated (- Z angle of the camera - 90º)
            originalVehiclePoint = (float(vehicleInfoDict[key][11]), float(vehicleInfoDict[key][12]), float(vehicleInfoDict[key][13]))
            rotatedVehiclePos = self.gtaSample.pcData.rotatePointAroundZaxis(originalVehiclePoint, self.gtaSample.pcData.rotation_amount)
            label_line += str(rotatedVehiclePos[0]) + " " + str(rotatedVehiclePos[1]) + " " + str(rotatedVehiclePos[2]) + " "
            
            # rotation_y
            label_line += "-10"

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












