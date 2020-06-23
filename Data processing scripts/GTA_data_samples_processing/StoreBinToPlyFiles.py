import os
from GtaSample import GtaSample
from KittiSample import KittiSample
from LoadBinPointclouds import loadKittiVelodyneFile
from LoadBinPointclouds import savePlyFile
from kitti_util import compute_box_3d
#from KittiSample import inverse_rigid_trans

pathToPointcloudBinDirectory = 'D:/diogo/Desktop/Tese/Tese_CODE/frustum-pointnets-master-kitti-data/dataset/KITTI/object/training/velodyne/'

outputpath = 'D:/diogo/Desktop/NewSolutionKitti/BinToPlyPointclouds/'

counter = 0

# Create ply point clouds from bin pointclouds (with intensity) from kitti dataset, with a red-green color contrast to trepresent the intensity value of each point
for subdir, dirs, files in os.walk(pathToPointcloudBinDirectory):

    for filename in files:

        # dimensions of the cars in a label
        label_cars_dims = []

        kittipointcloud = loadKittiVelodyneFile(pathToPointcloudBinDirectory + filename, include_luminance = True)

        savePlyFile(outputpath + filename.split('.')[0] + ".ply", kittipointcloud)
        #savePlyFile(outputpath + filename.split('.')[0] + "_intensity" + ".ply", kittipointcloud, attributes = 'i')

        counter+=1

        print(str(int(counter/len(files) * 100)) + '%')















# for subdir, dirs, files in os.walk(pathToPointcloudBinDirectory):

#     for filename in files:
#         data_labels = []
#         data_car_labels = []
#         data_calib = []

#         # read the calibration file corresponding to the poitncloud into a string list
#         with open(pathToCalibDirectory + filename.split('.')[0] + '.txt', 'r') as file:
#             data_calib = file.read().split('\n')

#         p0_mat = []
#         R0_mat = []
#         tr_velo_to_cam = []
#         for line in data_calib:
#             split_line = line.split(':')

#             if split_line[0] == 'P0':
#                 mat_elem_list = split_line[1].split(' ')
#                 count = 0
#                 mat_line = [] # 4 elems
#                 for i in range(0, mat_elem_list):
#                     if count == 4:
#                         count = 0
#                         p0_mat.append(mat_line)
#                         mat_line = []
                    
#                     mat_line.append(mat_elem_list[i])
#                     count+=1

#             print(split_line[0])

#         V2C = np.array(tr_velo_to_cam)
#         V2C = np.reshape(V2C, [3,4])
#         C2V = inverse_rigid_trans(V2C)

#         print(data_calib)

#         # read the label file corresponding to the poitncloud into a string list
#         with open(pathToLabelsDirectory + filename.split('.')[0] + '.txt', 'r') as file:
#             data_labels = file.read().split('\n')

#         # add all car elements to a new list
#         for label_str in data_labels:
#             label_list = label_str.split(' ')
#             if label_list[0] == 'Car':
#                 data_car_labels.append(label_str)
        
#         print(data_car_labels)

#         # ignore pointcloud without any car
#         if len(data_car_labels) <= 0:
#             continue
        
#         # dimensions of the cars in a label
#         label_cars_dims = []

#         kittipointcloud = loadKittiVelodyneFile(pathToPointcloudBinDirectory + filename, include_luminance = True)

#         savePlyFile(outputpath + filename.split('.')[0] + ".ply", kittipointcloud)

#         counter+=1

#         print(str(counter/len(files) * 100) + '%')




























