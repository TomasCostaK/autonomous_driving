import os
from GtaSample import GtaSample
from KittiSample import KittiSample

##### Input and output directory paths #####

# root directory where the samples generated in gta are present
rootDir = './GTASamples/'

# output directory for the generated kiiti formated samples
rootKittiOutputDir = './KittiOutput/'
# kitti labels output directory (index 2 is for left images in kitti)
kittiLabelsDir = 'data_object_label_2/training/label_2/'
# velodyne samples output directory
kittiVelodyneDir = 'data_object_velodyne/training/'
# image view samples output directory
kittiViewsDir = 'data_object_image_2/training/'
# calibration matrices samples output directory
kittiCalibDir = 'data_object_calib/training/calib/'

# to generate the naming sequence for the kitti samples
sampleCounter = 0

to_overwrite = input("Do you want to overwrite previous kitti samples? (No = 0, Yes = 1)\n> ")

if not int(to_overwrite):
    # determine how many samples are in one of the destination directories to increase the counter accordingly
    for subdir, dirs, files in os.walk(rootKittiOutputDir + kittiLabelsDir):
        for sample_label_file in files:
            sampleCounter += 1

print("Current number of samples in " + rootKittiOutputDir + ": " + str(sampleCounter))


# walk through all samples in rootDir and create the kitti output accordingly
for subdir, dirs, files in os.walk(rootDir):
    for dirName in dirs:
        print('\n\nCurrent sample directory: ' + dirName + '')

        # load sample (point cloud + front view image) and create the original pointcloud, a rotated point cloud, a front view point cloud, the kitti dataset resolution image
        pc_sample1 = GtaSample(rootDir + dirName)

        # save the original point cloud (not rotated) into a file
        pc_sample1.savePlyFile('Original point cloud.ply', pc_sample1.pcData.list_raw_pc)
        
        pc_sample1.savePlyFile("Rotated point cloud.ply", pc_sample1.pcData.list_rotated_raw_pc)

        pc_sample1.savePlyFile("Frontview point cloud.ply", pc_sample1.pcFvData.list_rotated_raw_pc)

        # create a point cloud only with points with label = 2, vehicles
        pc_sample1.pcFvData. \
            generateSingleCategoryPointCloud(2, category_name="vehicles", debug_mode=True)

        # if no vehicle points were detected in the front view point cloud, pass to the next sample
        if 2 not in pc_sample1.pcFvData.single_category_pcs_list.keys():
            continue

        # save the vehicles frontview pointcloud into a file
        pc_sample1.savePlyFileFromDict("Vehicles point cloud.ply", \
            pc_sample1.pcFvData.single_category_pcs_list[2].getColoredPointCloudDictByDetailedLabels(), attributes = 'c')

        kittiSample1 = KittiSample(pc_sample1, rootKittiOutputDir, kittiLabelsDir, kittiVelodyneDir, kittiViewsDir, kittiCalibDir, sampleCounter)

        #pc_sample1.outputKittiLabelFile(sampleCounter)

        # show kitti dataset resolution front viw with the calculated 2d bounding boxes
        #pc_sample1.imageView.show_image_view_with_2d_bounding_boxes(pc_sample1.imageView.dict_2d_bb_of_kitti_image, pc_sample1.imageView.kitti_image, pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].object_ids_list, window_size = 1)

        sampleCounter += 1



