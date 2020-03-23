import os
from GTASample import GTASample

'''
===== Kitti dataset structure =====
Source: http://www.cvlibs.net/datasets/kitti/eval_object.php

./GeneratedKittiSamples/
    data_object_image_2/
        training/
            000000.png
            000001.png
            000002.png
            (...)

    data_object_velodyne/
        training/
            000000.bin
            000001.bin
            000002.bin
            (...)

    data_object_label_2/
        training/
            000000.txt
            000001.txt
            000002.txt
            (...)

The information in data_object_calib/ is not needed because the points come already projected onto the image from GTA.

===== Labels files format =====
Source: https://github.com/pratikac/kitti/blob/master/readme.raw.txt

(1 value) label_name: 'Car', 'Van', 'Truck', 'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram', 'Misc' (qq outro tipo de vehiculo) or 'DontCare' (quando se pertende ignorar um veiculo)

(1 value) trucanted: [0, 1], evaluates how much an object  isn't within the image boundaries

(1 value) occluded: {0, 1, 2, 3}, fully-visible, partly occluded, largely-occuled, unknown

(1 value) alpha: [-pi, pi], observation angle of the object

(4 values) bbox: {left, top, right, bottom}, 2d bounding box of the object in the image

(3 values) dimensions: {height, width, length}, 3d object dimensions in meters

(3 values) location: {x, y, z}, 3d object location in camera coordinates in meters

(1 value) rotation_y: [-pi, pi], rotation around Y-axis (up axis) in camera coordinates

OPTIONAL PARAMETER- it's only used when you make a submission to the KITTI web site. When training a model in DIGITS, you don't have to provide a score and if you do, the value will be ignored. (https://github.com/NVIDIA/DIGITS/issues/992)
(1 value) score: Float, indicating confidence in detection, needed for p/r curves, higher is better.
'''


# root directory where the samples generated in gta are stored
rootdir = './GTASamples'

gen_kitti_samples_dir = "./GeneratedKittiSamples"
# 2 means left images. As labels dependem de se a image foi tirada com a camara da esquerda ou da direita devido às coordenadas 2D das bb q sao projetadas na imagem
kitti_labels_dirname = "/data_object_label_2/training"
# pos, posy, poz, reflection intensity (sempre igual a 1 para simular q todos os raios sao refletidos com a mesma e máxima intensidade)
kitti_velodyne_dirname = "/data_object_velodyne/training"
kitti_images_dirname = "/data_object_image_2/training"

sampleCounter = 0

# determine how many samples are in one of the destination directories to increase the counter accordingly
for subdir, dirs, files in os.walk(gen_kitti_samples_dir + kitti_labels_dirname):
    for sample_label_file in files:
        sampleCounter += 1

print("Current number of samples: " + str(sampleCounter))

for subdir, dirs, files in os.walk(rootdir):
    for dir_name in dirs:
        print("\n\n======== Sample directory: " + dir_name + "========")
        #for file in files:
        #    print os.path.join(subdir, file)

        # load sample (point cloud + front view image) and create the an original pointcloud, a rotated point cloud, a front view point cloud, the kitti dataset resolution image
        pc_sample1 = GTASample(rootdir + "/" + dir_name)

        # load and save velodyne poincloud (.bin file) (from the kitti dataset) and store it into a .ply file to be visible in blender
        #points_list = pc_sample1.load_kitti_velodyne_file('./000000.bin')
        #pc_sample1.save_ply_file("velodyne_pointcloud.ply", points_list)

        # save the original point cloud (not rotated) into a file
        #pc_sample1.save_ply_file("Original point cloud.ply", pc_sample1.pc_raw_data.list_raw_pc)

        # save the original point cloud rotated into a file
        pc_sample1.save_ply_file("Rotated point cloud.ply", pc_sample1.pc_fv_raw_data.list_rotated_raw_pc)

        # create a point cloud only with points with label = 2, vehicles
        pc_sample1.pc_fv_raw_data. \
            generate_single_category_point_cloud(2, category_name="vehicles", debug_mode=True)
        
        # if no vehicle points were detected in the front view point cloud, pass to the next sample
        if 2 not in pc_sample1.pc_fv_raw_data.single_category_pcs_list.keys():
            continue

        # save the vehicles pointcloud into a file
        pc_sample1.save_ply_file_from_dict("Vehicles point cloud.ply", \
            pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].get_colored_point_cloud_dict_by_detailed_labels(), attributes = 'c')

        # show gta resolution front view image
        #pc_sample1.imageView.show_image(pc_sample1.imageView.gta_image, window_title = "Hello.bmp", window_size = 0.6)

        # show kitti camera resolution front view image
        #pc_sample1.imageView.show_image(pc_sample1.imageView.kitti_cam_image, window_title = "Hello.bmp", window_size = 0.6)

        # show kitti dataset resolution front view image
        #pc_sample1.imageView.show_image(pc_sample1.imageView.kitti_image, window_title = "Hello.bmp", window_size = 0.6)

        # calculate 2d bounding boxes surrounding objects with label 2, vehicles, from the gta resolution front view.
        pc_sample1.imageView.calculate_2d_bb_for_gta_image(2, pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].dict_of_positions_per_obj,
                                                            pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].object_ids_list,
                                                            pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].dict_projected_coords_per_obj)

        # show gta resolution front view image and the bounding boxes surrounding the vehicles
        #pc_sample1.imageView.show_image_view_with_2d_bounding_boxes(pc_sample1.imageView.dict_2d_bb_of_gta_image, pc_sample1.imageView.gta_image, pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].object_ids_list)

        # calculate 2d bounding boxes surrounding objects with label 2, vehicles, from the kitti dataset resolution front view.
        pc_sample1.imageView.calculate_new_2d_bounding_boxes_for_kitti_img(pc_sample1.imageView.dict_2d_bb_of_gta_image)

        # show kitti dataset resolution front viw with the calculated 2d bounding boxes
        pc_sample1.imageView.show_image_view_with_2d_bounding_boxes(pc_sample1.imageView.dict_2d_bb_of_kitti_image, pc_sample1.imageView.kitti_image, pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].object_ids_list, window_size = 1)

        













