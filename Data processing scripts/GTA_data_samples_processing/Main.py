from GTASample import GTASample




# load sample (point cloud + front view image) and create the an original pointcloud, a rotated point cloud, a front view point cloud, the kitti dataset resolution image
pc_sample1 = GTASample('./Sample1/', -133.792633)

# save the original point cloud (not rotated) into a file
pc_sample1.save_ply_file("Original point cloud.ply", pc_sample1.pc_raw_data.list_raw_pc)

# save the original point cloud rotated into a file
pc_sample1.save_ply_file("Rotated point cloud.ply", pc_sample1.pc_fv_raw_data.list_rotated_raw_pc)

# create a point cloud only with points with label = 2, vehicles
pc_sample1.pc_fv_raw_data. \
    generate_single_category_point_cloud(2, category_name="vehicles", debug_mode=True)

# save the vehicles pointcloud into a file
pc_sample1.save_ply_file_from_dict("Vehicles point cloud.ply", \
    pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].get_colored_point_cloud_dict_by_detailed_labels(), attributes = 'c')

# show gta resolution front view image
pc_sample1.imageView.show_image(pc_sample1.imageView.gta_image, window_title = "Hello.bmp", window_size = 0.6)

# show kitti camera resolution front view image
pc_sample1.imageView.show_image(pc_sample1.imageView.kitti_cam_image, window_title = "Hello.bmp", window_size = 0.6)

# show kitti dataset resolution front view image
pc_sample1.imageView.show_image(pc_sample1.imageView.kitti_image, window_title = "Hello.bmp", window_size = 0.6)

# calculate 2d bounding boxes surrounding objects with label 2, vehicles, from the gta resolution front view.
pc_sample1.imageView.calculate_2d_bb_for_gta_image(2, pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].dict_of_positions_per_obj,
                                                    pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].object_ids_list,
                                                    pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].dict_projected_coords_per_obj)

# show gta resolution front view image and the bounding boxes surrounding the vehicles
pc_sample1.imageView.show_image_view_with_2d_bounding_boxes(pc_sample1.imageView.dict_2d_bb_of_gta_image, pc_sample1.imageView.gta_image, pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].object_ids_list)

# calculate 2d bounding boxes surrounding objects with label 2, vehicles, from the kitti dataset resolution front view.
pc_sample1.imageView.calculate_new_2d_bounding_boxes_for_kitti_img(pc_sample1.imageView.dict_2d_bb_of_gta_image)

# show kitti dataset resolution front viw with the calculated 2d bounding boxes
pc_sample1.imageView.show_image_view_with_2d_bounding_boxes(pc_sample1.imageView.dict_2d_bb_of_kitti_image, pc_sample1.imageView.kitti_image, pc_sample1.pc_fv_raw_data.single_category_pcs_list[2].object_ids_list)