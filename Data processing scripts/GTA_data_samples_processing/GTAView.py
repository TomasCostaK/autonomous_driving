import numpy as np
import cv2
import os.path
import math

class GTAView:
    '''
    Processing of the GTA image views to transform them into the same resolution as the 
    images used in the kitti dataset.
    '''
    # .ply file with the point cloud (stores the positions of the points)
    pc_ply_fn = "LiDAR_PointCloud.ply"
    # file with a label for each point of the point cloud (background, pedestrian, vehicle, props)
    pc_labels_fn = "LiDAR_PointCloud_labels.txt"
    # file with an id for every gameobject in the point cloud
    pc_labels_detailed_fn = "LiDAR_PointCloud_labelsDetailed.txt"
    # file that contains point position (x, y, z), point projected (x, y) pixels, and the index of the view that it's projected on (0, 1, or 2)
    # the index 0 means front view image
    pc_projected_points_fn = "LiDAR_PointCloud_points.txt"

    # image taken in gta, resolution equal to the screen
    gta_image = None
    # original image resolution taken by the kitti camera
    kitti_cam_image = None
    # properly transformed image view to be equal to the images present in the kitti dataset
    kitti_image = None
    # percentage of resize used to shrink the original image view resolution down to the resolution of the kitti camera
    resize_percentage = None

    directory_path = ""
    fv_img_fn = ""

    dict_2d_bb_of_gta_image = {}

    dict_2d_bb_of_kitti_image = {}

    def __init__(self, sample_directory_path, fv_img_fn):
        self.directory_path = sample_directory_path
        self.fv_img_fn = fv_img_fn
        self.transform_image_as_kitti_dataset()

    def transform_image_as_kitti_dataset(self):
        '''
        Make the image capture in gta the same dimensions as the images of the kitii dataset
        '''
        # load original image view
        self.gta_image = cv2.imread(os.path.join(self.directory_path, self.fv_img_fn), cv2.IMREAD_UNCHANGED)

        h_gta, w_gta, c_gta = self.gta_image.shape

        self.kitti_cam_image = self.image_resize(self.gta_image, width = 1392)
        h_kitti, w_kitti, c_kitti = self.kitti_cam_image.shape
        print(self.kitti_cam_image.shape)

        self.resize_percentage = w_kitti/w_gta # 0.725
        print(self.resize_percentage)

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

    def show_image(self, image, window_title = "Image view", window_size = 0.5):
        '''
        Open window showing the transformed GTA image view into the resolution used in the kitti dataset
        image: opencv image
        '''
        resized_image = cv2.resize(image, (0,0), fx=window_size, fy=window_size) 

        cv2.imshow(window_title + ", " + str(int(window_size*100)) + "% zoom", resized_image)
        print("Press any key to continue...")
        cv2.waitKey(0)
        
    def save_image(self, image, filename = "Image_default_name.bmp"):
        '''
        Save kitti image.
        '''
            
        cv2.imwrite(os.path.join(self.directory_path, filename), image)

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

    def calculate_2d_bb_for_gta_image(self, label, dict_with_same_labeled_objs, object_ids, dict_objs_projected_points):
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

        self.dict_2d_bb_of_gta_image = bounding_box_2d_for_each_car

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
            cv2.rectangle(image_copy, (int(minx), int(miny)), (int(maxx), int(maxy)), color, 2)

        # the bounding boxes are already drawn in the image, so they will also be resized according to the given amount
        resized_image = cv2.resize(image_copy, (0,0), fx=window_size, fy=window_size) 

        cv2.imshow(window_title + ", " + str(int(window_size*100)) + "% zoom", resized_image)
        print("Press any key to continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def calculate_new_2d_bounding_boxes_for_kitti_img(self, dict_2d_bounding_boxes, discard_trucated_boxes = True):
        '''
        Calculate 2d bounding boxes for the resized and cropped image views that match kitti's images.

        TODO: Support for discard_truncated_boxes = false
        TODO: discard the list of points belonging to objects where their 2d bounding boxes are out of bounds
        '''
        original_height, original_width, original_channels = self.gta_image.shape
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
            #print("before: minX=" + str(dict_2d_bounding_boxes[i][0]) + ", maxX=" + str(dict_2d_bounding_boxes[i][1]) + ", minY=" + str(dict_2d_bounding_boxes[i][2]) + ", maxY=" + str(dict_2d_bounding_boxes[i][3]))
            # minX
            minX = math.ceil(dict_2d_bounding_boxes[i][0]*resize_percentage) - lr_bar
            # maxX
            maxX = math.ceil(dict_2d_bounding_boxes[i][1]*resize_percentage) - lr_bar
            # minY
            minY = math.ceil(dict_2d_bounding_boxes[i][2]*resize_percentage) - ub_bar
            # maxY
            maxY = math.ceil(dict_2d_bounding_boxes[i][3]*resize_percentage) - ub_bar
            #print("after: minX=" + str(minX) + ", maxX=" + str(maxX) + ", minY=" + str(minY) + ", maxY=" + str(maxY))
            
            dict_kitti_2d_bounding_boxes[i] = []
            dict_kitti_2d_bounding_boxes[i].append(minX)
            dict_kitti_2d_bounding_boxes[i].append(maxX)
            dict_kitti_2d_bounding_boxes[i].append(minY)
            dict_kitti_2d_bounding_boxes[i].append(maxY)

        self.dict_2d_bb_of_kitti_image = dict_kitti_2d_bounding_boxes
























