import numpy as np
import cv2
import os.path
import math

class GtaView:
    '''
    Processing of the GTA image views to transform them into the same resolution as the 
    images used in the kitti dataset.
    '''
    
    # image taken in gta, resolution equal to the screen
    gtaImage = None
    # original image resolution taken by the kitti camera
    kittiCamImage = None
    # properly transformed image view to be equal to the images present in the kitti dataset
    kittiImage = None
    # percentage of resize used to shrink the original image view resolution down to the resolution of the kitti camera
    resizePercentage = None

    def __init__(self, sampleDirPath, fvImgFn):
        self.directoryPath = sampleDirPath
        self.fvImgFn = fvImgFn

        self.transformImageForKittiDataset()

    def transformImageForKittiDataset(self):
        '''
        Makes the image captured in gta the same dimensions as the images of the kitti dataset.
        '''
        # load original image view
        self.gtaImage = cv2.imread(os.path.join(self.directoryPath, self.fvImgFn), cv2.IMREAD_UNCHANGED)

        h_gta, w_gta, c_gta = self.gtaImage.shape

        self.kittiCamImage = self.imageResize(self.gtaImage, width = 1392)
        h_kitti, w_kitti, c_kitti = self.kittiCamImage.shape
        print(self.kittiCamImage.shape)

        self.resizePercentage = w_kitti/w_gta # 0.725
        print(self.resizePercentage)

        # cut height to 512 pixels (obtained the region of interest (roi)); maintain the same width
        roiDesiredMiddleHeight = 512
        startRow = int((h_kitti-roiDesiredMiddleHeight)/2)
        roiImage = self.kittiCamImage[startRow:startRow+roiDesiredMiddleHeight, 0:w_kitti-1]
        h_roi, w_roi, c_roi = roiImage.shape
        
        # kitti size: 1224x370
        desiredRectMiddleWidth = 1224
        desiredRectMiddleHeight = 370

        startRectRow = int((h_roi-desiredRectMiddleHeight)/2)
        startRectColumn = int((w_roi-desiredRectMiddleWidth)/2)

        self.kittiImage = roiImage[startRectRow:startRectRow+desiredRectMiddleHeight, startRectColumn:startRectColumn+desiredRectMiddleWidth]

    def getKittiImageDimensions(self):
        kitti_height, kitti_width, kitti_channels = self.kitti_image.shape

        return kitti_height, kitti_width, kitti_channels

    def imageResize(self, image, width = None, height = None, inter = cv2.INTER_AREA):
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

    def showImage(self, image, window_title = "Image view", window_size = 0.5):
        '''
        Open window showing the transformed GTA image view into the resolution used in the kitti dataset
        image: opencv image
        '''
        resized_image = cv2.resize(image, (0,0), fx=window_size, fy=window_size) 

        cv2.imshow(window_title + ", " + str(int(window_size*100)) + "% zoom", resized_image)
        print("Press any key to continue...")
        cv2.waitKey(0)
        
    def saveImage(self, image, dirPath, filename):
        '''
        Save kitti image.
        '''    
        cv2.imwrite(os.path.join(dirPath, filename), image)

    def showViewWith2dBoundingBoxes(self, boundingBoxList, image_opencv, color = (0, 0, 255), window_title = "Bounding box results", window_size = 0.5):
        '''
        Open a window showing the 2d bounding boxes over the given image view.
        boundingBoxList: list of tuples (minx, miny, maxx, maxy)
        '''
        # copy opencv image variable to make the added 2d bounding box rectangles not persistent in the image
        image_copy = image_opencv.copy()

        for box_tuple in boundingBoxList:
            minx = box_tuple[0]
            maxx = box_tuple[2]
            miny = box_tuple[1]
            maxy = box_tuple[3]
            cv2.rectangle(image_copy, (int(minx), int(miny)), (int(maxx), int(maxy)), color, 2)

        #if object_centers is not None:
        #    print("Object centers: " + str(object_centers))
        #    for key in object_centers.keys():
        #        cv2.rectangle(image_copy, (object_centers[key][0]-5, object_centers[key][1]-5), (object_centers[key][0]+5, object_centers[key][1]+5), [255,0,0], 2)
  
        # the bounding boxes are already drawn in the image, so they will also be resized according to the given amount
        resized_image = cv2.resize(image_copy, (0,0), fx=window_size, fy=window_size) 

        cv2.imshow(window_title + ", " + str(int(window_size*100)) + "% zoom", resized_image)

        print("Press any key to continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def calculate2dBoundingBoxesForKittiImgSize(self, tuple_2d_coords, discard_trucated_boxes = True, min_height = 40, min_length = 40):
        '''
        Calculate 2d bounding boxes for the resized and cropped image views that match kitti's images.
        '''
        original_height, original_width, original_channels = self.gtaImage.shape
        kitti_height, kitti_width, kitti_channels = self.kittiImage.shape

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
        print("kitti_height: " + str(kitti_height))
        kci_h, kci_w, kci_c = self.kittiCamImage.shape
        print("kitti_cam_width: " + str(kci_w))
        print("kitti_cam_height: " + str(kci_h))

        lr_bar = int((kci_w-kitti_width) / 2)
        ub_bar = int((kci_h-kitti_height) / 2)
        print("lr_bar_width: " + str(lr_bar))
        print("ub_bar_height: " + str(ub_bar))

        resize_percentage = kci_w/original_width # 0.725

        #dict_kitti_2d_bounding_boxes = {}
        #for i in dict_2d_bounding_boxes.keys():
        #print("before: minX=" + str(dict_2d_bounding_boxes[i][0]) + ", maxX=" + str(dict_2d_bounding_boxes[i][1]) + ", minY=" + str(dict_2d_bounding_boxes[i][2]) + ", maxY=" + str(dict_2d_bounding_boxes[i][3]))
        # minX
        minX = math.ceil(tuple_2d_coords[0]*resize_percentage) - lr_bar
        # maxX
        maxX = math.ceil(tuple_2d_coords[2]*resize_percentage) - lr_bar
        # minY
        minY = math.ceil(tuple_2d_coords[1]*resize_percentage) - ub_bar
        # maxY
        maxY = math.ceil(tuple_2d_coords[3]*resize_percentage) - ub_bar
        #print("after: minX=" + str(minX) + ", maxX=" + str(maxX) + ", minY=" + str(minY) + ", maxY=" + str(maxY))

        bb_height = maxX - minX
        bb_length = maxY - minY
        
        #divider_miny = 12
        #if bb_height > 100:
        #    divider_miny = 20

        #divider_maxy = 100
        #if bb_height < 40:
        #    divider_maxy = 20
        

        #minY = minY - int(bb_height/divider_miny) # the origin is the top left corner of the image
        #maxY = maxY + int(bb_height/divider_maxy)


        return minX, minY, maxX, maxY
        # add bounding box to dictionary
        #dict_kitti_2d_bounding_boxes[i] = []
        #dict_kitti_2d_bounding_boxes[i].append(minX)
        #dict_kitti_2d_bounding_boxes[i].append(maxX)
        #dict_kitti_2d_bounding_boxes[i].append(minY - int(bb_height/divider_miny))  # the origin is the top left corner of the image
        #dict_kitti_2d_bounding_boxes[i].append(maxY + int(bb_height/divider_maxy))
        #print(dict_kitti_2d_bounding_boxes)

        #self.dict_2d_bb_of_kitti_image = dict_kitti_2d_bounding_boxes

