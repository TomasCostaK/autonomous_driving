import cv2
import matplotlib
import numpy
import os

# From https://stackoverflow.com/questions/44650888/resize-an-image-without-distortion-opencv
def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
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

# load GTA image
imagePath = "LiDAR_PointCloud_Camera_Print_Day_0.bmp"
imageName = imagePath.split(".")[0]
gta_img = cv2.imread(imagePath,cv2.IMREAD_UNCHANGED)
#cv2.imshow('image',img)
#cv2.waitKey(0)
print("\nGTA image resolution: \n")
h_gta, w_gta, c_gta = gta_img.shape
print('width:  ', w_gta)
print('height: ', h_gta)
print('channels:', c_gta)
print("\n")

# resize GTA image to the resolution of the taken from the cameras used for KITTI
kitti_cam_image = image_resize(gta_img, width = 1392)
print("Kitti image resolution: \n")
h_kitti, w_kitti, c_kitti = kitti_cam_image.shape
print('width:  ', w_kitti)
print('height: ', h_kitti)
print('channels:', c_kitti)
print("\n")
# cut height to 512 pixels (obtained the region of interest (roi)); maintain the same width
roi_desired_middle_height = 512
start_row = int((h_kitti-roi_desired_middle_height)/2)
roi_image = kitti_cam_image[start_row:start_row+roi_desired_middle_height, 0:w_kitti-1]

print("RoI image resolution: \n")
h_roi, w_roi, c_roi = roi_image.shape
print('width:  ', w_roi)
print('height: ', h_roi)
print('channels:', c_roi)
print("\n")

cv2.imshow('image',roi_image)
cv2.waitKey(0)

# store roi image file
path = '.'
cv2.imwrite(os.path.join(path , imageName + '_roi.png'), roi_image)
cv2.waitKey(0)

# image size after lens distortion rectification (reduce pincushion distortion)
desired_rect_middle_width = 1224
desired_rect_middle_height = 370

start_rect_row = int((h_roi-desired_rect_middle_height)/2)
start_rect_column = int((w_roi-desired_rect_middle_width)/2)

rectified_image = roi_image[start_rect_row:start_rect_row+desired_rect_middle_height, start_rect_column:start_rect_column+desired_rect_middle_width]

print("Rectified image resolution: \n")
h_rect, w_rect, c_rect = rectified_image.shape
print('width:  ', w_rect)
print('height: ', h_rect)
print('channels:', c_rect)
print("\n")

cv2.imshow('image',rectified_image)
cv2.waitKey(0)

# store roi image file
path = '.'
cv2.imwrite(os.path.join(path , imageName + '_rect.png'), rectified_image)
cv2.waitKey(0)


