
import os
import shutil
import glob

# The following files need to be in the root dir:
#   - calib (name: 007481.txt)
#   - left image (name: left.png)
#   - right image (name: right.png)
#   - subval2
#   - train2
#   - trainval2
#   - config

print("Moving files...")
if os.path.exists("007481.txt"):
    shutil.move("007481.txt", "pseudo_lidar_V2/kitti/testing/calib/007481.txt")
if os.path.exists("left.png"):
    shutil.move("left.png", "pseudo_lidar_V2/kitti/testing/image_2/007481.png")
if os.path.exists("right.png"):
    shutil.move("right.png", "pseudo_lidar_V2/kitti/testing/image_3/007481.png")
if os.path.exists("sdn_kitti_train.config"):
    shutil.move("sdn_kitti_train.config", "pseudo_lidar_V2/src/configs/sdn_kitti_train.config")
shutil.copy("pseudo_lidar_V2/src/configs/sdn_kitti_train.config", "pseudo_lidar_V2/src/configs/sdn_kitti_test.config")
for split_file in glob.glob("*.txt"):
    shutil.move(split_file, "pseudo_lidar_V2/split/")
print("Done.")
