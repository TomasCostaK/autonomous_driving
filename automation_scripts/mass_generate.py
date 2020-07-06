
import os
import shutil
import subprocess
import glob

from zipfile import ZipFile

if os.path.exists("GTADataset.zip"):
    # Create a ZipFile Object and load sample.zip in it
    with ZipFile('GTADataset.zip', 'r') as zipObj:
       # Extract all the contents of zip file in different directory
       zipObj.extractall('/content/')
    # os.remove("GTADataset.zip")

data_dir = "GTADataset"
# n = 7480
# i = 1
for subdir in os.listdir(data_dir):
    # ni = n + i
    # photo_name = '00' + str(ni)
    photo_name = subdir
    path = os.path.join(data_dir, subdir)

    from GtaView import GtaView

    if os.path.exists("007481.txt"):
        shutil.copy("007481.txt", "pseudo_lidar_V2/kitti/training/calib/" + photo_name + ".txt")
    if os.path.exists(os.path.join(path, "left.png")):
        view = GtaView(path, 'left.png')
        view.transformImageForKittiDataset(kittiImageOutputDir = '/tmp/')
        shutil.copy(os.path.join('/tmp/', "left.png"), "pseudo_lidar_V2/kitti/training/image_2/" + photo_name + ".png")
        os.remove("/tmp/left.png")
    if os.path.exists(os.path.join(path, "right.png")):
        view = GtaView(path, 'right.png')
        view.transformImageForKittiDataset(kittiImageOutputDir = '/tmp/')
        shutil.copy(os.path.join('/tmp/', "right.png"), "pseudo_lidar_V2/kitti/training/image_3/" + photo_name + ".png")
        os.remove("/tmp/right.png")
    # i += 1
    with open("subval2.txt", "a") as sv, open("train2.txt", "a") as t:
        sv.write(photo_name + "\n")
        t.write(photo_name + "\n")

if not os.path.exists("test2.txt"):
    os.mknod("test2.txt")

if os.path.exists("sdn_kitti_train.config"):
    shutil.move("sdn_kitti_train.config", "pseudo_lidar_V2/src/configs/sdn_kitti_train.config")
if not os.path.exists("sdn_kitti_test.config"):
    shutil.copy("pseudo_lidar_V2/src/configs/sdn_kitti_train.config", "pseudo_lidar_V2/src/configs/sdn_kitti_test.config")

for split_file in glob.glob("*.txt"):
    shutil.move(os.path.join("/content", split_file), os.path.join("/content/pseudo_lidar_V2/split/", split_file))

os.chdir("pseudo_lidar_V2")
command = "python ./src/main.py --config ./src/configs/sdn_kitti_train.config --resume ./results/sdn_kitti_train_set/sdn_kitti_object_trainval.pth --dataset kitti --data_list ./split/train2.txt --generate_depth_map --data_tag trainval"
result = subprocess.run(command.split(' '), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#Print the stdout and stderr
print(result.stdout)
print(result.stderr)
print("Done.")

from generate_point_cloud import generate_point_cloud, loadKittiVelodyneFile, savePlyFile

# generate point clouds for all available images (depth maps)
generate_point_cloud()

print("Generating .ply from .bin...")
for pc in os.listdir('/content/pseudo_lidar_V2/results/sdn_kitti_train_set/pseudo_lidar_trainval/'):
    # '007481.bin'
    pc_name = pc.split('/')[-1].strip('.bin')
    # pc_number = int(pc_name) - 7480
    pc_number = pc_name
    ptl = loadKittiVelodyneFile(os.path.join('/content/pseudo_lidar_V2/results/sdn_kitti_train_set/pseudo_lidar_trainval/', pc))
    savePlyFile('/content/GTADataset/' + str(pc_number).zfill(2) + "/" + pc_name + '.ply', ptl)
    shutil.copy("/content/pseudo_lidar_V2/results/sdn_kitti_train_set/depth_maps/trainval/" + pc_name + ".npy", "/content/GTADataset/" + str(pc_number).zfill(2) + "/" + pc_name + ".npy")
print("Done.")

print("Generating zip file...")
command = "zip -r /content/GTADataset_res.zip /content/GTADataset/"
subprocess.run(command.split(" "))
print("Done.")
