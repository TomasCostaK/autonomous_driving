import os 

'''
    Este é o caminho onde este script deve ser colocado
    frustum-pointnets-master\dataset\KITTI\object\training
'''

rootDir = './label_2/'
destination_dir = '../../../../kitti/rgb_detections/'
image_dir = 'dataset/KITTI/object/training/image_2/'

gen_file_name = 'rgb_detection_val.txt'
gen_file_contents = ''
car_label = 2
confidence = 1

sample_counter = 0
# walk through all samples in rootDir and create the kitti output accordingly
for subdir, dirs, files in os.walk(rootDir):
    for label_file in files:
        #filename = './label_2/000000.txt'
        print("filename: " + label_file)
        file_prefix = label_file.split('.')[0]

        with open(rootDir + label_file) as f:
            content = f.readlines()

        content_list = [x.strip() for x in content] 

        for label in content_list:
            label_values_list = label.split(' ')

            gen_file_contents += image_dir + file_prefix + '.png' + ' ' + str(car_label) + ' ' + str(confidence) + ' ' + label_values_list[4] + ' ' + label_values_list[5] + ' ' + label_values_list[6] + ' ' + label_values_list[7] + '\n'


with open(destination_dir + gen_file_name, "w") as text_file:
    text_file.write(gen_file_contents)
        
print("Finished!")

