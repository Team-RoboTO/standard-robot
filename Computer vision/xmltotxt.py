import os
import glob
import pandas as pd
import xml.etree.ElementTree as ET
import pdb


# image      0   1
# origin:    2   3
labels = {'red armor': 0, 'blue armor': 1, 'grey armor': 2,
          'watcher': 3, 'car': 4, 'ignore': 5, 'base': 6}

#Normalizing we got some values that are >1 or <0. 
#This function is used to cut those off.
def set_exceed(value):
    if(value > 1):
        value = 1
    elif(value < 0):
        value=0
    return value



#TODO: implementation of other origins
def calc_coord(xmin, ymin, xmax, ymax, img_width, img_height, origin):
    # pdb.set_trace()
    width = set_exceed((xmax - xmin) / img_width)
    height = set_exceed((ymax - ymin) / img_height)
    x_center = set_exceed(((xmax + xmin) / 2) / img_width)
    y_center = set_exceed(((ymax + ymin) / 2) / img_height)
    if(origin == 0):
        x_center = 1 - x_center

    return round(x_center, 6), round(y_center, 6), round(width, 6), round(height, 6)


def xml_to_csv(path):
    # pdb.set_trace()
    for xml_file in glob.glob(path + '/*.xml'):
        xml_list = []
        jpg_address = (xml_file).split('.')[0]+'.jpg'
        jpg_filename = os.path.basename(jpg_address)
        jpg_address = os.path.join(os.getcwd(), "images")
        tree = ET.parse(xml_file)
        root = tree.getroot()
        if(os.path.exists(os.path.join(jpg_address, jpg_filename))):
            for member in root.findall('object'):
                bbx = member.find('bndbox')
                xmin = int(float(bbx.find('xmin').text))
                ymin = int(float(bbx.find('ymin').text))
                xmax = int(float(bbx.find('xmax').text))
                ymax = int(float(bbx.find('ymax').text))
                name = member.find('name').text
                if(name == 'armor'):
                    color = member.find('armor_color').text
                    name = f"{color} " + name
                x_center, y_center, width, height = calc_coord(
                    xmin, ymin, xmax, ymax, 1980, 1080, 0)
                #pdb.set_trace()
                value = (labels[name],  # class
                         x_center,
                         y_center,
                         width,
                         height
                         )
                xml_list.append(value)
            xml_list = pd.DataFrame(xml_list)
            xml_list.to_csv(os.path.join(os.getcwd(), "labels", xml_file.split('.')[0]+'.txt'), header=False, index=None, sep=' ')


if(__name__=="__main__"):
    file = xml_to_csv(os.path.join(os.getcwd(), "labels"))
