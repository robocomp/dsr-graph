
import xml.etree.ElementTree as ET
import json
import sys
import random
import os


key_words = ["wall", "floor_plane", "cubo"]

def convert_vreptype_to_jsontype(name):
    for w in key_words:
        if w in name.lower():
            return w
    return ""

def convert_vrep_axis_to_robocomp(pos):
    new_pos = [0]*3
    new_pos[0] = -pos[1]
    new_pos[1] = pos[2]
    new_pos[2] = pos[0]
    return new_pos

def convert_vrep_size_to_robocomp(size):
    new_size = [0]*3
    new_size[0] = size[1]
    new_size[1] = size[2]
    new_size[2] = size[0]
    return new_size


def main(input_file, output_file):

    new_json = {}
    new_json["dimensions"] = {}
    new_json["walls"] = {}
    new_json["boxes"] = {}

    tree = ET.parse(input_file)
    root = tree.getroot()

    for elem in (root.findall('shape') or root.findall('dummy/shape')):
        # check required tags
        if len(elem.findall('common/localFrame/position')) and len(elem.findall('primitive/type')):
            name = elem.findall('common/name')[0].text
            pos = [float(value)*1000 for value in elem.findall('common/localFrame/position')[0].text.split(' ')]
          #  pos = convert_vrep_axis_to_robocomp(pos)
            rot = [float(value)*3.14159/180 for value in elem.findall('common/localFrame/euler')[0].text.split(' ')]

            type = convert_vreptype_to_jsontype(name)
            size = [float(value) * 1000 for value in elem.findall('primitive/size')[0].text.split(' ')]
#            size = convert_vrep_size_to_robocomp(size)
            color = [int(value) for value in elem.findall('primitive/color/ambientDiffuse')[0].text.split(' ')]

            print(name, type, pos, rot[2], size, color)

            if type == "wall":
                new_wall = [0.0, 0.0] + [size[0]] + [size[1]] + [pos[0]] + [pos[1]] + [rot[2]]
                new_json["walls"][name] = new_wall
            elif type == "boxes":
                new_box = [0.0, 0.0] + [size[0]] + [size[1]] + [pos[0]] + [pos[1]] + [rot[2]]
                new_json["boxes"][name] = new_box
            elif type == "floor_plane":
                new_json["dimensions"]["LEFT"] = -size[0]/2
                new_json["dimensions"]["BOTTOM"] = -size[1]/2
                new_json["dimensions"]["WIDTH"] = size[0]
                new_json["dimensions"]["HEIGHT"] = size[1]
                new_json["dimensions"]["TILESIZE"] = 50



    # write to file
    with open(output_file, 'w') as outfile:
        json.dump(new_json, outfile, sort_keys=True, indent=4)


if __name__ == '__main__':
    if len(sys.argv) == 1 or len(sys.argv) >= 4:
        print("XML file must be provided")
        print("vrep_to_json input_file.xml output_file.json")
        exit(0)
    elif len(sys.argv) == 2:
        input_file = sys.argv[1]
        output_file = sys.argv[1].split('.')[0] + ".json"
    elif len(sys.argv) == 3:
        input_file = sys.argv[1]
        output_file = sys.argv[2]

    cwd = os.getcwd()
    if cwd not in input_file:
        input_file = os.path.join(cwd, input_file)

    if cwd not in output_file:
        output_file = os.path.join(cwd, output_file)

    # check file existence
    if os.path.isfile(input_file):
        print("Input file: ", input_file)
        print("Output file: ", output_file)
        main(input_file, output_file)
    else:
        print("Input file does not exists: ", input_file)
