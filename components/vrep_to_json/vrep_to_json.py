import xml.etree.ElementTree as ET
import json
import sys
import random
import math
import numpy as NP
import ctypes
from numpy import linalg as LA



#GLOBAL
NODE_ID = 0

def convert_vreptype_to_jsontype(name):
    if "wall" in name.lower():
        return 'plane'
    elif "floor" in name.lower():
        return "plane"
    elif "cubo" in name.lower():
        return "mesh"
    return ""


def get_color_type(type_):
    if type_ == "world":
        color = "SeaGreen"
    elif type_ == "transform":
        color = "SteelBlue"
    elif type_ == "plane":
        color = "Khaki"
    elif type_ == "differentialrobot" or type_ == "omnirobot":
        color = "GoldenRod"
    elif type_ == "laser":
        color = "GreenYellow"
    elif type_ == "mesh":
        color = "LightBlue"
    elif type_ == "imu":
        color = "LightSalmon"
    else:
        color = "Blue"
    return color


def check_truncate_value(name):
    return name in ["width", "height", "depth", "scalex", "scaley", "scalez"]


# str:0, int:1, float:2, [float]:3, bool:4, uint: 6
def type_to_integer(value, round):
    try:
        fl = float(value)
        if round or fl.is_integer():
            return 1, int(float(value))
        else:
            return 2, float(value)
    except ValueError:
        if value.lower() == "true":
            return 4, True
        elif value.lower() == "false":
            return 4, False
        return 0, value


def create_node(name, type, others):
    global NODE_ID
    NODE_ID += 1
    new_node = dict()
    new_node["id"] = NODE_ID
    new_node["name"] = name
    new_node["type"] = type
    new_node["attribute"] = {}
    new_node["links"] = []

    new_node["attribute"]["pos_x"] = {"type": 2, "value": random.randint(-200, 200)}
    new_node["attribute"]["pos_y"] = {"type": 2, "value": random.randint(-200, 200)}
    new_node["attribute"]["color"] = {"type": 0, "value": get_color_type(new_node["type"])}

    for name, value in others:
        if name == "parent":
            type = 6
            val = value
        else:
            type, val = type_to_integer(value, check_truncate_value(name))
        new_node["attribute"][name] = {"type": type, "value": val}

    return new_node


def create_edge(src, dst, type, pos, rot):
    new_edge = dict()
    new_edge["src"] = src
    new_edge["dst"] = dst
    new_edge["label"] = type
    new_edge["linkAttribute"] = {}
    if type == "RT":
        new_edge["linkAttribute"]["translation"] = {"type": 3, "value": pos}
        new_edge["linkAttribute"]["rotation_euler_xyz"] = {"type": 3, "value": rot}
    return new_edge


# convert size[] to width, height, depth values
def convert_size_to_parameters(type, size):
    if type == "plane" or type == "mesh" or type == "floor":
        return [("width", size[0]), ("height", size[1]), ("depth", size[2])]


def convert_color_to_texture(type, color):
    if type == "plane":
        value = '#%02x%02x%02x' %(color[0], color[1], color[2])
        new_attrib = [("texture", value)]
    else:
        # TODO: read mesh from coppelia
        new_attrib = [("path", "/home/robocomp/robocomp/files/osgModels/basics/cube2.3ds")]
    return new_attrib


def main(input_file, output_file):

    new_json = {}
    new_json["DSRModel"] = {}
    new_json["DSRModel"]["symbols"] = {}


    tree = ET.parse(input_file)
    root = tree.getroot()

    # insert world node
    w_node = create_node("world", "world", [("parent", 0), ("level", 0), ("OuterRegionBottom", -4250), ("OuterRegionLeft", -2000), ("OuterRegionRight",7500), ("OuterRegionTop",4250)])
    new_json["DSRModel"]["symbols"][str(w_node["id"])] = w_node

    for elem in root.findall('shape'):
        # check required tags
        if len(elem.findall('common/localFrame/position')) and len(elem.findall('primitive/type')):

            name = elem.findall('common/name')[0].text
            pos = [float(value)*1000 for value in elem.findall('common/localFrame/position')[0].text.split(' ')]
            rot = [float(value)*3.14159/180 for value in elem.findall('common/localFrame/euler')[0].text.split(' ')]
            type = convert_vreptype_to_jsontype(name)
#            type = convert_vreptype_to_jsontype(elem.findall('primitive/type')[0].text)
            size = [float(value)*1000 for value in elem.findall('primitive/size')[0].text.split(' ')]
            color = [int(value) for value in elem.findall('primitive/color/ambientDiffuse')[0].text.split(' ')]

            print(name, pos, rot, type, size, color)

            # first try
            if type:
                others = convert_size_to_parameters(type, size)
                if "floor" in name.lower():
                    others += [("texture", "/home/robocomp/robocomp/components/dsr-graph/etc/alab/textures/wood.jpg")]
                else:
                    others += convert_color_to_texture(type, color)
                n_node = create_node(name, type, others)
                new_json["DSRModel"]["symbols"][str(n_node["id"])] = n_node

                # edge
                # TODO: this must be read from Coppelia?¿
                src = w_node["id"]
                dst = n_node["id"]
                edge_type = "RT"
                new_edge = create_edge(src, dst, edge_type, pos, rot)

                # check parent an level attributes
                if "parent" not in new_json["DSRModel"]["symbols"][str(dst)]:
                    new_attribute = dict()
                    new_attribute["value"] = new_edge["src"]
                    new_attribute["type"] = 6
                    new_json["DSRModel"]["symbols"][str(dst)]["attribute"]["parent"] = new_attribute
                if "level" not in new_json["DSRModel"]["symbols"][str(dst)]:
                    new_attribute = dict()
                    new_attribute["value"] = new_json["DSRModel"]["symbols"][str(src)]["attribute"]["level"][
                                                 "value"] + 1
                    new_attribute["type"] = 1
                    new_json["DSRModel"]["symbols"][str(dst)]["attribute"]["level"] = new_attribute

                new_json["DSRModel"]["symbols"][str(src)]["links"].append(new_edge)

    # TODO: how to add robot?
    with open('../../etc/viriato.json') as json_file:
        viriato = json.load(json_file)

        new_edge = create_edge(w_node["id"], 200, "RT", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        new_json["DSRModel"]["symbols"][str(w_node["id"])]["links"].append(new_edge)
        # merge dictionaries
        new_json["DSRModel"]["symbols"] = {**(new_json["DSRModel"]["symbols"]), **(viriato["DSRModel"]["symbols"])}

    # write to file
    with open(output_file, 'w') as outfile:
        json.dump(new_json, outfile, sort_keys=True, indent=4)




if __name__ == '__main__':
    if len(sys.argv) == 1 or len(sys.argv) >= 4:
        print("XML file must be provided")
        print("vrep_to_json input_file.xml output_file.json")
        exit(0)
    elif len(sys.argv) == 2:
        input = sys.argv[1]
        output = sys.argv[1].split('.')[0] + ".json"
    elif len(sys.argv) == 3:
        input = sys.argv[1]
        output = sys.argv[2]

    main(input, output)
