import xml.etree.ElementTree as ET
import json
import sys
import random
import math
import numpy as NP
from numpy import linalg as LA



IGNORE_PARAMS = [ "imType", "port", "ifconfig", "collide", "noise", "min", "max", "angle", "focal" ]

def check_truncate_value(name):
    return name in ["width", "height", "depth", "scalex", "scaley", "scalez"]

# str:0, int:1, float:2, [float]:3, bool:4
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


def get_tr_pos(value):
    if value == "tx" or value == "x" or value == "px":
        return 0, 0
    elif value == "ty" or value == "y" or value == "py":
        return 0, 1
    elif value == "tz" or value == "z" or value == "pz":
        return 0, 2
    if value == "rx" or value == "nx":
        return 1, 0
    elif value == "ry" or value == "ny":
        return 1, 1
    elif value == "rz" or value == "nz":
        return 1, 2
    else:
        return -1, -1


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


def check_position_attrib(subelem, vector):
    v, pos = get_tr_pos(subelem.attrib["key"])
    if v == -1 or pos == -1:
        return False
    else:
        vector[v][pos] = float(subelem.attrib["value"])
        return True


def convert_nplanes_to_rotation(vector):
    RU = rotationMatrixFromPlane(NP.array(vector))
    return rotationMatrixToEulerAngles(RU)


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return NP.array([x, y, z])

# Build rotation matrix from Normal plane vector
def rotationMatrixFromPlane(vector):
    A = NP.array([0, 0, 1])
    B = NP.array(vector)

    v = NP.cross(A, B)

    if LA.norm(v) == 0:
        return NP.dot(A, B)*NP.identity(3)

    ssc = [[0, -v[2], v[1]],
           [v[2], 0, -v[0]],
           [-v[1], v[0], 0]]

    lp = (1 - NP.dot(A, B)) / (NP.square(LA.norm(v)))

    RU = NP.identity(3) + ssc + NP.dot(ssc, ssc)*lp
    return RU


#Use on conversion from Inner To VREP
def rotationFromInnerToVrep(vector):
    R = [0.0] * 3
    R[0] = -vector[2]
    R[1] = vector[0]
    R[2] = -vector[1]
    return R

def transfromFromInnerToVrep(vector):
    T = [0.0] * 3
    T[0] = vector[2]
    T[1] = -vector[0]
    T[2] = vector[1]
    return T

def main(input_file, output_file):
    new_json = {}
    new_json["DSRModel"] = {}
    new_json["DSRModel"]["symbols"] = {}

    tree = ET.parse(input_file)
    root = tree.getroot()

    # auxiliary, to store translation and rotation from nodes  before setting on links
    node_transforms = {}


    for elem in root:
        # Symbols
        if elem.tag == "symbol":
            new_symbol = {}
            new_symbol["id"] = int(elem.attrib["id"])
            new_symbol["type"] = elem.attrib["type"].lower()
            new_symbol["attribute"] = {}
            new_symbol["links"] = []
            new_tr = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
            for subelem in elem:
                if subelem.attrib["key"] == "imName":
                    new_symbol["name"] = subelem.attrib["value"]
                elif subelem.attrib["key"] in IGNORE_PARAMS:
                    continue
                else:
                    if not check_position_attrib(subelem, new_tr):
                        new_attribute = {}
                        round = check_truncate_value(subelem.attrib["key"])
                        type_, value_ = type_to_integer(subelem.attrib["value"], round)
                        new_attribute["value"] = value_
                        new_attribute["type"] = type_
                        new_symbol["attribute"][subelem.attrib["key"]] = new_attribute
            if new_symbol["type"] == "plane":  # store data to insert on link
                new_tr[1] = convert_nplanes_to_rotation(new_tr[1])
                node_transforms[new_symbol["id"]] = new_tr
            new_symbol["attribute"]["pos_x"] = {"type": 2, "value": random.randint(-200, 200)}
            new_symbol["attribute"]["pos_y"] = {"type": 2, "value": random.randint(-200, 200)}
            new_symbol["attribute"]["color"] = {"type": 0, "value": get_color_type(new_symbol["type"])}
            # check level on initial symbol (world)
            if new_symbol["type"] == "world":
                if "level" not in new_symbol["attribute"]:
                    new_symbol["attribute"]["level"] = {"type": 1, "value": 0}
                new_symbol["attribute"]["OuterRegionBottom"] = {"type": 1, "value": -4250}
                new_symbol["attribute"]["OuterRegionLeft"] = {"type": 1, "value": -2000}
                new_symbol["attribute"]["OuterRegionRight"] = {"type": 1, "value": 7500}
                new_symbol["attribute"]["OuterRegionTop"] = {"type": 1, "value": 4250}
                new_symbol["attribute"]["parent"] = {"type": 6, "value": 0}

            new_json["DSRModel"]["symbols"][elem.attrib["id"]] = new_symbol
        # Links
        if elem.tag == "link":
            new_edge = {}
            new_edge["src"] = int(elem.attrib["src"])
            new_edge["dst"] = int(elem.attrib["dst"])
            new_edge["label"] = elem.attrib["label"]
            new_edge["linkAttribute"] = {}
            if elem.attrib["label"] == "RT":
                new_tr = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
                for subelem in elem:
                    v, pos = get_tr_pos(subelem.attrib["key"])
                    if v == -1:
                        print("Error getting position for value", subelem.attrib["key"], "in source", new_edge["src"], "check input file")
                        sys.exit(0)
                    new_tr[v][pos] = float(subelem.attrib["value"])
                if new_edge["dst"] in node_transforms.keys():  # accumulate RT transformations
#                    print("************")
#                    print (new_tr, node_transforms[new_edge["dst"]])
                    new_tr[0] = [x + y for (x, y) in zip(new_tr[0], node_transforms[new_edge["dst"]][0])]
                    new_tr[1] = [x + y for (x, y) in zip(new_tr[1], node_transforms[new_edge["dst"]][1])]
                    #conversion to VREP
                    new_tr[0] = transfromFromInnerToVrep(new_tr[0])
                    new_tr[1] = transfromFromInnerToVrep(new_tr[1])
#                    print(new_tr)
                new_edge["linkAttribute"]["translation"] = {"type": 3, "value": new_tr[0]}
                new_edge["linkAttribute"]["rotation_euler_xyz"] = {"type": 3, "value": new_tr[1]}

                # check parent an level attributes
                if not "parent" in new_json["DSRModel"]["symbols"][elem.attrib["dst"]]:
                    new_attribute = {}
                    new_attribute["value"] = new_edge["src"]
                    new_attribute["type"] = 6
                    new_json["DSRModel"]["symbols"][elem.attrib["dst"]]["attribute"]["parent"] = new_attribute
                if not "level" in new_json["DSRModel"]["symbols"][elem.attrib["dst"]]:
                    new_attribute = {}
                    new_attribute["value"] = new_json["DSRModel"]["symbols"][elem.attrib["src"]]["attribute"]["level"]["value"] + 1
                    new_attribute["type"] = 1
                    new_json["DSRModel"]["symbols"][elem.attrib["dst"]]["attribute"]["level"] = new_attribute



            new_json["DSRModel"]["symbols"][elem.attrib["src"]]["links"].append(new_edge)

    # write to file
    with open(output_file, 'w') as outfile:
        json.dump(new_json, outfile, sort_keys=True, indent=4)


if __name__ == '__main__':

    if len(sys.argv) == 1 or len(sys.argv) >= 4:
        print("XML file must be provided")
        print("xml_to_json input_file.xml output_file.json")
        exit(0)
    elif len(sys.argv) == 2:
        input = sys.argv[1]
        output = sys.argv[1].split('.')[0] + ".json"
    elif len(sys.argv) == 3:
        input = sys.argv[1]
        output = sys.argv[2]

    main(input, output)


