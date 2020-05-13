import xml.etree.ElementTree as ET
import json
import sys
import random

# str:0, int:1, float:2
def type_to_integer(value):
    try:
        fl = float(value)
        if fl.is_integer():
            return 1, int(value)
        else:
            return 2, float(value)
    except ValueError:
        return 0, value


def get_tr_pos(value):
    if value == "tx" or value == "x":
        return 0, 0
    elif value == "ty" or value == "y":
        return 0, 1
    elif value == "tz" or value == "z":
        return 0, 2
    if value == "rx":
        return 1, 0
    elif value == "ry":
        return 1, 1
    elif value == "rz":
        return 1, 2
    else:
        print("Error getting position for value", value, "check input file")
        sys.exit(-1)

def get_color_type(type_):
    if type_ == "world":
        color = "SeaGreen"
    elif type_ == "transform":
        color = "SteelBlue"
    elif type_ == "plane":
        color = "Khaki"
    elif type_ == "differentialrobot":
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


def main(input_file, output_file):
    new_json = {}
    new_json["DSRModel"] = {}
    new_json["DSRModel"]["symbol"] = {}

    tree = ET.parse(input_file)
    root = tree.getroot()

    for elem in root:
        # Symbols
        if elem.tag == "symbol":
            new_symbol = {}
            new_symbol["id"] = elem.attrib["id"]
            new_symbol["type"] = elem.attrib["type"]
            new_symbol["attribute"] = {}
            new_symbol["links"] = []
            for subelem in elem:
                if subelem.attrib["key"] == "imName":
                    new_symbol["name"] = subelem.attrib["value"]
                else:
                    new_attribute = {}
                    type_, value_ = type_to_integer(subelem.attrib["value"])
                    new_attribute["value"] = value_
                    new_attribute["type"] = type_
                    new_symbol["attribute"][subelem.attrib["key"]] = new_attribute
            new_symbol["attribute"]["pos_x"] = {"type": 2, "value": random.randint(-200, 200)}
            new_symbol["attribute"]["pos_y"] = {"type": 2, "value": random.randint(-200, 200)}
            new_symbol["attribute"]["color"] = {"type": 0, "value": get_color_type(new_symbol["type"])}
            new_json["DSRModel"]["symbol"][new_symbol["id"]] = new_symbol
        # Links
        if elem.tag == "link":
            new_edge = {}
            new_edge["src"] = elem.attrib["src"]
            new_edge["dst"] = elem.attrib["dst"]
            new_edge["label"] = elem.attrib["label"]
            new_edge["linkAttribute"] = {}
            if elem.attrib["label"] == "RT":
                new_tr = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
                for subelem in elem:
                    v, pos = get_tr_pos(subelem.attrib["key"])
                    new_tr[v][pos] = float(subelem.attrib["value"])
                new_edge["linkAttribute"]["translation"] = {"type": 3, "value": new_tr[0]}
                new_edge["linkAttribute"]["rotation_euler_xyz"] = {"type": 3, "value": new_tr[1]}
            new_json["DSRModel"]["symbol"][elem.attrib["src"]]["links"].append(new_edge)

    # write to file
    with open(output_file, 'w') as outfile:
        json.dump(new_json, outfile)


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


