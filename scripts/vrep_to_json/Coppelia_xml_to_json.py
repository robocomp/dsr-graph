import xml.etree.ElementTree as ET
import json
import sys
import random
import os
import math
import numpy as NP
import ctypes
from numpy import linalg as LA

dictNames = {
	"wall" : "plane",
	"muro" : "plane",
	"pared" : "plane",
	"floor" : "plane",
	"suelo" : "plane",
	"block" : "plane",
	"column" : "plane",
	"cubo" : "mesh",
	"fridge" : "mesh",
	"frigo" : "mesh",
	"table" : "mesh",
	"mesa" : "mesh",
	"chair" : "mesh",
	"silla" : "mesh",
	"cupboard" : "mesh",
	"furniture" : "mesh",
	"muebl" : "mesh"
	}

dictColours = {
	"world" : "SeaGreen",
	"transform" : "SteelBlue",
	"plane" : "Khaki",
	"differentialrobot" : "GoldenRod",
	"omnirobot" : "GoldenRod",
	"laser" : "GreenYellow",
	"mesh" : "LightBlue",
	"imu" : "LightSalmon"
	}
	
dictRobots = {
	"youBot" : "viriato",
	"Pioneer" : "pioneer",
	"Giraff" : "giraff"
	}

#GLOBAL
NODE_ID = 0

##################################################################
### Obtiene el tipo a partir del nombre del objeto detectado
##################################################################
def convert_vreptype_to_jsontype(name):
    for key in dictNames.keys():
        if key in name.lower():
            return dictNames[key]
    return ""
    
##################################################################
### Obtiene el color del nodo a partir del tipo del nodo
##################################################################
def get_color_type(type_):
    return dictColours[type_] if type_ in dictColours.keys() else "Blue"


def check_truncate_value(name):
    return name in ["width", "height", "depth", "scalex", "scaley", "scalez"]

##################################################################
### Dependiendo del valor le da un número que define su tipo ###
##################################################################
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
        
#################################
### Crear un nodo ###
#################################
def create_node(name, type, others):
    global NODE_ID
    NODE_ID += 1
    
    ### nodes 200-299 for the robot
    if NODE_ID == 200:
    	NODE_ID += 100
    
    ### node 500 for the body	
    if NODE_ID == 500:
    	NODE_ID += 1
    
    ### node 1000 for the mind	
    if NODE_ID == 1000:
    	NODE_ID += 1
    
    new_node = dict()
    new_node["id"] = '{}'.format(NODE_ID)
    new_node["name"] = name
    new_node["type"] = type
    new_node["attribute"] = {}
    new_node["links"] = []

    new_node["attribute"]["pos_x"] = {"type": 2, "value": random.randint(-200, 200)}
    new_node["attribute"]["pos_y"] = {"type": 2, "value": random.randint(-200, 200)}
    new_node["attribute"]["color"] = {"type": 0, "value": get_color_type(new_node["type"])}

    for name, value in others:
        if name == "parent":
            type = 7
            val = '{}'.format(value)
        else:
            type, val = type_to_integer(value, check_truncate_value(name))
        new_node["attribute"][name] = {"type": type, "value": val}

    return new_node

#################################
### Crear un enlace ###
#################################
def create_edge(src, dst, type, pos, rot):
    new_edge = dict()
    new_edge["src"] = '{}'.format(src)
    new_edge["dst"] = '{}'.format(dst)
    new_edge["label"] = type
    new_edge["linkAttribute"] = {}
    if type == "RT":
        new_edge["linkAttribute"]["rt_translation"] = {"type": 3, "value": pos}
        new_edge["linkAttribute"]["rt_rotation_euler_xyz"] = {"type": 3, "value": rot}
    return new_edge


##################################################################
### Dividir el tamaño en anchura, altura y profundidad ###
##################################################################
# convert size[] to width, height, depth values
def convert_size_to_parameters(type, size):
    if type == "plane" or type == "mesh" or type == "floor":
        return [("width", size[0]), ("height", size[1]), ("depth", size[2])]


##################################################################
### Convertir el color a textura ###
##################################################################
def convert_color_to_texture(type, color):
    if type == "plane":
        value = '#%02x%02x%02x' %(color[0], color[1], color[2])
        new_attrib = [("texture", value)]
    else:
        # TODO: read mesh from coppelia
        new_attrib = [("path", "/home/robocomp/robocomp/files/osgModels/basics/cube2.3ds")]
    return new_attrib

##################################################################
### Insertar el robot en el DSR ###
##################################################################
def open_robot (new_json, robot_file, w_node):
    if os.path.isfile(robot_file):
        with open(robot_file) as json_file:
                robot = json.load(json_file)

                new_edge = create_edge(w_node["id"], 200, "RT", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                new_json["DSRModel"]["symbols"][str(w_node["id"])]["links"].append(new_edge)
                # merge dictionaries
                new_json["DSRModel"]["symbols"] = {**(new_json["DSRModel"]["symbols"]), **(robot["DSRModel"]["symbols"])}

#####################################################################################################################################
### Proceso de transformación de los datos de los objetos en XML en nodos para el DSR ###
#####################################################################################################################################
def transform_to_json(elem, new_json, directory_size, directory_color, w_node):
    name = elem.findall('common/name')[0].text
    type = convert_vreptype_to_jsontype(name)
    pos = [float(value)*1000 for value in elem.findall('common/localFrame/position')[0].text.split(' ')]
    rot = [float(value)*3.14159/180 for value in elem.findall('common/localFrame/euler')[0].text.split(' ')]
    size = [float(value)*1000 for value in elem.findall(directory_size)[0].text.split(' ')]
    color = [int(value) for value in elem.findall(directory_color)[0].text.split(' ')]
        

            # first try
    if type:
        print(name, pos, rot, type, size, color)
        others = convert_size_to_parameters(type, size)
        others += [("texture", "#999999")] if "floor" in name.lower() else convert_color_to_texture(type, color)
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
    	    new_attribute["value"] = '{}'.format(new_edge["src"])
    	    new_attribute["type"] = 7
    	    new_json["DSRModel"]["symbols"][str(dst)]["attribute"]["parent"] = new_attribute
        if "level" not in new_json["DSRModel"]["symbols"][str(dst)]:
    	    new_attribute = dict()
    	    new_attribute["value"] = new_json["DSRModel"]["symbols"][str(src)]["attribute"]["level"][
                                                 "value"] + 1
    	    new_attribute["type"] = 1
    	    new_json["DSRModel"]["symbols"][str(dst)]["attribute"]["level"] = new_attribute

        new_json["DSRModel"]["symbols"][str(src)]["links"].append(new_edge)
    return new_json

#####################################################################################################################################
### Recorrer el árbol xml e ir transformando los objetos detectados en nodos dependiendo si son formas puras o no puras ###
#####################################################################################################################################
def convert_xml_to_json(root, new_json, directory, w_node):

    for elem in root.findall(directory):
        # Get Robot
        name = elem.findall('common/name')[0].text
        name_robot = dictRobots[name] if name in dictRobots.keys() else ""
    
        if name_robot != "":
    	    print('Adding {} robot'.format(name_robot))
    	    robot_file = '/home/robocomp/robocomp/components/dsr-graph/etc/Robot_{}.json'.format(name_robot)
    	    open_robot(new_json, robot_file, w_node)
        
    
        # check required tags           
        if len(elem.findall('common/localFrame/position')) and len(elem.findall('primitive/type')):
            new_json = transform_to_json(elem, new_json, 'primitive/size', 'primitive/color/ambientDiffuse', w_node)
            
            
        else:
            if len(elem.findall('common/localFrame/position')) and len(elem.findall('shape/primitive/type')):
                new_json = transform_to_json(elem, new_json, 'shape/primitive/size', 'shape/primitive/color/ambientDiffuse', w_node) 

    return new_json

######################################################################################################
### Main ### Creación json, nodo mundo y detección de objetos dentro de dummies y fuera de ellos.
######################################################################################################
def main(input_file, output_file):

    new_json = {}
    new_json["DSRModel"] = {}
    new_json["DSRModel"]["symbols"] = {}


    tree = ET.parse(input_file)
    root = tree.getroot()

    # insert world node
    # TODO: cambiar los valores de los outerregion por los del suelo ( o unos valores aproximados razonables)
    w_node = create_node("world", "world", [("parent", 0), ("level", 0), ("OuterRegionBottom", -7500), ("OuterRegionLeft", -7500), ("OuterRegionRight", 7500), ("OuterRegionTop", 7500)])
    new_json["DSRModel"]["symbols"][str(w_node["id"])] = w_node

    print("\nElements found:")
    if(root.findall('shape')):
        new_json = convert_xml_to_json(root, new_json,'shape', w_node)


    print("\nElements found in DUMMIES:")
    if(root.findall('dummy/shape')):
        new_json = convert_xml_to_json(root, new_json, 'dummy/shape', w_node)


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
