#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
import itertools
import traceback

from PySide2.QtCore import QTimer, Signal, QPoint, QPointF
from PySide2.QtGui import QPolygon, Qt
from PySide2.QtWidgets import QApplication
from genericworker import *

from PersonalSpacesManager import PersonalSpacesManager

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
from pydsr import *

from rich.console import Console
console = Console(highlight=False)


# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class PersonType:
    def __init__(self, id=None, tx=0, ty=0, tz=0, rx=0, ry=0, rz=0):
        self.id = id
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz


class PersonalSpaceType:
    def __init__(self):
        self.intimate_polyline = []
        self.personal_polyline = []
        self.social_polyline = []


class SpecificWorker(GenericWorker):

    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 1000

        self.agent_id = 121
        self.g = DSRGraph(0, "human_social_spaces_dsr", self.agent_id)
        self.rt_api = rt_api(self.g)

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            print("signals connected")
        except RuntimeError as e:
            print(e)

        self.personal_spaces_manager = PersonalSpacesManager()

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        people_list = self.get_people_from_dsr()
        spaces = self.personal_spaces_manager.get_personal_spaces(people_list, True)
        dict_ids_personal_spaces = self.get_space_of_each_person(people_list, spaces)
        self.update_personal_spaces(dict_ids_personal_spaces)
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def get_people_from_dsr(self):
        # Read and store people from dsr
        people_nodes = self.g.get_nodes_by_type('person')
        people_list = []
        for person_node in people_nodes:
            person_id = person_node.attrs['person_id'].value
            edge_rt = self.rt_api.get_edge_RT(self.g.get_node("world"), person_node.id)
            tx, ty, tz = edge_rt.attrs['rt_translation'].value
            rx, ry, rz = edge_rt.attrs['rt_rotation_euler_xyz'].value
            person_type = PersonType(person_id, tx, ty, tz, rx, ry, rz)
            people_list.append(person_type)

        return people_list

    def get_space_of_each_person(self, people_list, spaces):
        console.print(' ----- get_space_of_each_person -----', style='blue')

        intimate_spaces, personal_spaces, social_spaces = spaces
        dict_ids_personal_spaces = {}

        for person in people_list:
            person_pose = QPoint(person.tx, person.ty)
            personal_space = PersonalSpaceType()

            for intimate in intimate_spaces:
                intimate_polygon = QPolygon()
                for point in intimate:
                    intimate_polygon << QPoint(point[0], point[1])

                if intimate_polygon.containsPoint(person_pose, Qt.OddEvenFill):
                    personal_space.intimate_polyline = intimate
                    break

            for personal in personal_spaces:
                personal_polygon = QPolygon()
                for point in personal:
                    personal_polygon << QPoint(point[0], point[1])

                if personal_polygon.containsPoint(person_pose, Qt.OddEvenFill):
                    personal_space.personal_polyline = personal
                    break

            for social in social_spaces:
                social_polygon = QPolygon()
                for point in social:
                    social_polygon << QPoint(point[0], point[1])

                if social_polygon.containsPoint(person_pose, Qt.OddEvenFill):
                    personal_space.social_polyline = social
                    break

            dict_ids_personal_spaces[person.id] = personal_space

        return dict_ids_personal_spaces

    def update_personal_spaces(self, dict_ids_personal_spaces):
        console.print(' ----- update_personal_space -----', style='blue')

        people_nodes = self.g.get_nodes_by_type('person')

        for person_node in people_nodes:
            person_id = person_node.attrs['person_id'].value

            x_intimate, y_intimate = zip(* dict_ids_personal_spaces[person_id].intimate_polyline)
            x_personal, y_personal = zip(* dict_ids_personal_spaces[person_id].personal_polyline)
            x_social, y_social = zip(* dict_ids_personal_spaces[person_id].social_polyline)
            print(type(x_intimate), x_intimate)
            print(type(list(x_intimate)), list(x_intimate))

            personal_space_node = None

            for destination, type_edge in person_node.edges:
                print(destination, type_edge)
                dest_node = self.g.get_node(destination)
                if type_edge == 'has' and dest_node.type == 'personal_space':
                    personal_space_node = dest_node
                    break

            # Update personal_space node
            if personal_space_node is not None:
                console.print('Updating personal space', style='red')
                personal_space_node = self.g.get_node('personal_space_0')

                personal_space_node.attrs['ps_social_x_pos'].value = list(x_social)
                personal_space_node.attrs['ps_social_y_pos'].value = list(y_social)
                personal_space_node.attrs['ps_personal_x_pos'].value = list(x_personal)
                personal_space_node.attrs['ps_personal_y_pos'].value = list(y_personal)
                personal_space_node.attrs['ps_intimate_x_pos'].value = list(x_intimate)
                personal_space_node.attrs['ps_intimate_y_pos'].value = list(y_intimate)

                try:
                    self.g.update_node(personal_space_node)
                except:
                    traceback.print_exc()
                    print('cant update node')

            # Create personal_space node
            else:
                print('Create personal_space node')
                node_name = 'personal_space_' + str(person_id)

                new_node = Node(agent_id=self.agent_id, type='personal_space', name=node_name)

                new_node.attrs['ps_social_x_pos'] = Attribute(list(x_social), self.agent_id)
                new_node.attrs['ps_social_y_pos'] = Attribute(list(y_social), self.agent_id)
                new_node.attrs['ps_personal_x_pos'] = Attribute(list(x_personal), self.agent_id)
                new_node.attrs['ps_personal_y_pos'] = Attribute(list(y_personal), self.agent_id)
                new_node.attrs['ps_intimate_x_pos'] = Attribute(list(x_intimate), self.agent_id)
                new_node.attrs['ps_intimate_y_pos'] = Attribute(list(y_intimate), self.agent_id)
                # new_node.attrs['ps_shared_with'] = Attribute(list([],) self.agent_id)

                try:
                    id_result = self.g.insert_node(new_node)
                    console.print('Personal space node created -- ', id_result, style='red')
                    has_edge = Edge(id_result, person_node.id, 'has', self.agent_id)
                    self.g.insert_or_assign_edge(has_edge)

                    print(' inserted new node  ', id_result)

                except:
                    traceback.print_exc()
                    print('cant insert node or add edge RT')

    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to newPeopleData method from HumanToDSRPub interface
    #
    def HumanToDSRPub_newPeopleData(self, people):
        print('HumanToDSRPub_newPeopleData ------')

        people_list = people.peoplelist
        people_nodes = self.g.get_nodes_by_type('person')

        for person in people_list:
            person_node_in_dsr = None

            for p_node in people_nodes:
                if p_node.attrs['person_id'].value == person.id:
                    person_node_in_dsr = p_node
                    break

            # Update Node
            if person_node_in_dsr is not None:

                try:
                    self.rt_api.insert_or_assign_edge_RT(self.g.get_node('world'), person_node_in_dsr.id,
                                                         [person.x, person.y, person.z], [.0, person.ry, .0])
                except:
                    traceback.print_exc()
                    print('Cant update RT edge')

                # print(self.rt_api.get_edge_RT(self.g.get_node("world"), person_node_in_dsr.id))

            # Create Node
            else:

                node_name = 'person_' + str(person.id)
                new_node = Node(agent_id=self.agent_id, type='person', name=node_name)
                new_node.attrs['person_id'] = Attribute(person.id, self.agent_id)

                try:
                    id_result = self.g.insert_node(new_node)
                    self.rt_api.insert_or_assign_edge_RT(self.g.get_node('world'), id_result,
                                                         [person.x, person.y, person.z], [.0, person.ry, .0])
                    print(' inserted new node  ', id_result)

                except:
                    traceback.print_exc()
                    print('cant update node or add edge RT')

    # ===================================================================
    # ===================================================================

    # =============== DSR Methods  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        console.print(f"UPDATE EDGE: {fr} to {to} {type}", style='green')

    def update_edge_att(self, fr: int, to: int, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {to} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {to} {type}", style='green')

    ######################
    # From the RoboCompHumanToDSRPub you can use this types:
    # RoboCompHumanToDSRPub.TJointData
    # RoboCompHumanToDSRPub.Person
    # RoboCompHumanToDSRPub.PeopleData
