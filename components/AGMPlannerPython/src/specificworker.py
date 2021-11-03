#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by Fernando Martín Ramos-Catalina
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

from rich.console import Console
from genericworker import *
import subprocess

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *
import sys
sys.path.append('/usr/local/share/agm')
from agglplanner import *
import os

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 45
        self.output_file_count=0
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)
        self.rt_api = rt_api(self.g)

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            #signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        console.print('SpecificWorker destructor')

    def setParams(self, params):

        return True


    @QtCore.Slot()
    def compute(self):
      
        print('Writing JSON')
        file = "AGMPlannerPython" + "_" + '{}'.format(self.output_file_count) + ".json"
        self.output_file_count = self.output_file_count +1
        self.g.write_to_json_file(file)
        print(file)

 #      if not(self.g.get_node('Planificador')):
#            print('holiiiiiiiiiiiiiiiiiiiiiiii')
#            new_node = Node(agent_id=self.agent_id, type='planner', name='Planificador')
#            id_mind = self.g.get_node('mind').id
#            vector=["Holo", "holi", "holuuuu"]
#            new_node.attrs['parent'] = Attribute(id_mind, self.agent_id)
#            new_node.attrs['plan_list'] = Attribute(vector, self.agent_id)
#            new_node.attrs['pos_x'] = Attribute(-239.0, self.agent_id)
#            new_node.attrs['pos_y'] = Attribute(50.0, self.agent_id)

#            try:
#                id_result = self.g.insert_node(new_node)
#                #self.rt_api.insert_or_assign_edge_RT(self.g.get_node('mind'), id_result, [0, 0, 0], [.0, 0, .0])
#                id_mind = self.g.get_node('mind').id
#                has_edge = Edge(id_result, id_mind, 'has', self.agent_id)
#                self.g.insert_or_assign_edge(has_edge)
#                print(' inserted new node  ', id_result)

#            except:
#                traceback.print_exc()
#                print('cant update node or add edge RT')
#        else:
#            print('adioooooooooooooooooooooooh')

        if self.g.get_node('mind'):
            print('EXISTE MENTE')
            if not (self.g.get_node('current_intention')):
                # Final Version of the code
                pythonarg = "python3 "  # python3
                agglplanarg = "../../../../../software/AGM/AGGLPlanner/agglplan.py "  # the command to call the .py file
                agglfile = "input_files/aggl_files/change_roomViriato2.aggl "  # ruleset.aggl file
                initfile = file + " "  # initialworld.xml/.json file
                aggtgoal = "input_files/aggt_files/targetGoToKitchen_Viriato.aggt "  # goalfile.aggt file
                resultplan = "input_files/plan_files/resultadoViriato.plan"  # resultplan.plan file
                # print the full chain before call the command line
                cadenatotal = pythonarg + agglplanarg + agglfile + initfile + aggtgoal + resultplan
                print("cadenatotal =")
                print(cadenatotal)

                os.system(cadenatotal)

                if os.stat(resultplan).st_size == 0:
                    print('ESTA VACIOOOOOOOOOOOO')
                else:
                    with open(resultplan) as f:
                        lines = f.read()
                        first = lines.split('\n', 1)[0]
                    print('PLAN: ', first)

                    print('se crea nodo intención')
                    new_node = Node(agent_id=self.agent_id, type='intention', name='current_intention')
                    id_mind = self.g.get_node('mind').id
                    # id_level = self.g.get_node('mind').level
                    # print(id_level)
                    new_node.attrs['parent'] = Attribute(id_mind, self.agent_id)
                    new_node.attrs['level'] = Attribute(3, self.agent_id)
                    new_node.attrs['pos_x'] = Attribute(-200.0, self.agent_id)
                    new_node.attrs['pos_y'] = Attribute(50.0, self.agent_id)
                    new_node.attrs['current_intention'] = Attribute(first, self.agent_id)

                    try:
                        id_result = self.g.insert_node(new_node)
                        # self.rt_api.insert_or_assign_edge_RT(self.g.get_node('mind'), id_result, [0, 0, 0], [.0, 0, .0])
                        id_mind = self.g.get_node('mind').id
                        has_edge = Edge(id_result, id_mind, 'has', self.agent_id)
                        self.g.insert_or_assign_edge(has_edge)
                        print(' inserted new node  ', id_result)

                    except:
                        traceback.print_exc()
                        print('cant update node or add edge RT')





            else:
                print('chaooooooo')

        os.remove(file)

            


#        plans = []
#        f = open("input_files/plan_files/resultadoViriato.plan", "r")
#        while (True):
#            linea = f.readline()
#            if not('#' in linea):
#                print(linea)
#                plans.append(linea)
#            if not linea:
#                break
#        f.close()
#        print('SEPARASIO')
#        for nom in peliculas:
#            print(nom)




        #pass
        return True

    def startup_check(self):
        from PySide2.QtCore import QTimer
        from PySide2.QtWidgets import QApplication
        QTimer.singleShot(200, QApplication.instance().quit)


  # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of AGGlplannerexecution method from AGGLPlanner interface
    #You can find it in robocomp/interfaces/IDSLs/AGGLPlanner.idsl
    
    #Entries: Arguments contain all the required arguments to execute the command. It is a struct. 
    #Exit: Always return true. I have make it like this because it can be usefull to future developments.
    #Functionality: Implements the AGGLPlanner interface. This interface has the objective of calling the AGM scheduler 
    #with the arguments received in "arguments"
    
    
    def AGGLPlanner_AGGlplannerexecution(self, arguments):
    
        ret = bool()
        ret = True
       
        #Test arguments. This choose a not dinamic parameters. Only used for check the properly execution of the code.
        
        #pythonarg = "python3 "
        #agglplanarg = "../../../../AGM/AGGLPlanner/agglplan.py "
        #agglfile ="examples/logistics/domain.aggl "
        #initfile="examples/logistics/init0.xml "
        #aggtgoal="examples/logistics/prueba0.aggt "
        #resultplan="../AGMPlanner/plandelagentepython.plan "
        #cadenatotal = pythonarg+agglplanarg +agglfile + initfile +aggtgoal +resultplan
        #print("cadenatotal =")
        #print(cadenatotal)
        
        #Final of Test arguments
        
        
        
        
        #Final Version of the code
        pythonarg = arguments.pythonarg # python3
        agglplanarg = arguments.agglplanarg # the command to call the .py file
        agglfile = arguments.agglfile # ruleset.aggl file
        initfile = arguments.initfile # initialworld.xml/.json file
        aggtgoal = arguments.aggtgoal # goalfile.aggt file
        resultplan = arguments.resultplan # resultplan.plan file
        #print the full chain before call the command line
        cadenatotal = pythonarg+agglplanarg +agglfile + initfile +aggtgoal +resultplan 
        print("cadenatotal =")
        print(cadenatotal)
        
        os.system(cadenatotal)
        
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompAGGLPlanner you can use this types:
    # RoboCompAGGLPlanner.Parameters




    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
