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
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)

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
      

        pass

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
