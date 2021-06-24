<<<<<<< HEAD
import copy
import json
import multiprocessing
import os
import signal
import subprocess
import time
import unittest
from assertions import ApiAssertionsMixin

os.environ['TERM'] = 'xterm'



class KillableCommand:
    def __init__(self, command):
        self._command = command
        self._exit = False

    def run_in_thread(self):
        environ=os.environ.copy()
        environ['PATH'] = "/opt/robocomp/bin:" + environ["PATH"]
        command_output = subprocess.Popen(self._command,
                                          stdout=subprocess.PIPE,
                                          stderr=subprocess.STDOUT, env=environ,
                                          shell=True,
                                          universal_newlines=True)

        for stdout_line in iter(command_output.stdout.readline, ""):
            print("%s running: %s"%(self._command.split(';')[-1], stdout_line))
            if self._exit:
                print("Killing %s" % self._command)
                command_output.kill()
                break
        print("Command finished. Waiting.")
        
        command_output.stdout.close()

        return_code = command_output.wait()
        if return_code:
            print("%s returned ERROR %s" % (self._command, return_code))
            return return_code
        else:
            print("%s finished OK" % self._command)
            return return_code

    def do_exit(self):
        self._exit = True
=======
import json
import os
import subprocess
import threading
import unittest
from test.assertions import ApiAssertionsMixin

os.environ['TERM'] = 'xterm'

COMMAND_1= "cd ../components/crdt_rtps_dsr_strings & ls"
COMMAND_2= "cd ../components/crdt_rtps_dsr_strings & ls"
JSON_1 = "agent0_dsr (2).json"
JSON_2 = "agent1_dsr (2).json"


def run_command_in_thread(command):
    environ=os.environ.copy()
    environ['PATH'] = "/opt/robocomp/bin:" + environ["PATH"]
    command_output = subprocess.Popen(command,
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, env=environ,
                                      shell=True,
                                      universal_newlines=True)
    for stdout_line in iter(command_output.stdout.readline, ""):
        print(stdout_line)
    command_output.stdout.close()
    return_code = command_output.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, command)
    else:
        return return_code
>>>>>>> 9c91cabed26ae695de398067db2c697fcc79c1d6


class ExternalToolsTester(ApiAssertionsMixin, unittest.TestCase):

<<<<<<< HEAD
    def test_attributes(self):
        COMMAND_1 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_attribute_test0"
        COMMAND_2 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_attribute_test1"
        JSON_1 = "../etc/change_attribute_agent0_dsr.json"
        JSON_2 = "../etc/change_attribute_agent1_dsr.json"
        self.rename_previous_files([JSON_1, JSON_2])
        idserver = self.launch_commands([COMMAND_1, COMMAND_2])
        self.open_and_compare_jsons(JSON_1, JSON_2)
        self.finish_all(idserver)

    def rename_previous_files(self, files):
        assert isinstance(files, list), 'files parameter must be a list of paths to files'
        timestamp = int(time.time())
        for file in files:
            if os.path.exists(file):
                os.rename(file, file+".back.%d" % timestamp)

    def test_conflict(self):
        COMMAND_1 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_conflict_test0"
        COMMAND_2 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_conflict_test1"
        JSON_1 = "../etc/conflict_resolution_agent0_dsr.json"
        JSON_2 = "../etc/conflict_resolution_agent1_dsr.json"
        self.rename_previous_files([JSON_1, JSON_2])
        idserver = self.launch_commands([COMMAND_1, COMMAND_2])
        self.open_and_compare_jsons(JSON_1, JSON_2)
        self.finish_all(idserver)

    def test_node(self):
        COMMAND_1 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_node_test0"
        COMMAND_2 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_node_test1"
        JSON_1 = "../etc/insert_remove_node_agent0_dsr.json"
        JSON_2 = "../etc/insert_remove_node_agent1_dsr.json"
        self.rename_previous_files([JSON_1, JSON_2])
        idserver = self.launch_commands([COMMAND_1, COMMAND_2])
        self.open_and_compare_jsons(JSON_1, JSON_2)
        self.finish_all(idserver)
        
    def test_edge(self):
        COMMAND_1 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_edge_test0"
        COMMAND_2 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_edge_test1"
        JSON_1 = "../etc/insert_remove_edge_agent0_dsr.json"
        JSON_2 = "../etc/insert_remove_edge_agent1_dsr.json"
        self.rename_previous_files([JSON_1, JSON_2])
        idserver = self.launch_commands([COMMAND_1, COMMAND_2])
        self.open_and_compare_jsons(JSON_1, JSON_2)
        self.finish_all(idserver)

    def test_concurrent(self):
        COMMAND_1 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_concurrent_test0"
        COMMAND_2 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_concurrent_test1"
        JSON_1 = "../etc/concurrent_operations_agent0_dsr.json"
        JSON_2 = "../etc/concurrent_operations_agent1_dsr.json"
        self.rename_previous_files([JSON_1, JSON_2])
        idserver = self.launch_commands([COMMAND_1, COMMAND_2])
        self.open_and_compare_jsons(JSON_1, JSON_2)
        self.finish_all(idserver)


    def test_delayed_start(self):
        COMMAND_1 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_delayed_test0"
        COMMAND_2 = "cd ../components/crdt_rtps_dsr_tests; ./bin/crdt_rtps_dsr_tests etc/config_delayed_test1"
        JSON_1 = "../etc/delayed_start_agent0_dsr.json"
        JSON_2 = "../etc/delayed_start_agent1_dsr.json"
        self.rename_previous_files([JSON_1, JSON_2])
        idserver = self.launch_commands([COMMAND_1, COMMAND_2], delay=6)
        self.open_and_compare_jsons(JSON_1, JSON_2)
        self.finish_all(idserver)

    def finish_all(self, idserver):
        idserver.terminate()
        check_kill_process("idserver")
        check_kill_process("crdt_rtps_dsr_tests")

    def open_and_compare_jsons(self, json1_path, json2_path):
        self.maxDiff = None
        # Compare jsons
        with open(json1_path, "r") as read_file1, open(json2_path, 'r') as read_file2:
=======
    def test_command2(self):
        comp1 = threading.Thread(target=run_command_in_thread,
                                 args=[COMMAND_1])
        comp2 = threading.Thread(target=run_command_in_thread,
                                 args=[COMMAND_2])
        for comp in [comp1, comp2]:
            comp.start()
        for comp in [comp1, comp2]:
            comp.join()
        # Compare jsons
        with open(JSON_1, "r") as read_file1, open(JSON_2, 'r') as read_file2:
>>>>>>> 9c91cabed26ae695de398067db2c697fcc79c1d6
            data1 = json.load(read_file1)
            data1 = json_to_struct(data1)
            data2 = json.load(read_file2)
            data2 = json_to_struct(data2)
            self.assertJsonEqual(data1, data2)

<<<<<<< HEAD
    def launch_commands(self, commands, delay=0):
        idserver_command = KillableCommand("cd ../components/idserver_nogui; ./bin/idserver etc/config")
        idserver = multiprocessing.Process(target=idserver_command.run_in_thread)
        jobs = []
        for command in commands:
            comp_command = KillableCommand(command)
            comp1 = multiprocessing.Process(target=comp_command.run_in_thread)
            jobs.append(comp1)
        idserver.start()
        for comp in jobs:
            comp.start()
            if comp != jobs[-1]:
                time.sleep(delay)
            time.sleep(0.2)
        for comp in jobs:
            comp.join()

        return idserver

def check_kill_process(pstring):
    for line in os.popen("ps ax | grep " + pstring + " | grep -v grep"):
        fields = line.split()
        pid = fields[0]
        os.kill(int(pid), signal.SIGKILL)

    while len([line for line in os.popen("ps ax | grep " + pstring + " | grep -v grep")]) > 0:
        time.sleep(2)

def json_to_struct(json_data):
    new_struct = copy.deepcopy(json_data)
    for symbol_id, symbol in new_struct['DSRModel']['symbols'].items():
        if 'links' in symbol:
            new_links = {}
            for link in symbol['links']:
                new_id = '_'.join([str(link['src']), str(link['dst']), link['label']])
                new_links[new_id] = link
            del new_struct['DSRModel']['symbols'][symbol_id]['links']
            new_struct['DSRModel']['symbols'][symbol_id]['links'] = new_links

    return new_struct


=======

def json_to_struct(json_data):
    new_struct = {'symbol':{}, 'link':{}}
    for link in json_data['DSRModel']['link']:
        new_id = '_'.join([link['src'], link['dst'], link['label']])
        new_struct['link'][new_id] = link
    for symbol in json_data['DSRModel']['symbol']:
        new_struct['symbol'][symbol['id']] = symbol
    return {'DSRModel': new_struct}
>>>>>>> 9c91cabed26ae695de398067db2c697fcc79c1d6

