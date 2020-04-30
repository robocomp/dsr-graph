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


class ExternalToolsTester(ApiAssertionsMixin, unittest.TestCase):

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
            data1 = json.load(read_file1)
            data1 = json_to_struct(data1)
            data2 = json.load(read_file2)
            data2 = json_to_struct(data2)
            self.assertJsonEqual(data1, data2)


def json_to_struct(json_data):
    new_struct = {'symbol':{}, 'link':{}}
    for link in json_data['DSRModel']['link']:
        new_id = '_'.join([link['src'], link['dst'], link['label']])
        new_struct['link'][new_id] = link
    for symbol in json_data['DSRModel']['symbol']:
        new_struct['symbol'][symbol['id']] = symbol
    return {'DSRModel': new_struct}

