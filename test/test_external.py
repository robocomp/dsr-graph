import json
import os
import time
import subprocess
import threading
import unittest
from test.assertions import ApiAssertionsMixin

os.environ['TERM'] = 'xterm'

COMMAND_1= "cd /home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests && bin/crdt_rtps_dsr_tests etc/config_attribute_test0"
COMMAND_2= "cd /home/robocomp/robocomp/components/dsr-graph/components/crdt_rtps_dsr_tests && bin/crdt_rtps_dsr_tests etc/config_attribute_test1"
JSON_1 = "/home/robocomp/robocomp/components/dsr-graph/etc/agent0_dsr.json"
JSON_2 = "/home/robocomp/robocomp/components/dsr-graph/etc/agent1_dsr.json"


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
            time.sleep(0.2)
        for comp in [comp1, comp2]:
            comp.join()
        # Compare jsons
        with open(JSON_1, "r") as read_file1, open(JSON_2, 'r') as read_file2:
            data1 = json.load(read_file1)
            data2 = json.load(read_file2)
            self.assertJsonEqual(data1, data2)

