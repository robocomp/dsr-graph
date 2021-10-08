import json
import re

def attt(mo):
    array_values = " ".join([line.strip() for line in mo.group(2).splitlines()])
    new_string = mo.group(1)+array_values+mo.group(3)
    return new_string


class ApiAssertionsMixin(object):

    def assertJsonEqual(self, first, second, msg=None):
        j1 = json.dumps(first, sort_keys=True, indent=4)
        j2 = json.dumps(second, sort_keys=True, indent=4)
        expresion = r'(\"value\": \[)((?:(?:\s*[-+]?[0-9]*\.?[0-9]*)\,)*(?:\s*[-+]?[0-9]*\.?[0-9]*)\s*)(])'
        j1 = re.sub(expresion, attt, j1)
        j2 = re.sub(expresion, attt, j2)
        with open('last_json1.txt', 'w') as outfile:
            outfile.write(j1)
        with open('last_json2.txt', 'w') as outfile:
            outfile.write(j2)
        self.assertEqual(j1, j2, msg)
