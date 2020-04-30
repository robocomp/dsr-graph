import json

class ApiAssertionsMixin(object):

    def assertJsonEqual(self, first, second, msg=None):
        j1 = json.dumps(first, sort_keys=True, indent=4)
        j2 = json.dumps(second, sort_keys=True, indent=4)
        self.maxDiff = None
        with open('last_json1.txt', 'w') as outfile:
            json.dump(first, outfile, sort_keys=True, indent=4)
        with open('last_json2.txt', 'w') as outfile:
            json.dump(second, outfile, sort_keys=True, indent=4)
        self.assertEqual(j1, j2, msg)