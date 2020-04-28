import json

class ApiAssertionsMixin(object):

    def assertJsonEqual(self, first, second, msg=None):
        j1 = json.dumps(first, sort_keys=True, indent=4)
        j2 = json.dumps(second, sort_keys=True, indent=4)
        self.maxDiff = None
        self.assertEqual(j1, j2, msg)