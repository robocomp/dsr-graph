import sys
import time

sys.path.append("/opt/robocomp/lib/")
from pydsr import DSRGraph


class DSRInspector:
    def __init__(self):
        self.graph = DSRGraph(int(0), "Prueba", int(12), "autonomyLab_complete.simscene.json", True)

    def recursive_print_nodes(self, node=None, indent="", node_attributes=True, edge_attributes=True):
        if node is None:
            node = self.graph.get_node_root()
        if indent:
            print(f"{indent}└--{node.name}")
        else:
            print(f"{node.name}")
        if node_attributes:
            for attribute_name, attribute_value in node.attrs.items():
                print(f"{indent}   · {attribute_name} = {attribute_value}")
        for edge_info, edge in node.edges.items():
            new_node = self.graph.get_node(edge_info[0])
            if edge_attributes:
                print(f"{indent} {edge.type} [{','.join(node.attrs)}]")
            if new_node:
                self.recursive_print_nodes(new_node, indent + "  ")



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    inspector = DSRInspector()
    inspector.recursive_print_nodes()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
