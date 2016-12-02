import uuid

import tree.node
from utils import vector2


class BehaviourTree:

    def __init__(self):

        self.title = ""
        self.description=  ""
        # Points to the first node on the tree.
        self._root = None
        self.display = vector2.Vector2()

        self.properties = {}

        # All the nodes in the tree.
        self._nodes = {}


    def add_node(self):
        node = tree.node.Node()

        self._nodes[node.id()] = node


    def nodes(self):
        return self._nodes


    def load_from_json(self, data):
        self.title = data['title']
        self.description = data['description']
        self._root = uuid.UUID(data['root'])

        self.display.x = data['display']['x']
        self.display.y = data['display']['y']

        # Add the nodes.
        for key, node_data in data['nodes'].iteritems():
            node = tree.node.Node()
            node.load_from_json(node_data)

            self._nodes[node.id()] = node

        # Connect the nodes to each other.
        for key, node_data in data['nodes'].iteritems():
            uid = uuid.UUID(key)
            node = self._nodes[uid]

            if 'children' in node_data:
                for child_key in node_data['children']:
                    child_uid = uuid.UUID(child_key)
                    node.add_child(self._nodes[child_uid])

            if 'child' in node_data:
                child_uid = uuid.UUID(node_data['child'])
                node.add_child(self._nodes[child_uid])
