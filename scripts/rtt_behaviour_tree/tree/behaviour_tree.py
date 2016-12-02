import uuid

import tree.node
from utils import vector2


class BehaviourTree:

    def __init__(self):

        self.title = ""
        self._root_node = None

        # All the nodes in the tree.
        self._nodes = {}


    def add_node(self):
        node = tree.node.Node()

        self._nodes[node.id()] = node


    def root(self):
        return self._root_node

    def nodes(self):
        return self._nodes


    def load_from_json(self, data):
        self.title = data['title']

        self._root_node = tree.node.Node()

        self._root_node.title = "O"
        self._root_node.description = data['description']

        self._root_node.display.x = data['display']['x']
        self._root_node.display.y = data['display']['y']

        # Add the nodes.
        for key, node_data in data['nodes'].iteritems():
            node = tree.node.Node()
            node.load_from_json(node_data)

            self._nodes[node.id()] = node

        # Connect the nodes to each other.
        if 'root' in data:
            uid = uuid.UUID(data['root'])
            self._root_node.add_child(self._nodes[uid])

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
