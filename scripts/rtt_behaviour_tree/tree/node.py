import uuid

from utils import vector2
from tree import node_types

class Node:

    def __init__(self):
        self._id = uuid.uuid1()
        self.type = None
        self.title = ""
        self.description = ""
        self.display = vector2.Vector2()
        self.properties = {}
        self._children = []

    def id(self):
        return self._id

    def load_from_json(self, data):
        self._id = uuid.UUID(data['id'])

        self.type = node_types.node_types[data['name']]
        self.title = data['title']
        self.description = data['description']

        self.display.x = data['display']['x']
        self.display.y = data['display']['y']


    def children(self):
        return self._children

    def add_child(self, child):
        self._children.append(child)
