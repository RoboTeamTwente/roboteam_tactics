from python_qt_binding import QtWidgets

from items import node_graphics_item

class TreeGraphicsItem(QtWidgets.QGraphicsItem):

    def __init__(self, tree_reference=None):
        """
        The `node_reference` should be a reference to a `tree.BehaviourTree` object.
        """
        super(QtWidgets.QGraphicsItem, self).__init__()

        if tree_reference:

            self._tree = tree_reference

            self._nodes = {}

            for uuid, node in self._tree.nodes().iteritems():
                node_graphic = node_graphics_item.NodeGraphicsItem(node)
                node_graphic.setParentItem(self)
                self._nodes[uuid] = node_graphic
