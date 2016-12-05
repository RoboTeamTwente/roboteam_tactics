from python_qt_binding import QtWidgets, QtGui, QtCore

from utils import vector2


class NodeGraphicsItem(QtWidgets.QGraphicsItem):

    def __init__(self, node_reference):
        """
        The `node_reference` should be a reference to a `tree.Node` object.
        """
        super(QtWidgets.QGraphicsItem, self).__init__()

        if node_reference:

            self._node = node_reference

            self.node_body = QtWidgets.QGraphicsItemGroup(parent=self)
            # Make the body dragable.
            self.node_body.setFlag(QtWidgets.QGraphicsItemGroup.ItemIsMovable, True)

            self.size = vector2.Vector2(100, 30)

            if self._node.type.category.name == 'condition':
                self.body = QtWidgets.QGraphicsEllipseItem(0, 0, self.size.x, self.size.y)
            else:
                self.body = QtWidgets.QGraphicsRectItem(0, 0, self.size.x, self.size.y)


            self.node_body.addToGroup(self.body)
            self.text = QtWidgets.QGraphicsTextItem("")
            self.node_body.addToGroup(self.text)

            self.connectors = []

            self.update()


    def update(self):

        self.setPos(self._node.display.x, self._node.display.y)

        text_string = self._node.title

        # See if the node has an overriding custom icon.
        if self._node.type.has_custom_icon():
            text_string = self._node.type.custom_icon

        self.text.setPlainText(text_string)

        self.size.x = self.text.boundingRect().width()
        self.size.y = self.text.boundingRect().height()

        self.body.setRect(0, -self.size.y/2, self.size.x, self.size.y)
        self.text.setPos(0, -self.size.y/2)

        for child in self._node.children():
            path = QtGui.QPainterPath(QtCore.QPointF(
                self.size.x, 0))
            path.lineTo(child.display.x - self._node.display.x, child.display.y - self._node.display.y)

            connector = QtWidgets.QGraphicsPathItem(path)
            connector.setParentItem(self)
            self.connectors.append(connector)


    def boundingRect(self):
        return self.body.boundingRect()

    def paint(self, painter, option, widget):
        pass
