import sys

import rospkg

from python_qt_binding import QtWidgets, QtGui

from tree import tree_list
from widgets import pan_graphics_view
from items import tree_graphics_item


TREE_DIR = rospkg.RosPack().get_path('roboteam_tactics') + "/src/trees/json/"


class MainWidget(QtWidgets.QWidget):

    def __init__(self):
        super(QtWidgets.QWidget, self).__init__()

        self.selected_tree = None

        # Read the tree name from the command line arguments.
        if len(sys.argv) > 1:
            self.selected_tree = sys.argv[1]
            print self.selected_tree

        self._trees = tree_list.TreeList()

        self.init_ui()


        self._trees.load_trees(TREE_DIR)
        self.init_list_widget()

        default_tree = None
        if self.selected_tree:
            default_tree = self._trees.get_tree(self.selected_tree)

        self._tree_item = tree_graphics_item.TreeGraphicsItem(default_tree)
        self.scene.addItem(self._tree_item)


    def init_ui(self):

        self.resize(500, 500)
        self.setWindowTitle("rtt behaviour tree")

        self.setLayout(QtWidgets.QHBoxLayout())

        # ---- Tree view ----

        self.scene = QtWidgets.QGraphicsScene()
        self.view = pan_graphics_view.PanGraphicsView()
        self.view.setScene(self.scene)
        # Enable antialiasing
        self.view.setRenderHints(QtGui.QPainter.Antialiasing)
        self.layout().addWidget(self.view)

        self.tree_item = None

        # ---- Tree view ----

        # ---- Tree list ----

        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.setSizePolicy(self.list_widget.sizePolicy().Preferred, self.list_widget.sizePolicy().Preferred)
        self.layout().addWidget(self.list_widget)

        self.list_widget.currentTextChanged.connect(self.list_select_tree)

        # ---- Tree list ----

        self.show()


    def init_list_widget(self):
        # TODO: Don't make double entries. Also remove old ones.
        for title in self._trees.title_list():
            self.list_widget.addItem(title)


    # ---- Slots ----------

    def list_select_tree(self, title):
        self.selected_tree = title

        self.scene.removeItem(self._tree_item)

        new_tree = self._trees.get_tree(self.selected_tree)

        self._tree_item = tree_graphics_item.TreeGraphicsItem(new_tree)
        self.scene.addItem(self._tree_item)

        # Shrink the scene to fit the new tree when is smaller.
        self.scene.setSceneRect(self.scene.itemsBoundingRect())
