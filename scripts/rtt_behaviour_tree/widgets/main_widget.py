import sys

from python_qt_binding import QtWidgets, QtGui

from tree import tree_list
from widgets import pan_graphics_view
from items import tree_graphics_item


TREE_DIR = "src/roboteam_tactics/src/trees/json/"


class MainWidget(QtWidgets.QWidget):

    def __init__(self):
        super(QtWidgets.QWidget, self).__init__()

        default_tree = 'BasicKeeperTree'

        # Read the tree name from the command line arguments.
        if len(sys.argv) > 1:
            default_tree = sys.argv[1]
            print default_tree

        self._trees = tree_list.TreeList()

        self.init_ui()

        self._trees.load_trees(TREE_DIR)

        self._tree_item = tree_graphics_item.TreeGraphicsItem(self._trees.get_tree(default_tree))
        self.scene.addItem(self._tree_item)

        # with open(TREE_FOLDER + "CoolRole.json") as data_file:
        #     data = json.load(data_file)
        #     self._tree.from_json(data)


    def init_ui(self):

        self.resize(500, 500)
        self.setWindowTitle("rtt behaviour tree")

        self.setLayout(QtWidgets.QVBoxLayout())

        self.b_add_node = QtWidgets.QPushButton("Add node")
        self.layout().addWidget(self.b_add_node)
        self.b_add_node.clicked.connect(self.slot_add_node)

        # ---- Tree view ----

        self.scene = QtWidgets.QGraphicsScene()
        self.view = pan_graphics_view.PanGraphicsView()
        self.view.setScene(self.scene)
        # Enable antialiasing
        self.view.setRenderHints(QtGui.QPainter.Antialiasing)
        self.layout().addWidget(self.view)

        self.tree_item = None

        # ---- Tree view ----

        self.show()


    # ---- Button slots ----------

    def slot_add_node(self):
        pass
