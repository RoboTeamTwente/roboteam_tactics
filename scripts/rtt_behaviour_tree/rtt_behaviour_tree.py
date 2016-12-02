#! /usr/bin/env python2

import sys
import os.path
from python_qt_binding import QtWidgets

from widgets import main_widget


def main():
    # Check that we are executing inside a catkin workspace.
    if not os.path.isfile('.catkin_workspace'):
        print "Please execute me in the roboteam catkin workspace."
        sys.exit(1)

    widget = main_widget.MainWidget()
    sys.exit(app.exec_())


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    main()
