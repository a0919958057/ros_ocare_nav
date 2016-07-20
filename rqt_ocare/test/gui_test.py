
import sys
from PyQt4 import QtGui
from ocare_ui import Ui_OcareGui


def main():
    app = QtGui.QApplication(sys.argv)
    widget = QtGui.QWidget()
    ui = Ui_OcareGui()
    ui.setupUi(widget)
    widget.show()

    sys.exit(app.exec_())

if __name__=='__main__':
    main()