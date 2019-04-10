import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from UAV_GUI import Ui_MainWindow

class Editor(QtWidgets.QMainWindow):

    def __init__(self):
        super(Editor, self).__init__()
        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)
        self.show()

def main():
    app = QtWidgets.QApplication(sys.argv)
    ex = Editor()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

    