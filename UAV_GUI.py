import sys

import cv2

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainUI(object):
    def setupUi(self, MainUI):
        MainUI.setObjectName("MainUI")
        MainUI.resize(1007, 519)
        self.centralWidget = QtWidgets.QWidget(MainUI)
        self.centralWidget.setMinimumSize(QtCore.QSize(813, 0))
        self.centralWidget.setObjectName("centralWidget")
        
        # horizontal layout that contains the sliders
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralWidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 190, 371, 251))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout.setSpacing(6)
        self.horizontalLayout.setObjectName("horizontalLayout")

        # slider widgets
        self.sUpperSlider = QtWidgets.QSlider(self.horizontalLayoutWidget)
        self.sUpperSlider.setOrientation(QtCore.Qt.Vertical)
        self.sUpperSlider.setObjectName("sUpperSlider")
        self.horizontalLayout.addWidget(self.sUpperSlider)
        self.vUpperSlider = QtWidgets.QSlider(self.horizontalLayoutWidget)
        self.vUpperSlider.setOrientation(QtCore.Qt.Vertical)
        self.vUpperSlider.setObjectName("vUpperSlider")
        self.horizontalLayout.addWidget(self.vUpperSlider)
        self.vLowerSlider = QtWidgets.QSlider(self.horizontalLayoutWidget)
        self.vLowerSlider.setOrientation(QtCore.Qt.Vertical)
        self.vLowerSlider.setObjectName("vLowerSlider")
        self.horizontalLayout.addWidget(self.vLowerSlider)
        
        # vertical layout that holds the source and export input
        # along with the associated buttons that gather inputte text
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.centralWidget)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(9, 9, 251, 171))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_3.setContentsMargins(11, 11, 11, 11)
        self.verticalLayout_3.setSpacing(6)
        self.verticalLayout_3.setObjectName("verticalLayout_3")

        # source input
        self.sourceLabel = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        self.sourceLabel.setObjectName("sourceLabel")
        self.verticalLayout_3.addWidget(self.sourceLabel)
        self.sourceInput = QtWidgets.QLineEdit(self.verticalLayoutWidget_3)
        self.sourceInput.setText("")
        self.sourceInput.setObjectName("sourceInput")
        self.verticalLayout_3.addWidget(self.sourceInput)

        # export input
        self.exportLabel = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        self.exportLabel.setObjectName("exportLabel")
        self.verticalLayout_3.addWidget(self.exportLabel)
        self.exportInput = QtWidgets.QLineEdit(self.verticalLayoutWidget_3)
        self.exportInput.setText("")
        self.exportInput.setObjectName("exportInput")
        self.verticalLayout_3.addWidget(self.exportInput)

        # checkbox for default window size
        # note: may take this out if I cannot get it figured out
        self.defaultCheckBox = QtWidgets.QCheckBox(self.verticalLayoutWidget_3)
        self.defaultCheckBox.setObjectName("defaultCheckBox")
        self.verticalLayout_3.addWidget(self.defaultCheckBox)

        # enter button for source
        self.enterButton = QtWidgets.QPushButton(self.centralWidget)
        self.enterButton.setGeometry(QtCore.QRect(260, 50, 113, 32))
        self.enterButton.setAutoDefault(True)
        self.enterButton.setObjectName("enterButton")

        # export button for export input
        self.exportButton = QtWidgets.QPushButton(self.centralWidget)
        self.exportButton.setGeometry(QtCore.QRect(260, 120, 113, 32))
        self.exportButton.setAutoDefault(True)
        self.exportButton.setDefault(False)
        self.exportButton.setFlat(False)
        self.exportButton.setObjectName("exportButton")

        # window that displays the input from the camera or the associated video file
        # note: video file must be specified in the code. Work on getting it from
        # the command line
        self.trafficWindow = QtWidgets.QLabel(self.centralWidget)
        self.trafficWindow.setGeometry(QtCore.QRect(400, 10, 601, 421))
        self.trafficWindow.setScaledContents(True)
        self.trafficWindow.setObjectName("trafficWindow")
        MainUI.setCentralWidget(self.centralWidget)

        self.menuBar = QtWidgets.QMenuBar(MainUI)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 1007, 22))
        self.menuBar.setObjectName("menuBar")
        MainUI.setMenuBar(self.menuBar)
        self.mainToolBar = QtWidgets.QToolBar(MainUI)
        self.mainToolBar.setObjectName("mainToolBar")
        MainUI.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtWidgets.QStatusBar(MainUI)
        self.statusBar.setObjectName("statusBar")
        MainUI.setStatusBar(self.statusBar)

        self.retranslateUi(MainUI)
        self.exportInput.returnPressed.connect(self.exportInput.copy)
        self.sourceInput.returnPressed.connect(self.sourceInput.copy)
        QtCore.QMetaObject.connectSlotsByName(MainUI)

    def retranslateUi(self, MainUI):
        _translate = QtCore.QCoreApplication.translate
        MainUI.setWindowTitle(_translate("MainUI", "UAV GUI"))
        self.sourceLabel.setText(_translate("MainUI", "Source:"))
        self.exportLabel.setText(_translate("MainUI", "Export:"))
        self.defaultCheckBox.setText(_translate("MainUI", "Default Size"))
        self.enterButton.setText(_translate("MainUI", "Enter"))
        self.exportButton.setText(_translate("MainUI", "Export"))

class Thread(QtCore.QThread):
    changePixmap = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self, *args, **kwargs):
        QtCore.QThread.__init__(self, *args, **kwargs)
        self.flag = False

    def run(self):
        cap1 = cv2.VideoCapture(0)
        self.flag = True
        while self.flag:
            ret, frame = cap1.read()
            if ret:
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                cvt2qt = QtGui.QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0], QtGui.QImage.Format_RGB888)
                self.changePixmap.emit(cvt2qt)                         # I don't really understand this yet

    def stop(self):
        self.flag = False


class Prog(QtWidgets.QMainWindow, Ui_MainUI):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.th = Thread(self)
        self.th.changePixmap.connect(self.setImage)
        self.th.start()

    @QtCore.pyqtSlot(QtGui.QImage) 
    def setImage(self, image):
        self.trafficWindow.setPixmap(QtGui.QPixmap.fromImage(image))

    def closeEvent(self, event):
        self.th.stop()
        self.th.wait()
        super().closeEvent(event)

if __name__=='__main__':
    Program =  QtWidgets.QApplication(sys.argv)
    MyProg = Prog()
    MyProg.show()
    sys.exit(Program.exec_())

