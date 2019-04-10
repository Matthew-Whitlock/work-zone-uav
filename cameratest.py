import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QApplication, QAction, QWidget
import cv2

class QtCapture(QtWidgets.QApplication):
    def __init__(self, *args):
        super(QtGui.QWidget, self).__init__()

        self.fps = 24
        self.cap = cv2.VideoCapture(*args)

        self.video_frame = QtGui.QLabel()
        lay = QtGui.QVBoxLayout()
        lay.setMargin(0)
        lay.addWidget(self.video_frame)
        self.setLayout(lay)

        # ------ Modification ------ #
        self.isCapturing = False
        self.ith_frame = 1
        # ------ Modification ------ #

    def setFPS(self, fps):
        self.fps = fps

    def nextFrameSlot(self):
        ret, frame = self.cap.read()

        # ------ Modification ------ #
        # Save images if isCapturing
        if self.isCapturing:
            cv2.imwrite('img_%05d.jpg'%self.ith_frame, frame)
            self.ith_frame += 1
        # ------ Modification ------ #

        # My webcam yields frames in BGR format
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(img)
        self.video_frame.setPixmap(pix)

    def start(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.nextFrameSlot)
        self.timer.start(1000./self.fps)

    def stop(self):
        self.timer.stop()

    # ------ Modification ------ #
    def capture(self):
        if not self.isCapturing:
            self.isCapturing = True
        else:
            self.isCapturing = False
    # ------ Modification ------ #

    def deleteLater(self):
        self.cap.release()
        super(QtGui.QWidget, self).deleteLater()

class vbox(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        lay = QtWidgets.QVBoxLayout(self)


class ControlWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.capture = None

        self.start_button = QtWidgets.QPushButton('Start')
        self.start_button.clicked.connect(self.startCapture)
        self.quit_button = QtWidgets.QPushButton('End')
        self.quit_button.clicked.connect(self.endCapture)
        self.end_button = QtWidgets.QPushButton('Stop')

        # ------ Modification ------ #
        self.capture_button = QtWidgets.QPushButton('Capture')
        self.capture_button.clicked.connect(self.saveCapture)
        # ------ Modification ------ #

        self.setCentralWidget(vbox)
        vbox.addWidget(self.start_button)
        vbox.addWidget(self.end_button)
        vbox.addWidget(self.quit_button)

        # ------ Modification ------ #
        vbox.addWidget(self.capture_button)
        # ------ Modification ------ #

        self.setLayout(vbox)
        self.setWindowTitle('Control Panel')
        self.setGeometry(100,100,200,200)
        self.show()

    def startCapture(self):
        if not self.capture:
            self.capture = QtCapture(0)
            self.end_button.clicked.connect(self.capture.stop)
            # self.capture.setFPS(1)
            self.capture.setParent(self)
            self.capture.setWindowFlags(QtCore.Qt.Tool)
        self.capture.start()
        self.capture.show()

    def endCapture(self):
        self.capture.deleteLater()
        self.capture = None

    # ------ Modification ------ #
    def saveCapture(self):
        if self.capture:
            self.capture.capture()
    # ------ Modification ------ #



if __name__ == '__main__':

    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = ControlWindow()
    sys.exit(app.exec_())