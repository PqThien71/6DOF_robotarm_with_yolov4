from serialmodule import *
import serial

from ik6DOF import ik
import tkinter as tk
import math
import glob
import PIL
import os
from PIL import Image
import numpy as np
import argparse
import cv2
import time
from numpy.lib.type_check import isreal, real
from dectect import detection

from PyQt5.QtGui import QImage
from PyQt5.QtCore import QTimer, QDateTime
from PyQt5 import QtCore, QtGui, QtWidgets


mySerial = SerialObject("COM4", 9600, 3)

classes = None  # tạo class từ file yolo.names
with open('config\yolo.names', 'r') as f:
    classes = [line.strip() for line in f.readlines()]
print(classes)


def pixelToMilli(DectectPixelX, DectectPixelY):
    if DectectPixelX == 340:
        Ymm = 0
    elif DectectPixelX > 340:
        Ymm = 200 - DectectPixelX * 370/640
    else:
        Ymm = DectectPixelX * 370/640
    Xmm = 300 - DectectPixelY * 300/480
    return Xmm, Ymm


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(629, 618)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.detectionButton = QtWidgets.QPushButton(self.centralwidget)
        self.detectionButton.setGeometry(QtCore.QRect(20, 460, 101, 41))
        self.detectionButton.setObjectName("detectionButton")
        self.goHomeButton = QtWidgets.QPushButton(self.centralwidget)
        self.goHomeButton.setGeometry(QtCore.QRect(180, 460, 101, 41))
        self.goHomeButton.setObjectName("goHomeButton")
        self.photo = QtWidgets.QLabel(self.centralwidget)
        self.photo.setGeometry(QtCore.QRect(0, 0, 621, 441))
        self.photo.setText("")
        self.photo.setScaledContents(True)
        self.photo.setObjectName("photo")
        self.toaDoX = QtWidgets.QLineEdit(self.centralwidget)
        self.toaDoX.setGeometry(QtCore.QRect(20, 520, 101, 41))
        self.toaDoX.setObjectName("toaDoX")
        self.toaDoY = QtWidgets.QLineEdit(self.centralwidget)
        self.toaDoY.setGeometry(QtCore.QRect(180, 520, 101, 41))
        self.toaDoY.setObjectName("toaDoY")
        self.toaDoZ = QtWidgets.QLineEdit(self.centralwidget)
        self.toaDoZ.setGeometry(QtCore.QRect(340, 520, 101, 41))
        self.toaDoZ.setObjectName("toaDoZ")
        self.apple = QtWidgets.QPushButton(self.centralwidget)
        self.apple.setGeometry(QtCore.QRect(340, 460, 101, 41))
        self.apple.setObjectName("apple")
        self.orange = QtWidgets.QPushButton(self.centralwidget)
        self.orange.setGeometry(QtCore.QRect(500, 460, 101, 41))
        self.orange.setObjectName("orange")
        self.goPointButton = QtWidgets.QPushButton(self.centralwidget)
        self.goPointButton.setGeometry(QtCore.QRect(500, 520, 101, 41))
        self.goPointButton.setObjectName("goPointButton")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 629, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        # connect button set
        self.detectionButton.clicked.connect(self.controlTimer)
        self.apple.clicked.connect(self.starCalAndSendData)
        self.goPointButton.clicked.connect(self.goPointByHand)
        self.goHomeButton.clicked.connect(self.goHome)
        self.timer = QTimer()
        self.timer.timeout.connect(self.viewCam)

    def starCalAndSendData(self):
        for sanPham in listSanPham:
            name = sanPham.split(":")[0]
            x_px = float(sanPham.split(":")[1])
            y_px = float(sanPham.split(":")[2])
            x_mm, y_mm = pixelToMilli(x_px, y_px)
            print(x_mm, y_mm)
            if name == "CAM":
                mode = 1
            if name == "NHO":
                mode = 2
            if name == "KHE":
                mode = 3
            result = ik(x_mm, y_mm, 20)
            result = np.insert(result, 0, mode)
            mySerial.sendData(result)
            time.sleep(45)

    Home = [0, 85, 0, 94, 85, 90, 0]

    def goHome(self):
        mySerial.sendData(self.Home)  # mode 0: go home

    def goPointByHand(self):
        x = float(self.toaDoX.text())
        y = float(self.toaDoY.text())
        z = float(self.toaDoZ.text())
        result = ik(x, y, z)
        result = np.insert(result, 0, 1)
        mySerial.sendData(result)
        print(result)

    def viewCam(self):
        # read image in BGR format
        global image
        ret, image = self.cap.read()
        # convert image to RGB format
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # get image infos
        height, width, channel = image.shape
        step = channel * width
        global listSanPham
        global listSanPhamTest
        listSanPham = []
        listSanPham = detection(image)        # print(listSanPham)
        # create QImage from image
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        # show image in img_label
        self.photo.setPixmap(QtGui.QPixmap.fromImage(qImg))

    def controlTimer(self):
        # if timer is stopped
        if not self.timer.isActive():
            # create video capture
            self.cap = cv2.VideoCapture(0)
            # start timer
            self.timer.start(20)
        # if timer is started
        else:
            # stop timer
            self.timer.stop()
            # release video capture
            self.cap.release()
            # update control_bt text

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.detectionButton.setText(_translate("MainWindow", "Detection"))
        self.goHomeButton.setText(_translate("MainWindow", "Home"))
        self.apple.setText(_translate("MainWindow", "Gắp Cam"))
        self.orange.setText(_translate("MainWindow", "Gắp Nho"))
        self.goPointButton.setText(_translate("MainWindow", "Go Point"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
