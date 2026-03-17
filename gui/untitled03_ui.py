# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'untitled03.ui'
##
## Created by: Qt User Interface Compiler version 6.10.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QLabel, QListView,
    QListWidget, QListWidgetItem, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(707, 569)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.btn_forw = QPushButton(self.centralwidget)
        self.btn_forw.setObjectName(u"btn_forw")
        self.btn_forw.setGeometry(QRect(130, 410, 95, 25))
        self.btn_left = QPushButton(self.centralwidget)
        self.btn_left.setObjectName(u"btn_left")
        self.btn_left.setGeometry(QRect(14, 440, 111, 25))
        self.btn_stop = QPushButton(self.centralwidget)
        self.btn_stop.setObjectName(u"btn_stop")
        self.btn_stop.setGeometry(QRect(130, 440, 95, 25))
        self.btn_righ = QPushButton(self.centralwidget)
        self.btn_righ.setObjectName(u"btn_righ")
        self.btn_righ.setGeometry(QRect(230, 440, 111, 25))
        self.btn_back = QPushButton(self.centralwidget)
        self.btn_back.setObjectName(u"btn_back")
        self.btn_back.setGeometry(QRect(130, 470, 95, 25))
        self.list_log = QListWidget(self.centralwidget)
        QListWidgetItem(self.list_log)
        self.list_log.setObjectName(u"list_log")
        self.list_log.setGeometry(QRect(360, 50, 311, 461))
        self.list_log.setMovement(QListView.Movement.Static)
        self.list_log.setFlow(QListView.Flow.TopToBottom)
        self.list_log.setProperty(u"isWrapping", False)
        self.label_camera = QLabel(self.centralwidget)
        self.label_camera.setObjectName(u"label_camera")
        self.label_camera.setGeometry(QRect(20, 50, 311, 311))
        self.label_camera.setLayoutDirection(Qt.LayoutDirection.LeftToRight)
        self.label_camera.setFrameShape(QFrame.Shape.StyledPanel)
        self.label_camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_01 = QLabel(self.centralwidget)
        self.label_01.setObjectName(u"label_01")
        self.label_01.setGeometry(QRect(20, 20, 131, 17))
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 380, 101, 17))
        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(360, 20, 81, 17))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 707, 27))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.btn_forw.setText(QCoreApplication.translate("MainWindow", u"\uc55e\uc73c\ub85c \uc774\ub3d9", None))
        self.btn_left.setText(QCoreApplication.translate("MainWindow", u"\uc67c\ucabd\uc73c\ub85c \ub3cc\uae30", None))
        self.btn_stop.setText(QCoreApplication.translate("MainWindow", u"\uba48\ucd94\uae30", None))
        self.btn_righ.setText(QCoreApplication.translate("MainWindow", u"\uc624\ub978\ucabd\uc73c\ub85c \ub3cc\uae30", None))
        self.btn_back.setText(QCoreApplication.translate("MainWindow", u"\ub4a4\ub85c \uc774\ub3d9", None))

        __sortingEnabled = self.list_log.isSortingEnabled()
        self.list_log.setSortingEnabled(False)
        ___qlistwidgetitem = self.list_log.item(0)
        ___qlistwidgetitem.setText(QCoreApplication.translate("MainWindow", u"Welcome!", None));
        self.list_log.setSortingEnabled(__sortingEnabled)

        self.label_camera.setText(QCoreApplication.translate("MainWindow", u"label_camera", None))
        self.label_01.setText(QCoreApplication.translate("MainWindow", u"\uc81c\uc2a4\ucc98 \uc778\uc2dd \uce74\uba54\ub77c :", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"\uc218\ub3d9 \ubc84\ud2bc \uc870\uc791 :", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"\ub85c\uadf8 \ub9ac\uc2a4\ud2b8 :", None))
    # retranslateUi

