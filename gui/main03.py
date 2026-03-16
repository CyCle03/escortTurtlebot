import sys
import cv2
import numpy as np
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QThread, Signal, Qt
from PySide6.QtGui import QImage, QPixmap
from untitled03_ui import Ui_MainWindow

# --- 카메라 읽기 전용 스레드 클래스 ---
class CameraThread(QThread):
    # 신호로 전달할 때 QImage 객체를 보냄
    change_pixmap_signal = Signal(QImage)

    def __init__(self):
        super().__init__()
        self._run_flag = True

    def run(self):
        # 0번부터 2번까지 시도해보는 안전 장치 추가
        for idx in [0, 1, 2]:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                break
        
        # MJPG 포맷 강제 지정 (전송 대역폭 문제 해결)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        while self._run_flag:
            ret, frame = cap.read()
            if ret:
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                # 이미지를 복사하여 메인 스레드로 안전하게 전달
                qt_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
                self.change_pixmap_signal.emit(qt_img)
            else:
                self.msleep(30)
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

# --- 메인 윈도우 클래스 ---
class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # [디버깅용] 라벨 위치를 확인하기 위해 배경색을 노란색으로 설정
        self.ui.label_camera.setStyleSheet("background-color: yellow; border: 1px solid black;")
        self.ui.label_camera.setText("Wait for Camera...")

        # 1. 카메라 스레드 인스턴스 생성 및 시작
        self.thread = CameraThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()
        
        self.ui.list_log.addItem("System: Camera Thread Started")

        # 2. 버튼 핸들러 연결
        self.ui.btn_forw.clicked.connect(self.btn_forw_clicked)
        self.ui.btn_left.clicked.connect(self.btn_left_clicked)
        self.ui.btn_righ.clicked.connect(self.btn_righ_clicked)
        self.ui.btn_back.clicked.connect(self.btn_back_clicked)
        self.ui.btn_stop.clicked.connect(self.btn_stop_clicked)

    # 스레드로부터 이미지를 전달받아 화면에 표시
    def update_image(self, qt_img):
        # 라벨 크기 확인 (너무 작으면 기본값 적용)
        lbl_w = self.ui.label_camera.width()
        lbl_h = self.ui.label_camera.height()
        
        if lbl_w > 10 and lbl_h > 10:
            # 전달받은 QImage를 Pixmap으로 변환하여 라벨에 설정
            pixmap = QPixmap.fromImage(qt_img).scaled(
                lbl_w, lbl_h, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self.ui.label_camera.setPixmap(pixmap)

    # --- 버튼 이벤트 함수들 ---
    def btn_forw_clicked(self): self.ui.list_log.addItem("btn_forw Clicked")
    def btn_left_clicked(self): self.ui.list_log.addItem("btn_left Clicked")
    def btn_righ_clicked(self): self.ui.list_log.addItem("btn_righ Clicked")
    def btn_back_clicked(self): self.ui.list_log.addItem("btn_back Clicked")
    def btn_stop_clicked(self): self.ui.list_log.addItem("btn_stop Clicked")

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
