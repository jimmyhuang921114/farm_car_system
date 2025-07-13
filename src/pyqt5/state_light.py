import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor

class StatusLight(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(20, 20)  # 設定最小尺寸
        self.setMaximumSize(20, 20)  # 設定最大尺寸
        self.setAlignment(Qt.AlignCenter)
        self.set_color("gray")  # 預設顏色為灰色

    def set_color(self, color):
        if color == "green":
            self.setStyleSheet("background-color: green; border-radius: 10px;")
        elif color == "red":
            self.setStyleSheet("background-color: red; border-radius: 10px;")
        elif color == "yellow":
            self.setStyleSheet("background-color: yellow; border-radius: 10px;")
        else:
            self.setStyleSheet("background-color: gray; border-radius: 10px;")

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.light = StatusLight(self)
        self.light.set_color("green") #設定燈號顏色
        vbox = QVBoxLayout()
        vbox.addWidget(self.light)
        self.setLayout(vbox)
        self.setWindowTitle('狀態燈號範例')
        self.setGeometry(300, 300, 200, 100)
        self.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())