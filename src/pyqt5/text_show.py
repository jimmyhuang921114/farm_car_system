import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        lbl = QLabel('這是一個 QLabel 文字欄')
        vbox = QVBoxLayout()
        vbox.addWidget(lbl)
        self.setLayout(vbox)
        self.setWindowTitle('QLabel 範例')
        self.setGeometry(300, 300, 300, 200)
        self.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())