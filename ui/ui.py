from PyQt5.QtWidgets import QApplication
from ui.ui_window import ControlPanel
import sys

def launch_ui():
    app = QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())