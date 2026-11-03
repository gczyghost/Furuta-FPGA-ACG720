"""Numeric fields edit by click/keys; wheel gestures belong to the page."""
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QSpinBox,QDoubleSpinBox

class ScrollSpinBox(QSpinBox):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
    def wheelEvent(self,event):
        event.ignore()

class ScrollDoubleSpinBox(QDoubleSpinBox):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
    def wheelEvent(self,event):
        event.ignore()
