"""Shared theme tokens for native widgets and painted charts."""
PALETTES = {
    'dark': dict(bg='#0b1423', panel='#131e30', text='#e1eaf5', muted='#91a4bc',
                 line='#2a3c55', field='#101a2b', button='#20324c', hover='#304c70',
                 accent='#6eacff', teal='#44dec0', amber='#f5bf62', danger='#d74959'),
    'light': dict(bg='#edf2f8', panel='#ffffff', text='#1b2d45', muted='#566b83',
                  line='#d4dfed', field='#f6f9fd', button='#e6eef9', hover='#d3e3f8',
                  accent='#2361bf', teal='#007f70', amber='#9b6100', danger='#bd293c'),
}


def stylesheet(name):
    c = PALETTES[name]
    return '''
    QWidget { background: %(bg)s; color: %(text)s; font-family: "Microsoft YaHei UI"; font-size: 12px; }
    QFrame#panel { background: %(panel)s; border: 1px solid %(line)s; border-radius: 10px; }
    QFrame#panel QLabel { background: transparent; }
    QPushButton#dialPosition { padding: 2px; font-size: 11px; }
    QLabel#title { font-size: 24px; font-weight: 700; }
    QLabel#muted { color: %(muted)s; }
    QLabel#section { color: %(muted)s; font-weight: 600; }
    QLabel#metric { font-size: 27px; font-weight: 600; }
    QLabel#badge { background: %(button)s; color: %(accent)s; padding: 8px 14px; border-radius: 7px; }
    QPushButton { background: %(button)s; border: 1px solid %(line)s; border-radius: 6px; padding: 9px 12px; }
    QPushButton:hover { background: %(hover)s; border-color: %(accent)s; }
    QPushButton:disabled { color: %(muted)s; background: %(field)s; }
    QFrame#panel QPushButton#primary { background: #2869c7; color: white; font-weight: 700; }
    QFrame#panel QPushButton#primary:disabled { background: %(button)s; color: %(muted)s; }
    QPushButton#stop { background: %(danger)s; color: white; font-weight: 700; }
    QComboBox, QDoubleSpinBox { background: %(field)s; border: 1px solid %(line)s; border-radius: 6px; padding: 7px; }
    QComboBox { padding-right: 30px; min-height: 20px; }
    QComboBox:hover, QComboBox:focus { border-color: %(accent)s; }
    QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; width: 28px; border: none; background: transparent; }
    QComboBox::down-arrow { image: none; width: 0px; height: 0px; }
    QComboBox QAbstractItemView { background: %(panel)s; color: %(text)s; border: 1px solid %(line)s; padding: 4px; outline: none; selection-background-color: %(hover)s; selection-color: %(text)s; }
    QComboBox QAbstractItemView::item { padding: 7px 10px; min-height: 22px; }
    QPlainTextEdit { background: %(field)s; border: 1px solid %(line)s; border-radius: 6px; padding: 5px; }
    QTabWidget::pane { border: 1px solid %(line)s; border-radius: 8px; background: %(panel)s; }
    QTabBar::tab { padding: 10px 20px; color: %(muted)s; border: none; }
    QTabBar::tab:selected { color: %(accent)s; border-bottom: 2px solid %(accent)s; }
    QProgressBar { background: %(line)s; border: none; border-radius: 4px; height: 8px; }
    QProgressBar::chunk { background: %(teal)s; border-radius: 4px; }
    QScrollArea { border: none; }
    QScrollBar:vertical { background: %(field)s; width: 10px; }
    QScrollBar::handle:vertical { background: %(line)s; min-height: 24px; }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
    QScrollBar#leftScrollBar:vertical { background: %(bg)s; width: 10px; margin: 2px 0; border: none; }
    QScrollBar#leftScrollBar::handle:vertical { background: %(line)s; min-height: 32px; border-radius: 4px; }
    QScrollBar#leftScrollBar::handle:vertical:hover { background: %(muted)s; }
    QScrollBar#leftScrollBar::handle:vertical:pressed { background: %(accent)s; }
    QScrollBar#leftScrollBar::add-page:vertical, QScrollBar#leftScrollBar::sub-page:vertical { background: transparent; }
    QSlider::groove:horizontal { height: 5px; background: %(line)s; border-radius: 2px; }
    QSlider::handle:horizontal { width: 12px; margin: -4px 0; background: %(accent)s; border-radius: 6px; }
    QToolTip { background: %(panel)s; color: %(text)s; border: 1px solid %(line)s; padding: 6px; }
    ''' % c
