def get_blue_stylesheet() -> str:
    return """
        QWidget {
            background-color: #0d1117;
            color: #e0e6ed;
            font-family: 'Segoe UI', 'SF Pro Display', sans-serif;
            font-size: 13px;
        }
        QGroupBox {
            border: 1px solid #1e3a5f;
            border-radius: 10px;
            margin-top: 14px;
            padding-top: 6px;
            font-weight: bold;
            font-size: 12px;
            color: #38bdf8;
            letter-spacing: 1px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 12px;
            padding: 0 6px;
            background-color: #0d1117;
        }
        QPushButton {
            background-color: #111827;
            border: 1px solid #1e3a5f;
            border-radius: 6px;
            padding: 8px 14px;
            min-width: 70px;
            color: #cbd5e1;
            font-weight: 500;
        }
        QPushButton:hover {
            background-color: #1e293b;
            border-color: #38bdf8;
            color: #f0f9ff;
        }
        QPushButton:pressed {
            background-color: #0284c7;
            border-color: #38bdf8;
            color: #ffffff;
        }
        QPushButton:focus {
            border: 1px solid #06b6d4;
            outline: none;
        }
        QPushButton:disabled {
            background-color: #0a0f1a;
            border: 1px solid #1e293b;
            color: #334155;
        }
        QPushButton#dpad_btn {
            font-size: 20px;
            font-weight: bold;
            min-width: 52px;
            min-height: 52px;
            padding: 4px;
            border-radius: 8px;
            color: #7dd3fc;
            border: 1px solid #1e3a5f;
        }
        QPushButton#dpad_btn:hover {
            background-color: #1e3a5f;
            color: #e0f2fe;
        }
        QPushButton#dpad_btn:pressed {
            background-color: #0369a1;
            border-color: #38bdf8;
            color: #ffffff;
        }
        QPushButton#dpad_btn:disabled {
            color: #1e3a5f;
            border-color: #111827;
        }
        QComboBox {
            background-color: #111827;
            border: 1px solid #1e3a5f;
            border-radius: 5px;
            padding: 4px 8px;
            color: #cbd5e1;
            selection-background-color: #0ea5e9;
        }
        QComboBox::drop-down {
            border: none;
        }
        QComboBox QAbstractItemView {
            background-color: #111827;
            border: 1px solid #1e3a5f;
            selection-background-color: #0369a1;
        }
        QSlider::groove:horizontal {
            border: none;
            height: 6px;
            background: #1e293b;
            border-radius: 3px;
        }
        QSlider::sub-page:horizontal {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 #0369a1, stop:1 #38bdf8
            );
            border-radius: 3px;
            height: 6px;
        }
        QSlider::handle:horizontal {
            background: #38bdf8;
            border: 2px solid #0ea5e9;
            width: 16px;
            height: 16px;
            margin: -5px 0;
            border-radius: 8px;
        }
        QRadioButton::indicator {
            width: 14px;
            height: 14px;
            border-radius: 7px;
            border: 2px solid #38bdf8;
            background: #0d1117;
        }
        QRadioButton::indicator:checked {
            background: #0ea5e9;
            border-color: #38bdf8;
        }
        QScrollBar:vertical {
            background: #111827;
            width: 8px;
            border-radius: 4px;
        }
        QScrollBar::handle:vertical {
            background: #1e3a5f;
            border-radius: 4px;
            min-height: 20px;
        }
    """
