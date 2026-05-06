def get_blue_stylesheet() -> str:
    return """
        QWidget {
            background-color: #0d1117;
            color: #c8d6e8;
            font-family: 'Avenir Next', 'Avenir', 'Helvetica Neue', 'Arial', sans-serif;
            font-size: 12px;
            font-weight: 400;
        }
        QLabel {
            color: #94a3b8;
            font-size: 12px;
        }
        QGroupBox {
            border: 1px solid #1e3a5f;
            border-radius: 8px;
            margin-top: 16px;
            padding-top: 4px;
            font-size: 10px;
            font-weight: 600;
            color: #38bdf8;
            letter-spacing: 2px;
            text-transform: uppercase;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 6px;
            background-color: #0d1117;
        }
        QPushButton {
            background-color: #0f1923;
            border: 1px solid #1e3a5f;
            border-radius: 5px;
            padding: 7px 14px;
            min-width: 70px;
            color: #94a3b8;
            font-size: 11px;
            font-weight: 500;
            letter-spacing: 0.5px;
        }
        QPushButton:hover {
            background-color: #162035;
            border-color: #38bdf8;
            color: #e0f2fe;
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
            border: 1px solid #151f2e;
            color: #2d3d52;
        }
        QPushButton#dpad_btn {
            font-size: 18px;
            font-weight: 600;
            min-width: 52px;
            min-height: 52px;
            padding: 4px;
            border-radius: 8px;
            color: #60a5fa;
            border: 1px solid #1e3a5f;
            background-color: #0c1520;
        }
        QPushButton#dpad_btn:hover {
            background-color: #172035;
            border-color: #38bdf8;
            color: #bae6fd;
        }
        QPushButton#dpad_btn:pressed {
            background-color: #0369a1;
            border-color: #38bdf8;
            color: #ffffff;
        }
        QPushButton#dpad_btn:disabled {
            color: #1a2d42;
            border-color: #0f1923;
            background-color: #080d14;
        }
        QComboBox {
            background-color: #0f1923;
            border: 1px solid #1e3a5f;
            border-radius: 4px;
            padding: 5px 8px;
            color: #94a3b8;
            font-size: 11px;
            selection-background-color: #0ea5e9;
        }
        QComboBox::drop-down {
            border: none;
        }
        QComboBox QAbstractItemView {
            background-color: #0f1923;
            border: 1px solid #1e3a5f;
            selection-background-color: #0369a1;
            color: #c8d6e8;
        }
        QSlider::groove:horizontal {
            border: none;
            height: 4px;
            background: #1a2840;
            border-radius: 2px;
        }
        QSlider::sub-page:horizontal {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 #0369a1, stop:1 #38bdf8
            );
            border-radius: 2px;
            height: 4px;
        }
        QSlider::handle:horizontal {
            background: #38bdf8;
            border: 2px solid #0ea5e9;
            width: 14px;
            height: 14px;
            margin: -5px 0;
            border-radius: 7px;
        }
        QRadioButton {
            color: #94a3b8;
            font-size: 12px;
            spacing: 6px;
        }
        QRadioButton::indicator {
            width: 13px;
            height: 13px;
            border-radius: 7px;
            border: 1.5px solid #2d5a7a;
            background: #0d1117;
        }
        QRadioButton::indicator:checked {
            background: #0ea5e9;
            border-color: #38bdf8;
        }
        QPushButton#auto_start_btn {
            background-color: #166534;
            border: 1px solid #22c55e;
            border-radius: 8px;
            padding: 9px 22px;
            min-width: 92px;
            font-weight: 700;
            font-size: 14px;
            color: #ecfdf5;
        }
        QPushButton#auto_start_btn:hover {
            background-color: #15803d;
            border-color: #4ade80;
            color: #ffffff;
        }
        QPushButton#auto_start_btn:pressed {
            background-color: #14532d;
            border-color: #22c55e;
        }
        QPushButton#auto_start_btn:disabled {
            background-color: #1e293b;
            border-color: #334155;
            color: #64748b;
        }
        QPushButton#auto_stop_btn {
            background-color: #111827;
            border: 1px solid #475569;
            border-radius: 8px;
            padding: 8px 16px;
            min-width: 72px;
            font-weight: 600;
            font-size: 13px;
            color: #cbd5e1;
        }
        QPushButton#auto_stop_btn:hover {
            background-color: #1e293b;
            border-color: #f87171;
            color: #fecaca;
        }
        QPushButton#auto_stop_btn:pressed {
            background-color: #450a0a;
            border-color: #ef4444;
            color: #fee2e2;
        }
        QScrollBar:vertical {
            background: #0d1117;
            width: 6px;
            border-radius: 3px;
        }
        QScrollBar::handle:vertical {
            background: #1e3a5f;
            border-radius: 3px;
            min-height: 20px;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0px;
        }
    """
