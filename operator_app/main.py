"""Entry point for the F1 Qt operator app."""
import argparse
import sys

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication

try:
    from .window import OperatorWindow
except ImportError:
    from window import OperatorWindow


def main():
    parser = argparse.ArgumentParser(description="F1 Windows operator console")
    parser.add_argument("--url", default="ws://127.0.0.1:8765")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    app = QApplication(sys.argv)
    window = OperatorWindow(args.url, mock=args.mock)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
