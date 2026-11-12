"""Entry point for the F1 Qt operator app."""
import argparse
import sys

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    from PyQt5.QtWidgets import QApplication

try:
    from .window import OperatorWindow
    from .underwater import underwater_url
except ImportError:
    from window import OperatorWindow
    from underwater import underwater_url


def main():
    parser = argparse.ArgumentParser(description="F1 Windows operator console")
    parser.add_argument("--url", default="ws://127.0.0.1:8765")
    parser.add_argument("--under-url", help="water H7 service; defaults to the same host on port 8766")
    parser.add_argument("--surface-only", action="store_true", help="hide underwater controls")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    app = QApplication(sys.argv)
    try:
        under_url = None if args.surface_only else (args.under_url or underwater_url(args.url))
    except ValueError as error:
        parser.error(str(error))
    window = OperatorWindow(args.url, mock=args.mock, under_url=under_url)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
