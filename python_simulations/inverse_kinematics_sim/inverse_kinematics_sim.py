"""
Inverse Kinematics Simulation for differential drive robot.
Robot follows waypoints using PID-based motion control.
"""

import sys
import traceback
from PySide6.QtWidgets import QApplication, QMessageBox, QMainWindow
from PySide6.QtCore import Qt
from path_view_widget import PathViewWidget


class MainWindow(QMainWindow):
    """
    Main application window containing the OpenGL visualization.
    """
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Inverse Kinematics Simulation - Python (Click to add waypoints | R - Reset | ↑↓/PgUp/PgDn - Zoom)")
        self.setGeometry(100, 100, 1000, 800)
        
        # Create and set OpenGL widget as central widget (full window)
        self.gl_widget = PathViewWidget(self)
        self.setCentralWidget(self.gl_widget)
        
        # Set focus to OpenGL widget for keyboard input
        self.gl_widget.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.gl_widget.setFocus()


def main():
    """Main entry point with comprehensive exception handling."""
    app = None
    
    try:
        # Initialize Qt application
        app = QApplication(sys.argv)
        
        # Create and show main window
        window = MainWindow()
        window.show()
        
        # Run application event loop
        return app.exec()
        
    except KeyboardInterrupt:
        print("\nApplication interrupted by user (Ctrl+C)")
        return 0
        
    except ImportError as e:
        # Handle missing dependencies
        error_msg = (
            f"Failed to import required modules:\n{str(e)}\n\n"
            f"Make sure all dependencies are installed:\n"
            f"- PySide6\n"
            f"- PyOpenGL\n"
            f"- numpy"
        )
        print(error_msg, file=sys.stderr)
        
        if app is not None:
            QMessageBox.critical(None, "Import Error", error_msg)
        
        return 1
        
    except Exception as e:
        # Catch any unexpected errors
        error_msg = (
            f"Unexpected error occurred:\n"
            f"{type(e).__name__}: {str(e)}\n\n"
            f"Traceback:\n{traceback.format_exc()}"
        )
        
        print(error_msg, file=sys.stderr)
        
        # Show GUI error if app was created
        if app is not None:
            QMessageBox.critical(
                None,
                "Critical Error",
                error_msg
            )
        
        return 1
    
    finally:
        # Cleanup
        if app is not None:
            try:
                app.quit()
            except:
                pass


if __name__ == "__main__":
    sys.exit(main())
