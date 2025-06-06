import os
os.environ["GOOGLE_API_KEY"] = "AIzaSyDSLAh4j-7B9l17JUEtbYbUIKN1jW_Q_SM "  # TODO: 請填入你的實際金鑰

from core.core import stop_all_threads
from ui.ui import launch_ui

def main():
    try:
        launch_ui()
    except Exception as error:
        print(f"[Main] Error: {error}")
    finally:
        stop_all_threads()

if __name__ == "__main__":
    main()
