import os
# $env:GOOGLE_API_KEY="YOUR_KEY"; python main.py 

from core.core import stop_all_threads
from ui.ui import launch_ui
from memory_profiler import profile

@profile
def main():
    try:
        launch_ui()
    except Exception as error:
        print(f"[Main] Error: {error}")
    finally:
        stop_all_threads()

if __name__ == "__main__":
    main()
