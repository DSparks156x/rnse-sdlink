"""Build the firmware into its own directory, no network, no upload: leaves .pio/build (other sessions' output) alone.
   teensy/.venv/Scripts/python.exe tools/build_firmware_out.py [BUILD_DIR]   (default teensy/build_fixes), cwd teensy/ not required"""
import os
from pathlib import Path
import socket
import sys

ROOT = Path(__file__).resolve().parent.parent
os.chdir(ROOT)
out = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / "build_fixes")
os.environ.update(PLATFORMIO_CORE_DIR=str(Path.home() / ".pio-teensy"),
                  PLATFORMIO_BUILD_DIR=out,
                  PLATFORMIO_SETTING_ENABLE_TELEMETRY="false",
                  PLATFORMIO_SETTING_CHECK_PLATFORMIO_INTERVAL="0",
                  PLATFORMIO_SETTING_CHECK_PLATFORMS_INTERVAL="0",
                  PLATFORMIO_SETTING_CHECK_LIBRARIES_INTERVAL="0")
def no_network(*args, **kwargs):
    raise RuntimeError("Network disabled for local firmware verification")
socket.socket.connect = no_network
socket.create_connection = no_network
sys.argv = ["pio", "run"]
from platformio import maintenance
maintenance.on_cmd_end = lambda: None
from platformio.__main__ import main
main()
