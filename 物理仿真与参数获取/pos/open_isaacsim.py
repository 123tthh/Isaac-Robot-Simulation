"""
Open Isaac Sim GUI
==================

A minimal script that launches the Isaac Sim GUI with an empty stage,
so you can manually import files and configure physics properties.

Usage:
    python isaaclab_arena/scripts/open_isaacsim.py
"""

import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Open Isaac Sim GUI")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch with GUI
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

print("=" * 50)
print("  Isaac Sim GUI is ready.")
print("  You can now manually import files and")
print("  configure physics properties.")
print("  Close the window to exit.")
print("=" * 50)

# Keep the GUI running
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
