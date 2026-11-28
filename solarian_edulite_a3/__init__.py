"""
solarian-edulite-a3

EDULITE A3 arm and gripper modules for the Solarian robotics framework.
"""

from pathlib import Path

__version__ = "0.1.1"

#: Directory holding this plugin's simulation assets (MJCF + onshape config).
ASSETS_DIR = Path(__file__).resolve().parent / "assets"

#: Absolute path to the arm MJCF used by MinkIK and scene attachments.
#: Replace this file with the OnShape export when the CAD assembly is ready.
ARM_MJCF_PATH = str(ASSETS_DIR / "edulite_a3.xml")

__all__ = [
    "ASSETS_DIR",
    "ARM_MJCF_PATH",
    "__version__",
]
