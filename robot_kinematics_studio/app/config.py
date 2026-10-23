"""
config.py

Central place for application-wide constants. Kept minimal in Phase 1;
will grow to hold user preferences, default units, and rendering
options as later phases add configurable behavior.
"""

APP_NAME = "Robot Kinematics Studio"
APP_VERSION = "0.1.0-phase1"

# Default rendering options
DEFAULT_WORLD_AXIS_LENGTH = 0.3  # meters
DEFAULT_LINK_RADIUS = 0.02       # meters
DEFAULT_JOINT_RADIUS = 0.035     # meters

# Units policy: internally always SI (meters, radians, seconds).
INTERNAL_LENGTH_UNIT = "m"
INTERNAL_ANGLE_UNIT = "rad"
