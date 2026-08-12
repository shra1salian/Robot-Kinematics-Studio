"""
main.py

Entry point for Robot Kinematics Studio. Run this file to launch the
application:

    python -m app.main

(run from the project root so the `robot`, `kinematics`, `gui`, and
`visualization` packages are importable).
"""

import sys
import os

# Ensure the project root is on sys.path when run as a script (not -m).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.application import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
