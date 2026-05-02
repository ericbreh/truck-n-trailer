import os
import sys

# Allow running as: python -m truck_n_trailer.gui  (from repo root or core/)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from truck_n_trailer.gui.app import main

raise SystemExit(main())
