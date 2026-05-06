import os
import sys
import types

# Register core/ as truck_n_trailer so imports work without `pip install -e .`
_core_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if "truck_n_trailer" not in sys.modules:
    _pkg = types.ModuleType("truck_n_trailer")
    _pkg.__path__ = [_core_dir]
    _pkg.__package__ = "truck_n_trailer"
    sys.modules["truck_n_trailer"] = _pkg

from truck_n_trailer.gui.app import main

raise SystemExit(main())
