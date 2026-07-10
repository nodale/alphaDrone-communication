import sys
from pathlib import Path

# Make `alpha/` the import root so tests run with the same import context as
# the services themselves (e.g. `from shm.bus import ShmWriter`).
sys.path.insert(0, str(Path(__file__).parent))
