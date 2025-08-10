import shutil
import sys
from pathlib import Path

# Paths
venv_path = Path(sys.executable).parent.parent  # Go from python.exe to venv root
src = Path("patches/jsbsim/aircraft/c172x/c172ap.xml")
dst = venv_path / "Lib" / "site-packages" / "jsbsim" / "aircraft" / "c172x" / "c172ap.xml"

# Copy the patched file
print(f"Copying {src} -> {dst}")
shutil.copy2(src, dst)