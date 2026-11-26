"""Write the paper figures and results table from notebooks/story.py without opening the notebook UI."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "notebooks"))
import story  # noqa: E402

_, defs = story.app.run()
print("written:", ", ".join(defs["write_paper"]()))
