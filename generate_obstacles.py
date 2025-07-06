he#!/usr/bin/env python3
"""
generate_obstacles.py
---------------------

Creates 20 obstacle files – 10 valid + 10 intentionally bad – and saves
each one as a separate .txt file inside the folder  ./obstacles/.

GOOD  files : obstacle_001.txt … obstacle_010.txt
BAD   files : obstacle_011.txt … obstacle_020.txt

A zip archive (obstacles.zip) is also produced for quick sharing.
"""

from pathlib import Path
import zipfile
from typing import Dict

def write_all(base_path: Path, mapping: Dict[str, str]) -> None:
# def write_all(base_path: Path, mapping: dict[str, str]) -> None:
    """Write every key/value in *mapping* as a file under *base_path*."""
    base_path.mkdir(exist_ok=True)
    for fname, text in mapping.items():
        (base_path / fname).write_text(text.lstrip(), encoding="utf-8")



good = {
    "obstacle_001.txt": """1
1.50
4
0.0 0.0
4.0 0.0
4.0 3.0
0.0 3.0""",
    "obstacle_002.txt": """0
2.00
4
5.0 0.0
9.0 0.0
9.0 4.0
5.0 4.0""",
    "obstacle_003.txt": """1
2.50
4
10.0 0.0
14.0 0.0
14.0 5.0
10.0 5.0""",
    "obstacle_004.txt": """0
3.00
4
15.0 0.0
19.0 0.0
19.0 6.0
15.0 6.0""",
    "obstacle_005.txt": """1
3.50
4
20.0 0.0
24.0 0.0
24.0 7.0
20.0 7.0""",
    "obstacle_006.txt": """0
4.00
4
25.0 0.0
29.0 0.0
29.0 8.0
25.0 8.0""",
    "obstacle_007.txt": """1
4.50
4
30.0 0.0
34.0 0.0
34.0 9.0
30.0 9.0""",
    "obstacle_008.txt": """0
5.00
4
35.0 0.0
39.0 0.0
39.0 10.0
35.0 10.0""",
    "obstacle_009.txt": """1
5.50
4
40.0 0.0
44.0 0.0
44.0 11.0
40.0 11.0""",
    "obstacle_010.txt": """0
6.00
4
45.0 0.0
49.0 0.0
49.0 12.0
45.0 12.0""",
}

bad = {
    "obstacle_011.txt": """2
1.0
4
0 0
1 0
1 1
0 1""",                       # bad crucial flag
    "obstacle_012.txt": """1
-3.0
4
0 0
1 0
1 1
0 1""",                       # negative weight
    "obstacle_013.txt": """0
1.0
2
0 0
1 1""",                       # too few points
    "obstacle_014.txt": """1
1.0
4
0 0
1 0
1 one
0 1""",                       # non-numeric token
    "obstacle_015.txt": """0
1.0
4
0 0 0
1 0
1 1
0 1""",                       # extra token on a row
    "obstacle_016.txt": """1
1.0
-5
0 0
1 0
1 1
0 1""",                       # negative N
    "obstacle_017.txt": """0
1.0
4
0 0
1 0

1 1""",                       # missing coordinate row
    "obstacle_018.txt": """0
1.0
4
0 0
1 0
1 1""",                       # header says 4, only 3 rows
    "obstacle_019.txt": """0
1.0
4
0 0
1 0
1 1
0 1
2 2""",                       # extra junk after block
    "obstacle_020.txt": """0
1.0
4
0 0
0 0
0 1
0 0"""                        # duplicate points → self-intersection
}

all_files = {**good, **bad}

# ------------------------------------------------------------------ execution
obs_dir = Path("obstacles")
write_all(obs_dir, all_files)

