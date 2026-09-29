#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 [--apply] /path/to/Edge_Based_Visual_Odometry-or-Multinocular-Edge-Visual-Odometry" >&2
  exit 2
}

apply=0
if [[ "${1:-}" == "--apply" ]]; then
  apply=1
  shift
fi
[[ $# -eq 1 ]] || usage

repo_root=$(cd "$1" && pwd)
header="$repo_root/include/toed/cpu_toed.hpp"
[[ -f "$header" ]] || { echo "missing detector header: $header" >&2; exit 1; }

python3 - "$header" "$apply" <<'PY'
from pathlib import Path
import re
import shutil
import sys

header = Path(sys.argv[1])
apply = sys.argv[2] == "1"
text = header.read_text()

start_match = re.search(r"(?m)^struct Edge\s*\n", text)
if not start_match:
    raise SystemExit(f"{header}: expected the legacy 'struct Edge' block")
start = start_match.start()
class_match = re.search(r"(?m)^class ThirdOrderEdgeDetectionCPU\s*\n", text[start:])
if not class_match:
    raise SystemExit(f"{header}: detector class marker not found; refusing rewrite")
class_start = start + class_match.start()
legacy = text[start:class_start]
if "struct Edge" not in legacy:
    raise SystemExit(f"{header}: legacy edge block was not selected")
if "namespace std" not in legacy:
    raise SystemExit(f"{header}: hash<Edge> block not found; refusing rewrite")

if "#include <lems/data/types.hpp>" not in text:
    include_match = re.search(r"(?m)^#include [\"<]indices\.hpp[\">]\s*$", text)
    if not include_match:
        raise SystemExit(f"{header}: indices.hpp include not found; add the shared include manually")
    text = text[:include_match.start()] + "#include <lems/data/types.hpp>\n" + text[include_match.start():]
    # The insertion above shifts the selected offsets by one line, so locate
    # the class marker again before replacing the edge block.
    start = re.search(r"(?m)^struct Edge\s*\n", text).start()
    class_start = start + re.search(r"(?m)^class ThirdOrderEdgeDetectionCPU\s*\n", text[start:]).start()

replacement = (
    "// Edge and Edge_3D are owned by lems-data.  These aliases preserve the\n"
    "// global names used by the existing detector implementation.\n"
    "using lems::data::Edge;\n"
    "using lems::data::Edge_3D;\n\n"
)
rewritten = text[:start] + replacement + text[class_start:]
if not apply:
    print(f"would replace {header}")
    print("rerun with --apply after reviewing this migration")
    raise SystemExit(0)

backup = header.with_suffix(header.suffix + ".lems-data-backup")
if backup.exists():
    raise SystemExit(f"backup already exists: {backup}; refusing to overwrite it")
shutil.copy2(header, backup)
header.write_text(rewritten)
print(f"updated {header}")
print(f"backup  {backup}")
PY
