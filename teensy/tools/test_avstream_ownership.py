"""Build/run the actual AvStream ownership regressions without hardware.

Uses the existing Teensy venv's ziglang package. --selftest instead runs the
existing full reference self-test with the same local compiler caches.
"""
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CACHE = HERE / "native_test_cache"
for name in ("tmp", "global", "local"):
    (CACHE / name).mkdir(parents=True, exist_ok=True)
os.environ.update(TMP=str(CACHE / "tmp"), TEMP=str(CACHE / "tmp"),
                  ZIG_GLOBAL_CACHE_DIR=str(CACHE / "global"),
                  ZIG_LOCAL_CACHE_DIR=str(CACHE / "local"))

if "--reference-output" in sys.argv:
    output = Path(sys.argv[sys.argv.index("--reference-output") + 1]).read_text()
    reference = runpy.run_path(str(HERE / "selftest.py"))
    expected = reference["REF"].expected_selftest()
    errors = reference["compare"](reference["parse_st"](output.splitlines()), expected)
    print(f"{len(expected)} reference values compared: {'ALL MATCH' if not errors else 'FAILED'}")
    raise SystemExit(bool(errors))
elif "--selftest" in sys.argv:
    sys.argv = [str(HERE / "selftest.py"), "--host"]
    runpy.run_path(str(HERE / "selftest.py"), run_name="__main__")
else:
    py = ROOT / ".venv" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    # AVS_TEST_EXE: put the binary elsewhere, the cache holds earlier runs' output
    exe = Path(os.environ.get("AVS_TEST_EXE") or CACHE / ("avstream_ownership.exe" if os.name == "nt" else "avstream_ownership"))
    compiler = [str(py), "-m", "ziglang", "c++"] if py.exists() else [shutil.which("g++") or "g++"]
    subprocess.run(compiler + ["-O2", "-std=c++17",
                    "-DAVS_HOST", "-DAVS_TEST_INTERLEAVE", "-Wall", "-Wextra",
                    "-I", str(ROOT / "sdemu"), "-o", str(exe), str(HERE / "test_avstream_ownership.cpp"),
                    str(ROOT / "sdemu/avstream.cpp"), str(ROOT / "sdemu/sdproto.cpp")], check=True)
    subprocess.run([str(exe)], check=True)
