r"""Refresh this repo from the working tree (analysis/av_over_sd_v1) using tools/manifest.txt, then show git status.

  python -I tools/sync_from_worktree.py [--dry]

Only files named in the manifest are copied (standard library only). Source files in the scanned directories that are
neither listed nor marked 'skip' are reported so nothing silently goes missing. Text files get a few fixed
substitutions (SCRUB / PATCHES below) so personal paths and the analysis-tree layout do not leak into the repo.
Never touches the source tree.
"""
import fnmatch, glob, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..'))
# the author's working tree ('RNSE hacking'): next to this repo, or set SDLINK_WORKTREE
ROOT = os.environ.get('SDLINK_WORKTREE') or os.path.normpath(os.path.join(REPO, '..', 'RNSE hacking'))
SRC_TREE = 'analysis/av_over_sd_v1'
SRC_EXT = {'.c', '.h', '.cpp', '.py', '.sh', '.s', '.ld', '.ino', '.ini', '.md', '.mk'}
IGNORE_DIRS = {'.venv', '.pio', '__pycache__', 'out', 'results', 'logs', 'sim', 'evidence', 'fw', 'build'}
GIT = 'git'

HOME = os.path.expanduser('~')   # the author's profile folder: scrubbed from copied files, never written into the repo
PY312 = HOME + r'\AppData\Local\Programs\Python\Python312\python.exe'
SCRUB = [   # (old, new) applied to every copied text file
    (PY312, 'python'),
    (HOME + r'\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe', 'python'),
    (PY312.replace('\\', '\\\\'), 'python'),
    ('r"' + HOME + r'\Documents\carstuff\RNSE hacking\analysis\av_over_sd_v1\teensy\tools"',
     'os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")'),
    ('192.168.12.1', '<pi-address>'),
    (HOME + r'\.pio-teensy\packages\tool-teensy', r'<PLATFORMIO_CORE_DIR>\packages\tool-teensy'),
]
NOTE_SPEC = ("> Written as a design document, since implemented and run on the bench (see the root README for the real state).\n"
             "> References to other folders (datasheets, analysis notes, FACTS.md, sdgen.py, video/) point into the author's private\n"
             "> working tree and are not part of this repo.")
PATCHES = {   # dest -> [(old, new)]: the CAN transport lives in rnse/can/ here instead of ../../tools and analysis/soc_caps_v1
    'docs/PROTOCOL.md': [
        ("Design on paper. Nothing in this folder has touched the car or the bench.", NOTE_SPEC)],
    'pi/avlink/README.txt': [
        ("avlink -- Pi 5 side", "NOTE: paths such as ../../link, rnse_proto/sdgen.py and video/out in this file refer to the author's private working tree.\nIn this repo: rnse_proto = rnse/, tests are pi/avlink/test/ (stages 2 and 3 need sdgen.py and source clips that are not here: use --skip-sdgen).\n\navlink -- Pi 5 side")],
    'pi/avlink/test/run_tests.py': [
        ("PROTO = os.path.join(ROOT, 'rnse_proto')", "PROTO = os.path.join(ROOT, 'rnse')"),
        ("import sdgen    # noqa: E402",
         "try:\n    import sdgen    # noqa: E402  (reference generator + source clips are not in this repo)\nexcept ImportError:\n    sdgen = None"),
        ("    ap.add_argument('--skip-live', action='store_true')",
         "    ap.add_argument('--skip-live', action='store_true')\n"
         "    ap.add_argument('--skip-sdgen', action='store_true', help='skip stages 2 and 3 (they need sdgen.py and the source clips)')"),
        ("    build(not a.skip_e2e)",
         "    if a.skip_sdgen or sdgen is None:\n        a.skip_e2e = True\n    build(not a.skip_e2e)"),
        ("    print('[2] superframes vs sdgen.py')\n    res['sdgen'], refs = test_sdgen(cases)\n",
         "    if a.skip_sdgen or sdgen is None:\n"
         "        print('[2][3] superframes vs sdgen.py, end to end: skipped (need sdgen.py and the source clips, not in this repo)')\n"
         "        refs = None\n"
         "    else:\n"
         "        print('[2] superframes vs sdgen.py')\n"
         "        res['sdgen'], refs = test_sdgen(cases)\n")],
    'pi/scripts/car/CAR_INSTALL.md': [
        ("# A/V over SD: car install",
         "> Note: R61.md, rebuild.py, sim_r61.py and tva_bench.py named below are the author's private firmware tooling and are not in this repo; the Pi side (install_car.sh, services, hud2.sh) is.\n\n# A/V over SD: car install")],
    'rnse/rnse_bench.py': [
        ("sys.path.insert(0, os.path.join(ROOT, 'analysis', 'soc_caps_v1'))", "sys.path.insert(0, os.path.join(HERE, 'can'))")],
    'rnse/can/probe.py': [
        ("ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))\nsys.path.insert(0, os.path.join(ROOT, 'tools'))",
         "sys.path.insert(0, HERE)")],
}


def read_manifest():
    entries, skips, scans = [], [], []
    for ln in open(os.path.join(HERE, 'manifest.txt'), encoding='utf-8'):
        ln = ln.strip()
        if not ln or ln.startswith('#'):
            continue
        p = ln.split()
        if p[0] == 'skip':
            skips.append(p[1])
        elif p[0] == 'scan':
            scans.append(p[1])
        else:
            entries.append((p[0], p[1]))
    return entries, skips, scans


def transform(dest, data):
    try:
        s = data.decode("utf-8").replace(chr(13) + chr(10), chr(10))   # the repo is LF only (.gitattributes)
    except UnicodeDecodeError:
        return data
    for a, b in SCRUB:
        s = s.replace(a, b)
    for a, b in PATCHES.get(dest, []):
        if a not in s:
            print('  WARNING: patch not applicable in %s (source changed?): %r' % (dest, a[:50]))
        s = s.replace(a, b)
    return s.encode('utf-8')


def main():
    dry = '--dry' in sys.argv
    entries, skips, scans = read_manifest()
    listed, changed, missing = set(), [], []
    for dest, src in entries:
        if dest.endswith('/'):
            files = sorted(glob.glob(os.path.join(ROOT, src)))
            if not files:
                print("  (no files yet for %s)" % src)
            pairs = [(dest + os.path.basename(f), f) for f in files if os.path.isfile(f)]
        else:
            f = os.path.join(ROOT, src)
            pairs = [(dest, f)]
            if not os.path.isfile(f):
                missing.append(src)
                pairs = []
        for d, f in pairs:
            listed.add(os.path.normcase(os.path.normpath(f)))
            new = transform(d, open(f, 'rb').read())
            out = os.path.join(REPO, d)
            old = open(out, 'rb').read() if os.path.isfile(out) else None
            if old != new:
                changed.append(d)
                if not dry:
                    os.makedirs(os.path.dirname(out), exist_ok=True)
                    open(out, 'wb').write(new)
    print('manifest entries: %d, files updated: %d%s' % (len(entries), len(changed), ' (dry run)' if dry else ''))
    for d in changed:
        print('  updated', d)
    for m in missing:
        print('MISSING SOURCE (listed in manifest, not found):', m)

    base = os.path.join(ROOT, SRC_TREE)
    unlisted = []
    for sc in scans:
        for dp, dn, fn in os.walk(os.path.join(base, sc)):
            dn[:] = [d for d in dn if d not in IGNORE_DIRS]
            for n in fn:
                f = os.path.join(dp, n)
                rel = os.path.relpath(f, base).replace(os.sep, '/')
                if os.path.splitext(n)[1].lower() not in SRC_EXT or '.bak' in n or '.before_' in n:
                    continue
                if os.path.normcase(os.path.normpath(f)) in listed or any(fnmatch.fnmatch(rel, s) for s in skips):
                    continue
                unlisted.append(rel)
    if unlisted:
        print('\nNEW UNLISTED SOURCE FILES (add to tools/manifest.txt as a copy line or a skip line):')
        for u in sorted(unlisted):
            print('  ', u)
    else:
        print('no unlisted source files')
    print()
    subprocess.run([GIT, '-C', REPO, 'status', '--short'])


if __name__ == '__main__':
    main()
