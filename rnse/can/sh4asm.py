"""Minimal two-pass SH-4 assembler (big-endian 16-bit words).

API: assemble(src: str, base: int) -> bytes
     assemble_listing(src, base) -> list of records (addr, text, data, target)

Syntax: labels `name:`, `;` comments, numbers 0x.. / decimal / negative,
directives .align N, .long expr, .word expr, .byte expr.
Run this file directly to execute the self-test against shdis.dis1().
"""
import re
import struct
import sys
from pathlib import Path


class AsmError(Exception):
    pass


REG = r'r(1[0-5]|[0-9])'
_R = re.compile('^' + REG + '$')
_IND = re.compile(r'^@' + REG + '$')
_POST = re.compile(r'^@' + REG + r'\+$')
_PRE = re.compile(r'^@-' + REG + '$')
_DISP = re.compile(r'^@\(\s*([^,]+?)\s*,\s*' + REG + r'\s*\)$')
_IMM = re.compile(r'^#(.+)$')
_IDENT = re.compile(r'^[A-Za-z_.$][\w.$]*$')


def _split_ops(s):
    ops, depth, cur = [], 0, ''
    for ch in s:
        if ch == '(':
            depth += 1
        elif ch == ')':
            depth -= 1
        if ch == ',' and depth == 0:
            ops.append(cur.strip())
            cur = ''
        else:
            cur += ch
    if cur.strip():
        ops.append(cur.strip())
    return ops


def _expr(s, labels):
    s = s.strip()
    toks = re.findall(r'[+-]?[^+-]+', s.replace(' ', ''))
    if not toks or ''.join(toks) != s.replace(' ', ''):
        raise AsmError('bad expression %r' % s)
    total = 0
    for t in toks:
        sign = 1
        if t[0] in '+-':
            sign = -1 if t[0] == '-' else 1
            t = t[1:]
        try:
            v = int(t, 0)
        except ValueError:
            if t not in labels:
                raise AsmError('undefined symbol %r' % t)
            v = labels[t]
        total += sign * v
    return total


def _kind(op, labels, pass2):
    """Classify operand -> (kind, a, b)."""
    o = op.strip().lower()
    m = _R.match(o)
    if m:
        return ('R', int(m.group(1)), None)
    if o == 'pr':
        return ('PR', None, None)
    m = _IND.match(o)
    if m:
        return ('@R', int(m.group(1)), None)
    m = _POST.match(o)
    if m:
        return ('@R+', int(m.group(1)), None)
    m = _PRE.match(o)
    if m:
        return ('@-R', int(m.group(1)), None)
    m = _DISP.match(op.strip().lower())
    if m:
        return ('@D', _expr(m.group(1), labels) if pass2 else 0, int(m.group(2)))
    m = _IMM.match(op.strip())
    if m:
        return ('#', _expr(m.group(1), labels) if pass2 else 0, None)
    return ('E', op.strip(), None)  # label / address expression


def _rng(v, lo, hi, what):
    if not lo <= v <= hi:
        raise AsmError('%s %d out of range (%d..%d)' % (what, v, lo, hi))
    return v


def _rr(op, n, m):
    return op | (n << 8) | (m << 4)


# (mnemonic) -> opcode low bits for plain "Rm,Rn" forms
_RR = {
    'mov': 0x6003, 'neg': 0x600B, 'extu.b': 0x600C, 'extu.w': 0x600D,
    'exts.b': 0x600E, 'exts.w': 0x600F,
    'add': 0x300C, 'sub': 0x3008, 'and': 0x2009, 'or': 0x200B, 'xor': 0x200A,
    'tst': 0x2008, 'cmp/eq': 0x3000, 'cmp/hs': 0x3002, 'cmp/ge': 0x3003,
    'cmp/hi': 0x3006, 'cmp/gt': 0x3007,
}
_SZ = {'b': 0, 'w': 1, 'l': 2}
_ONE = {'shll': 0x4000, 'shlr': 0x4001, 'shlr2': 0x4009, 'shlr8': 0x4019,
        'shll8': 0x4018, 'dt': 0x4010, 'jmp': 0x402B, 'jsr': 0x400B}
_IMM0 = {'and': 0xC900, 'or': 0xCB00, 'xor': 0xCA00, 'tst': 0xC800}
_BR8 = {'bt': 0x8900, 'bf': 0x8B00, 'bt/s': 0x8D00, 'bf/s': 0x8F00}


def _encode(mn, ops, pc, labels, pass2):
    """Return (word, target_or_None)."""
    K = [_kind(o, labels, pass2) for o in ops]
    kinds = tuple(k[0] for k in K)

    def need(*pat):
        if kinds != pat:
            raise AsmError('bad operands for %s: %s' % (mn, ', '.join(ops)))

    if mn == 'nop':
        need()
        return 0x0009, None
    if mn == 'rts':
        need()
        return 0x000B, None

    if mn in _BR8 or mn in ('bra', 'bsr'):
        need('E')
        tgt = _expr(K[0][1], labels) if pass2 else pc
        if tgt & 1:
            raise AsmError('%s target 0x%x not even' % (mn, tgt))
        d = (tgt - (pc + 4)) // 2
        if mn in _BR8:
            _rng(d, -128, 127, mn + ' displacement')
            return _BR8[mn] | (d & 0xFF), tgt
        _rng(d, -2048, 2047, mn + ' displacement')
        return (0xA000 if mn == 'bra' else 0xB000) | (d & 0xFFF), tgt

    if mn in _ONE:
        need('R' if mn not in ('jmp', 'jsr') else '@R')
        return _ONE[mn] | (K[0][1] << 8), None

    if mn == 'sts.l':
        need('PR', '@-R')
        return 0x4022 | (K[1][1] << 8), None
    if mn == 'lds.l':
        need('@R+', 'PR')
        return 0x4026 | (K[0][1] << 8), None
    if mn == 'sts':
        need('PR', 'R')
        return 0x002A | (K[1][1] << 8), None
    if mn == 'lds':
        need('R', 'PR')
        return 0x402A | (K[0][1] << 8), None

    if mn == 'mova':
        need('E', 'R')
        if K[1][1] != 0:
            raise AsmError('mova destination must be r0')
        tgt = _expr(K[0][1], labels) if pass2 else pc
        if tgt & 3:
            raise AsmError('mova target 0x%x not 4-aligned' % tgt)
        d = (tgt - ((pc + 4) & ~3)) // 4
        _rng(d, 0, 255, 'mova displacement')
        return 0xC700 | d, tgt

    # '#imm,r0' forms for and/or/xor/tst/cmp/eq
    if mn in _IMM0 and kinds == ('#', 'R'):
        if K[1][1] != 0:
            raise AsmError('%s #imm destination must be r0' % mn)
        return _IMM0[mn] | _rng(K[0][1], 0, 255, 'immediate'), None
    if mn == 'cmp/eq' and kinds == ('#', 'R'):
        if K[1][1] != 0:
            raise AsmError('cmp/eq #imm destination must be r0')
        return 0x8800 | (_rng(K[0][1], -128, 127, 'immediate') & 0xFF), None
    if mn == 'add' and kinds == ('#', 'R'):
        return 0x7000 | (K[1][1] << 8) | (_rng(K[0][1], -128, 127, 'immediate') & 0xFF), None
    if mn == 'mov' and kinds == ('#', 'R'):
        return 0xE000 | (K[1][1] << 8) | (_rng(K[0][1], -128, 127, 'immediate') & 0xFF), None

    if mn in _RR and kinds == ('R', 'R'):
        return _rr(_RR[mn], K[1][1], K[0][1]), None

    if mn in ('mov.b', 'mov.w', 'mov.l'):
        s = _SZ[mn[-1]]
        sc = 1 << s
        # PC-relative literal loads
        if kinds == ('E', 'R') and s in (1, 2):
            tgt = _expr(K[0][1], labels) if pass2 else pc
            n = K[1][1]
            if s == 2:
                if tgt & 3:
                    raise AsmError('mov.l literal 0x%x not 4-aligned' % tgt)
                d = (tgt - ((pc + 4) & ~3)) // 4
                _rng(d, 0, 255, 'mov.l literal displacement')
                return 0xD000 | (n << 8) | d, tgt
            if tgt & 1:
                raise AsmError('mov.w literal 0x%x not 2-aligned' % tgt)
            d = (tgt - (pc + 4)) // 2
            _rng(d, 0, 255, 'mov.w literal displacement')
            return 0x9000 | (n << 8) | d, tgt
        if kinds == ('@R', 'R'):
            return _rr(0x6000 | s, K[1][1], K[0][1]), None
        if kinds == ('R', '@R'):
            return _rr(0x2000 | s, K[1][1], K[0][1]), None
        if kinds == ('@R+', 'R'):
            return _rr(0x6004 | s, K[1][1], K[0][1]), None
        if kinds == ('R', '@-R'):
            return _rr(0x2004 | s, K[1][1], K[0][1]), None
        if kinds == ('@D', 'R'):
            d, m = K[0][1], K[0][2]
            n = K[1][1]
            if d % sc:
                raise AsmError('%s displacement %d not multiple of %d' % (mn, d, sc))
            if s == 2:
                _rng(d, 0, 60, 'displacement')
                return 0x5000 | (n << 8) | (m << 4) | (d // 4), None
            if n != 0:
                raise AsmError('%s @(disp,Rm) destination must be r0' % mn)
            _rng(d, 0, 15 * sc, 'displacement')
            return (0x8400 if s == 0 else 0x8500) | (m << 4) | (d // sc), None
        if kinds == ('R', '@D'):
            d, n = K[1][1], K[1][2]
            m = K[0][1]
            if d % sc:
                raise AsmError('%s displacement %d not multiple of %d' % (mn, d, sc))
            if s == 2:
                _rng(d, 0, 60, 'displacement')
                return 0x1000 | (n << 8) | (m << 4) | (d // 4), None
            if m != 0:
                raise AsmError('%s Rm,@(disp,Rn) source must be r0' % mn)
            _rng(d, 0, 15 * sc, 'displacement')
            return (0x8000 if s == 0 else 0x8100) | (n << 4) | (d // sc), None

    raise AsmError('unsupported operand form for %s: %s' % (mn, ', '.join(ops)))


_KNOWN = (set(_RR) | set(_ONE) | set(_BR8) | set(_IMM0) |
          {'nop', 'rts', 'bra', 'bsr', 'sts.l', 'lds.l', 'sts', 'lds', 'mova',
           'mov.b', 'mov.w', 'mov.l', 'cmp/eq'})


def _parse(src):
    """Yield (lineno, label_list, directive_or_mnemonic, operand_str)."""
    out = []
    for ln, raw in enumerate(src.splitlines(), 1):
        line = raw.split(';', 1)[0].strip()
        labs = []
        while True:
            m = re.match(r'^([A-Za-z_.$][\w.$]*)\s*:\s*(.*)$', line)
            if not m:
                break
            labs.append(m.group(1))
            line = m.group(2)
        if not line:
            if labs:
                out.append((ln, labs, None, ''))
            continue
        parts = line.split(None, 1)
        out.append((ln, labs, parts[0].lower(), parts[1].strip() if len(parts) > 1 else ''))
    return out


def assemble_listing(src, base):
    """Two-pass assemble. Returns list of dicts: addr, text, data(bytes), target, mn, ops."""
    items = _parse(src)
    labels = {}
    off = 0
    # pass 1: sizes and labels
    sized = []
    for ln, labs, mn, rest in items:
        for l in labs:
            if l in labels:
                raise AsmError('line %d: duplicate label %r' % (ln, l))
            labels[l] = base + off
        if mn is None:
            continue
        try:
            if mn == '.align':
                n = int(rest, 0)
                if n <= 0 or n & (n - 1):
                    raise AsmError('.align needs a power of two')
                pad = (-(base + off)) % n
                sized.append((ln, mn, rest, off, pad))
                off += pad
            elif mn in ('.long', '.word', '.byte'):
                cnt = len(_split_ops(rest)) or 1
                sz = {'.long': 4, '.word': 2, '.byte': 1}[mn]
                sized.append((ln, mn, rest, off, sz * cnt))
                off += sz * cnt
            elif mn in _KNOWN:
                if (base + off) & 1:
                    raise AsmError('instruction at odd address 0x%x' % (base + off))
                sized.append((ln, mn, rest, off, 2))
                off += 2
            else:
                raise AsmError('unknown mnemonic or directive %r' % mn)
        except AsmError as e:
            raise AsmError('line %d: %s' % (ln, e))
    # pass 2: encode
    recs = []
    for ln, mn, rest, o, size in sized:
        pc = base + o
        try:
            target = None
            if mn == '.align':
                data = b'\0' * size
            elif mn in ('.long', '.word', '.byte'):
                data = b''
                for e in _split_ops(rest):
                    v = _expr(e, labels)
                    if mn == '.long':
                        _rng(v, -2**31, 2**32 - 1, '.long value')
                        data += struct.pack('>I', v & 0xFFFFFFFF)
                    elif mn == '.word':
                        _rng(v, -2**15, 2**16 - 1, '.word value')
                        data += struct.pack('>H', v & 0xFFFF)
                    else:
                        _rng(v, -128, 255, '.byte value')
                        data += bytes([v & 0xFF])
            else:
                w, target = _encode(mn, _split_ops(rest), pc, labels, True)
                data = struct.pack('>H', w)
        except AsmError as e:
            raise AsmError('line %d: %s' % (ln, e))
        recs.append({'addr': pc, 'mn': mn, 'ops': _split_ops(rest), 'text': (mn + ' ' + rest).strip(),
                     'data': data, 'target': target})
    return recs


def assemble(src, base):
    return b''.join(r['data'] for r in assemble_listing(src, base))


# --------------------------------------------------------------------------
# self-test
# --------------------------------------------------------------------------
_TEST = """
top:
    mov r1,r2
    mov #5,r3
    mov #-1,r4
    mov.b @r1,r2
    mov.w @r1,r2
    mov.l @r1,r2
    mov.b r1,@r2
    mov.w r1,@r2
    mov.l r1,@r2
    mov.b @(3,r5),r0
    mov.w @(6,r5),r0
    mov.l @(8,r5),r6
    mov.b r0,@(3,r5)
    mov.w r0,@(6,r5)
    mov.l r6,@(8,r5)
    mov.l r8,@-r15
    mov.w r8,@-r15
    mov.b r8,@-r15
    mov.l @r15+,r8
    mov.w @r15+,r8
    mov.b @r15+,r8
    mov.l lit1,r1
    mov.w wlit,r2
    mova lit2,r0
    sts.l pr,@-r15
    lds.l @r15+,pr
    sts pr,r3
    lds r3,pr
    add r1,r2
    add #-8,r2
    sub r1,r2
    neg r1,r2
    and r1,r2
    or r1,r2
    xor r1,r2
    and #15,r0
    or #0x80,r0
    xor #3,r0
    tst r1,r2
    tst #1,r0
    cmp/eq r1,r2
    cmp/eq #-2,r0
    cmp/hs r1,r2
    cmp/ge r1,r2
    cmp/hi r1,r2
    cmp/gt r1,r2
    extu.b r1,r2
    extu.w r1,r2
    exts.b r1,r2
    exts.w r1,r2
    shll r1
    shlr r1
    shlr2 r1
    shlr8 r1
    shll8 r1
    dt r4
back:
    bt top
    bf back
    bt/s fwd
    nop
    bf/s fwd
    nop
    bra fwd
    nop
    bsr fwd
    nop
    jmp @r2
    nop
    jsr @r3
    nop
fwd:
    rts
    nop
    .align 4
lit1: .long 0x12345678
lit2: .long fwd
wlit: .word 0x1234
    .byte 0x7f
    .byte -1
"""

# source mnemonics where shdis has no decoder; verified against the known opcode word instead
_SHDIS_GAP = {'xor #3,r0': 0xCA03}


def _norm(s):
    return re.sub(r'\s+', '', s.lower())


def _selftest():
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'analysis' / 'settings_menu_v1' / 'scripts'))
    from shdis import dis1
    base = 0x8C010000
    recs = assemble_listing(_TEST, base)
    fails = 0
    for r in recs:
        if r['mn'].startswith('.'):
            print('SKIP  %08x  %-28s %s' % (r['addr'], r['text'], r['data'].hex()))
            continue
        w = struct.unpack('>H', r['data'])[0]
        dis = dis1(r['addr'], w).split(';')[0]
        mn, ops = r['mn'], list(r['ops'])
        if r['target'] is not None:
            if mn in ('bra', 'bsr') or mn in _BR8:
                ops[0] = '%08x' % r['target']
            else:
                ops[0] = '@(%08x)' % r['target']
        exp = mn + ' ' + ','.join(ops)
        exp = re.sub(r'0x[0-9a-f]+', lambda m: str(int(m.group(0), 16)), exp.lower()) if r['target'] is None else exp
        note = ''
        if r['text'].lower() in _SHDIS_GAP:
            ok = w == _SHDIS_GAP[r['text'].lower()]
            note = '  (shdis=%r; checked word %04x)' % (dis, w)
        else:
            ok = _norm(dis) == _norm(exp)
        if not ok:
            fails += 1
        print('%s  %08x  %04x  %-26s -> %s%s' % ('PASS' if ok else 'FAIL', r['addr'], w, r['text'], dis, note))
    # error checks
    bad = [('foo r1', 'unknown mnemonic'), ('mov #200,r1', 'range'), ('bt far\n .align 512\nfar: nop', 'range'),
           ('mov.l l,r0\nl: .long 0', 'misaligned'), ('mov.l @(5,r1),r2', 'range/align')]
    for s, what in bad:
        try:
            assemble(s, 0)
            print('FAIL  error expected (%s): %r' % (what, s))
            fails += 1
        except AsmError as e:
            print('PASS  error (%s): %s' % (what, e))
    print('%s: %d failure(s)' % ('FAIL' if fails else 'ALL PASS', fails))
    return fails


if __name__ == '__main__':
    sys.exit(1 if _selftest() else 0)
