"""Generate sin_table.mem for sincos_lut.sv (quarter-wave, plain table).

rom[k] = round(MAX * sin(k * (pi/2) / N)),  N = 2^(PHASE_WIDTH-2),  k = 0..N-1

The table covers [0, pi/2) and rom[0] == 0. sincos_lut mirrors with N - idx
and supplies the pi/2 peak itself (it is one past the table), and relies on
rom[0] == 0 for that -- so OFFSET must stay 0.0 (the RTL asserts this).

Usage: python gen_sin_table.py [PHASE_WIDTH] [OUT_WIDTH]   (default 14 16)
"""
import math
import sys
from pathlib import Path

PHASE_WIDTH = int(sys.argv[1]) if len(sys.argv) > 1 else 14
OUT_WIDTH = int(sys.argv[2]) if len(sys.argv) > 2 else 16
OFFSET = 0.0

N = 1 << (PHASE_WIDTH - 2)
MAX = (1 << (OUT_WIDTH - 1)) - 1          # 32767 for Q15
DIGITS = (OUT_WIDTH + 3) // 4

out = Path(__file__).resolve().parent.parent / "rtl" / "sin_table.mem"
with out.open("w", newline="\n") as f:
    for k in range(N):
        v = round(MAX * math.sin((k + OFFSET) * (math.pi / 2) / N))
        f.write(f"{min(v, MAX):0{DIGITS}x}\n")

print(f"wrote {N} entries ({OUT_WIDTH}-bit, offset {OFFSET}) to {out}")
