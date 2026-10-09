#!/usr/bin/env python3
"""Interference checks for the Voxel Shell. Run after changing generate.py.

Every check builds solids with manifold3d and measures overlap volume; any
overlap above 0.001 mm3 fails. The four plain shells can be fitted with the
66 mm side of their panel either way round, so all 16 combinations are tested.
"""

import itertools
import sys

import generate as g

TOL = 1e-3
failures = []


def check(name, vol, want_zero=True):
    ok = vol <= TOL if want_zero else vol > TOL
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {vol:.4f} mm3")
    if not ok:
        failures.append(name)


def panel(dx=0.0, dy=0.0, oversize=0.0):
    """PCB + 64 LEDs + 64 capacitors in shell coordinates; LED grid shifted by dx, dy."""
    hx, hy = g.HX + oversize / 2, g.HY + oversize / 2
    pcb = g.box(-hx, hx, -hy, hy, g.Z_BACK, g.Z_PCB)
    leds, caps = [], []
    for i in range(8):
        for j in range(8):
            cx = (i + 0.5) * g.PITCH - g.H + dx
            cy = (j + 0.5) * g.PITCH - g.H + dy
            leds.append(g.box(cx - 2.5, cx + 2.5, cy - 2.5, cy + 2.5, g.Z_PCB, g.Z_PCB + 1.6))
            # capacitor in the gap next to the LED, 1.25 mm tall (0805 worst case)
            caps.append(g.box(cx - 1.0, cx + 1.0, cy + 2.75, cy + 3.35, g.Z_PCB, g.Z_PCB + 1.25))
    return pcb, g.sum_all(leds), g.sum_all(caps)


shell = g.face_shell()
cut = g.port_cut()


def grown(m, d=0.4):
    """Crude Minkowski grow: union of copies shifted by +-d on each axis."""
    out = m
    for v in ([d, 0, 0], [-d, 0, 0], [0, d, 0], [0, -d, 0], [0, 0, d], [0, 0, -d]):
        out = out + m.translate(v)
    return out

print("== one shell and its own panel")
for label, dx, dy in (("LED grid centred", 0, 0), ("LED grid 0.5 mm off-centre on the 66 mm side", 0, 0.5)):
    pcb, leds, caps = panel(dx, dy)
    check(f"PCB vs shell ({label})", (shell ^ pcb).volume())
    check(f"LEDs vs shell ({label})", (shell ^ leds).volume())
    check(f"capacitors vs shell ({label})", (shell ^ caps).volume())
over = round(2 * g.FIT - 0.1, 2)
pcb, _, _ = panel(oversize=over)
check(f"PCB {over} mm oversize ({g.PANEL_X + over} x {g.PANEL_Y + over}) vs shell", (shell ^ pcb).volume())
contact = g.box(-g.HX, g.HX, -g.HY, g.HY, g.Z_PCB, g.Z_PCB + 0.05)
check("shell rim rests on the PCB margin (must be > 0)", (shell ^ contact).volume(), want_zero=False)

print("== assembled cube, every panel orientation")
worst = 0.0
for combo in itertools.product((0, 90), repeat=4):
    shells, boards = [], []
    turns = dict(zip(("back", "top", "right", "left"), combo))
    for name, rot in g.FACES.items():
        spin = turns.get(name, 0)
        base = g.face_shell(g.PORT_FACES[name]) if name in g.PORT_FACES else shell
        sh = base.rotate([0, 0, spin])
        pcb, leds, caps = panel()
        bd = (pcb + leds + caps).rotate([0, 0, spin])
        sh, bd = sh.rotate(rot), bd.rotate(rot)
        if name in g.PORT_FACES:
            sh = sh - cut
        shells.append(sh)
        boards.append(bd)
    for a, b in itertools.combinations(range(6), 2):
        worst = max(worst, (shells[a] ^ shells[b]).volume(), (boards[a] ^ boards[b]).volume())
    allshells, allboards = g.sum_all(shells), g.sum_all(boards)
    worst = max(worst, (allshells ^ allboards).volume())
    usb, sw = g.usb_socket_model(), g.switch_model()
    worst = max(worst, (allshells ^ usb).volume(), (allshells ^ sw).volume(),
                (allboards ^ usb).volume(), (allboards ^ sw).volume())
check("worst overlap over 16 orientations (shells, boards, USB-C, switch)", worst)

print("== funnel cells")
for name, side in g.PORT_FACES.items():
    cells = g.sum_all(g.light_cells(side)).rotate(g.FACES[name])
    check(f"{name} port pockets vs light cells, 0.4 mm margin", (grown(cut) ^ cells).volume())
check("alignment pin holes vs light cells, 0.4 mm margin", (grown(g.pin_holes()) ^ g.sum_all(g.light_cells(0))).volume())
check("alignment pin holes vs port pockets", (g.pin_holes().rotate(g.FACES["front"]) ^ cut).volume()
      + (g.pin_holes().rotate(g.FACES["bottom"]) ^ cut).volume())

print(f"== printability (walls >= {g.MIN_WALL} mm)")
import numpy as np, trimesh
for fname in ("shell.stl", "shell_port_front.stl", "shell_port_bottom.stl", "fit_test.stl"):
    tm = trimesh.load(fname)
    np.random.seed(0)
    pts, fi = trimesh.sample.sample_surface_even(tm, 60000)
    nn = tm.face_normals[fi]
    o = pts - nn * 1e-3
    loc, ri, ti = tm.ray.intersects_location(o, -nn, multiple_hits=False)
    d = np.linalg.norm(loc - o[ri], axis=1)
    # wall thickness = distance to an opposite surface, including ones up to 45 degrees off
    # (print-service checkers flag thin wedges too)
    opp = (tm.face_normals[ti] * nn[ri]).sum(1) < -0.7
    thinnest = float(d[opp].min())
    check(f"{fname} thinnest wall {thinnest:.2f} mm (must be >= {g.MIN_WALL})", g.MIN_WALL - thinnest if thinnest < g.MIN_WALL - 0.005 else 0.0)  # 0.005 mm for float rounding

print("== wiring")
edge_channel = g.box(-20, 20, -g.Z_BACK, -g.HMAX, g.HMAX, g.Z_BACK)
check("port pockets open into the edge channel (must be > 0)", (cut ^ edge_channel).volume(), want_zero=False)

print()
print(f"LED to grid wall: {(g.PITCH - g.WALL - 5.0) / 2:.2f} mm per side (centred), "
      f"{(g.PITCH - g.WALL - 5.0) / 2 - 0.5:.2f} mm if 0.5 mm off-centre")
print(f"PCB to pocket wall: {g.FIT:.2f} mm per side; panel edge to neighbouring panel back: "
      f"{g.Z_BACK - g.HMAX:.2f} mm")
print(f"Outer cube: {2 * g.OUTER:.1f} mm")
if failures:
    print("FAILED:", ", ".join(failures))
    sys.exit(1)
print("All checks passed.")
