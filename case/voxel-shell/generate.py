#!/usr/bin/env python3
"""Voxel Shell enclosure for the 8x8x8 Gravity Cube.

Six identical face shells with 45 degree mitred edges hold six 65 x 66 mm
WS2812 8x8 panels. Each shell is one translucent part: a pocket for the PCB,
an 8x8 light grid and a closed diffuser skin on the front. Two variants carry
a USB-C socket and a slide switch on the bottom front edge.

Coordinates: mm, cube centred on the origin. A shell is modelled with its
face normal along +Z; the front (diffuser) face is at z = OUTER.

    pip install manifold3d trimesh numpy
    python3 generate.py            # writes the STL files next to this script
"""

import math
import os

import numpy as np
import trimesh
from manifold3d import Manifold

# ---------------------------------------------------------------- panel
# The boards are 65 x 66 mm. In shell coordinates the 65 mm side runs along X
# and the 66 mm side along Y. The 8x8 LED grid (8.125 mm pitch, outer LEDs
# 1.56 mm from the 65 mm edges) is assumed centred, so the 66 mm direction
# has 0.5 mm extra margin on each side.
PANEL_X = 65.0  # PCB width
PANEL_Y = 66.0  # PCB length
PCB_T = 1.6  # PCB thickness
GRID_SPAN = 65.0  # LED grid span: 8 x pitch
PITCH = GRID_SPAN / 8  # LED pitch, 8.125
EDGE_MARGIN = 1.56  # 65 mm edge to the outer LED rows

# ---------------------------------------------------------------- shell
FIT = 0.25  # clearance around the PCB in its pocket (resin prints vary by ~0.1-0.2)
PANEL_GAP = 1.5  # gap between the back of a panel and its neighbour's edge
RELIEF = 1.3  # grid walls stop this far above the PCB (clears the capacitors)
GRID_DEPTH = 7.5  # PCB front to diffuser skin (deep enough for the USB-C socket)
SKIN = 1.2  # diffuser skin thickness
WALL = 1.6  # grid wall thickness (measured along the face)
# Funnel cells: each LED's cell starts at the LED grid on the PCB and widens
# towards the skin, so the 8 x 8 pixels spread over LIT_SPAN of the outer face.
LIT_SPAN = 80.0  # lit width on the outside of each face (89.6 mm face -> 4.8 mm rim)
# Along the port edge the cells above the USB-C socket cannot spread all the
# way: in PORT_COLS the first PORT_ROWS rows share the space between
# PORT_EDGE and the normal funnel row line instead.
PORT_COLS = (2, 3)  # above the USB-C socket; the switch is small enough for full cells
PORT_EDGE = 37.0  # how far (from the face centre) those cells may reach at the skin
PORT_ROWS = 1  # only the outermost row is shortened
CLAMP_RING = 0.6  # width of the ring that presses on the PCB margin

H = GRID_SPAN / 2  # half of the LED grid (32.5)
HX, HY = PANEL_X / 2, PANEL_Y / 2  # PCB half sizes (32.5, 33.0)
HMAX = max(HX, HY)  # clearances use the long side, so a panel can face either way
Z_BACK = HMAX + PANEL_GAP  # back of the PCB / back of the shell (34.5)
Z_PCB = Z_BACK + PCB_T  # front of the PCB (36.1)
Z_GRID = Z_PCB + RELIEF  # bottom of the grid walls
Z_SKIN = Z_PCB + GRID_DEPTH  # back of the diffuser skin
LED_H = 1.6  # WS2812 5050 package height
Z_KINK = Z_PCB + LED_H + 0.2  # cell walls stay vertical up to here, clear of the LEDs
OUTER = Z_SKIN + SKIN  # front face (44.8 -> 89.6 mm cube)

# ---------------------------------------------------------------- pins
PIN_D = 2.1  # for 2 mm dowels or 1.75 mm filament
PIN_DEPTH = 3.0  # per side; must stay clear of the funnel cells
PIN_ALONG = 25.0  # pin positions along each edge: +-PIN_ALONG
PIN_AT = 37.0  # where the pin axis meets the mitre (x = z = PIN_AT)

# ---------------------------------------------------------------- USB-C + switch (cube coordinates; front = +Z, bottom = -Y)
# Both sit on a 45 degree bevel cut into the bottom front edge, an inset
# facet BEVEL_S deep (measured square to the facet) that stays inside the
# unlit rim. The parts point diagonally into the wedge between the front
# and bottom panels. Part geometry below is written as if the facet were a
# front face at z = OUTER with its centre line at y = 0; bevel_place() turns
# it onto the facet.
BEVEL_S = 2.5  # facet depth from the edge; meets each face BEVEL_S * sqrt(2) from the edge
BEVEL_X = (-21.0, 21.0)  # facet length along the edge
#
# USB-C: snap-in socket with flying leads. Flange 15.8 x 9.3 x 2.0, body 9.0
# deep behind it. The flange stays on the outside; the body goes through a
# cut-out in a 1.6 mm wall (like the thin panel the socket is made for) and its
# snap wings open into the wider pocket behind it.
USB_FLANGE = (15.8, 9.3, 2.0)
USB_BODY_DEPTH = 9.0
USB_CUTOUT = (14.6, 8.2)  # check against your socket's body; flange laps 0.6 mm
USB_WALL = 1.6
USB_X = -9.2  # socket centre along the edge
# Slide switch: SS12F15-style, plate 19.6 x 5.5 with M2 holes 11.5 apart,
# 7.8 mm from the plate to the pin tips, 3 mm lever travel.
SW_PLATE = (19.6, 5.5)
SW_DEPTH = 7.8
SW_PLATE_T = 0.6  # metal mounting plate thickness
SW_BODY_W = 8.6  # switch body length behind the plate
SW_HOLE_PITCH = 11.5
SW_SCREW_D = 2.2  # M2 screws through the wall (set 0 to glue instead)
SW_SLOT = (6.6, 3.4)  # lever opening: 3 mm travel + lever + clearance
SW_WALL = 1.0
SW_X = 10.0
PART_FIT = 0.2  # clearance per side
SW_BODY_H = 3.6  # switch body height across the facet
WIRE_SLOT = (8.0, 1.4)  # wire channel from each pocket into the cube, along the diagonal
WIRE_LEN = 6.0  # how far the wire channel runs past the back of each pocket

SEG = 48


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def rounded_slot(w, h, depth):
    """Stadium-shaped prism, centred in XY, extruded along +Z from 0 to depth."""
    r = h / 2
    c = Manifold.cylinder(depth, r, r, SEG)
    return Manifold.batch_hull([c.translate([-(w / 2 - r), 0, 0]), c.translate([w / 2 - r, 0, 0])])


def frustum():
    """Mitred slab: |x| <= z and |y| <= z for Z_BACK <= z <= OUTER."""
    pts = []
    for z in (Z_BACK, OUTER):
        for sx in (-1, 1):
            for sy in (-1, 1):
                pts.append([sx * z, sy * z, z])
    return Manifold.hull_points(np.array(pts))


def pin_holes():
    holes = []
    for along in (-PIN_ALONG, PIN_ALONG):
        # hole along +Z, then tilt 45 deg so it is normal to the x = z mitre
        h = Manifold.cylinder(PIN_DEPTH * 2, PIN_D / 2, PIN_D / 2, SEG, True)
        h = h.rotate([0, -45, 0]).translate([PIN_AT, along, PIN_AT])
        for k in range(4):
            holes.append(h.rotate([0, 0, 90 * k]))
    return sum_all(holes)


def sum_all(parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def _funnel(n_edge_lo, n_edge_hi):
    """Skin-side boundaries for 8 cells between the two given half-widths."""
    lo, hi = -n_edge_lo, n_edge_hi
    return [lo + k * (hi - lo) / 8 for k in range(9)]


def _rect(x, y, z):
    w = WALL / 2
    return [[xx, yy, z] for xx in (x[0] + w, x[1] - w) for yy in (y[0] + w, y[1] - w)]


def _cell(xb, yb, xt, yt):
    """Light cell: straight beside the LED (up to Z_KINK), then a funnel to its skin rectangle."""
    straight = Manifold.hull_points(np.array(_rect(xb, yb, Z_GRID - 0.01) + _rect(xb, yb, Z_KINK + 0.01)))
    funnel = Manifold.hull_points(np.array(_rect(xb, yb, Z_KINK) + _rect(xt, yt, Z_SKIN)))
    return straight + funnel


def face_shell(port_side=0):
    """One mitred face shell. port_side = -1 / +1 shortens the outer cells along the
    -Y / +Y edge in PORT_COLS, leaving room for the USB-C socket."""
    s = frustum()
    ax, ay = HX + FIT, HY + FIT
    # PCB pocket
    s = s - box(-ax, ax, -ay, ay, Z_BACK - 1, Z_PCB)
    # relief over the LEDs and capacitors, leaving a ring on the PCB margin
    rx, ry = HX - CLAMP_RING, HY - CLAMP_RING
    s = s - box(-rx, rx, -ry, ry, Z_PCB - 0.01, Z_GRID)
    s = s - sum_all(light_cells(port_side))
    s = s - pin_holes()
    return s


def light_cells(port_side=0):
    """The 64 funnel cells of one face (as solids to subtract)."""
    led = [-H + k * PITCH for k in range(9)]
    wl = LIT_SPAN / 2
    xt = _funnel(wl, wl)
    yt_all = _funnel(wl, wl)
    cells = []
    for i in range(8):
        yt = list(yt_all)
        if port_side and i in PORT_COLS:
            n = PORT_ROWS
            if port_side < 0:
                lo, hi = -PORT_EDGE, yt_all[n]
                for k in range(n + 1):
                    yt[k] = lo + k * (hi - lo) / n
            else:
                lo, hi = yt_all[8 - n], PORT_EDGE
                for k in range(n + 1):
                    yt[8 - n + k] = lo + k * (hi - lo) / n
        for j in range(8):
            cells.append(_cell((led[i], led[i + 1]), (led[j], led[j + 1]), (xt[i], xt[i + 1]), (yt[j], yt[j + 1])))
    return cells


def bevel_place(m):
    """Move geometry from the facet frame (surface at z = OUTER, centre line y = 0) onto the bevel."""
    c = OUTER - BEVEL_S / math.sqrt(2)
    return m.translate([0, 0, -OUTER]).rotate([45, 0, 0]).translate([0, -c, c])


def port_cut():
    """Bevel, USB-C socket and slide switch cavities in cube coordinates."""
    f = PART_FIT
    cut = box(BEVEL_X[0], BEVEL_X[1], -30, 30, OUTER, OUTER + 30)
    # USB-C: cut-out through the wall, wider pocket behind for body and snap wings
    cw, ch = USB_CUTOUT
    cut = cut + box(USB_X - cw / 2, USB_X + cw / 2, -ch / 2, ch / 2, OUTER - USB_WALL - 1, OUTER + 1)
    pw = USB_FLANGE[0] + 2 * f
    usb_back = OUTER - USB_WALL - USB_BODY_DEPTH + USB_FLANGE[2] - 0.5
    cut = cut + box(USB_X - pw / 2, USB_X + pw / 2, -ch / 2 - f, ch / 2 + f, usb_back, OUTER - USB_WALL)
    # switch: plate sits against the inside of a thin wall, lever through a slot
    sw, sh = SW_PLATE[0] + 2 * f, SW_PLATE[1] + 2 * f
    plate = OUTER - SW_WALL
    sw_back = plate - SW_DEPTH - 0.4
    # full width only for the thin mounting plate; the body behind it is smaller
    cut = cut + box(SW_X - sw / 2, SW_X + sw / 2, -sh / 2, sh / 2, plate - SW_PLATE_T - 0.2, plate)
    bh = SW_BODY_H / 2 + f
    cut = cut + box(SW_X - SW_BODY_W / 2 - f, SW_X + SW_BODY_W / 2 + f, -bh, bh, sw_back, plate)
    cut = cut + box(SW_X - SW_SLOT[0] / 2, SW_X + SW_SLOT[0] / 2,
                    -SW_SLOT[1] / 2, SW_SLOT[1] / 2, plate - 1, OUTER + 1)
    if SW_SCREW_D:
        for dx in (-SW_HOLE_PITCH / 2, SW_HOLE_PITCH / 2):
            cut = cut + Manifold.cylinder(SW_WALL + 2, SW_SCREW_D / 2, SW_SCREW_D / 2, SEG).translate(
                [SW_X + dx, 0, plate - 1])
    # wire channels: straight on from the back of each pocket, between the two panels, into the cube
    ww, wh = WIRE_SLOT
    for x, zb in ((USB_X, usb_back), (SW_X, sw_back)):
        cut = cut + box(x - ww / 2, x + ww / 2, -wh / 2, wh / 2, zb - WIRE_LEN, zb + 0.01)
    return bevel_place(cut)


def usb_socket_model():
    """Reference model of the USB-C socket, placed in cube coordinates."""
    fw, fh, ft = USB_FLANGE
    flange = box(USB_X - fw / 2, USB_X + fw / 2, -fh / 2, fh / 2, OUTER, OUTER + ft)
    bw, bh = USB_CUTOUT[0] - 0.4, USB_CUTOUT[1] - 0.4
    body = box(USB_X - bw / 2, USB_X + bw / 2, -bh / 2, bh / 2, OUTER + ft - USB_BODY_DEPTH - ft, OUTER)
    mouth = rounded_slot(8.4, 2.6, 7.0).translate([USB_X, 0, OUTER + ft - 7.0 + 0.01])
    return bevel_place(flange + body - mouth)


def switch_model():
    """Reference model of the SS12F15 slide switch, placed in cube coordinates."""
    plate_z = OUTER - SW_WALL
    pw, ph = SW_PLATE
    plate = box(SW_X - pw / 2, SW_X + pw / 2, -ph / 2, ph / 2, plate_z - 0.5, plate_z)
    for dx in (-SW_HOLE_PITCH / 2, SW_HOLE_PITCH / 2):
        plate = plate - Manifold.cylinder(2, 1.0, 1.0, SEG).translate([SW_X + dx, 0, plate_z - 1])
    body = box(SW_X - SW_BODY_W / 2, SW_X + SW_BODY_W / 2, -SW_BODY_H / 2, SW_BODY_H / 2, plate_z - 5.5, plate_z - 0.5)
    pins = sum_all([box(SW_X + dx - 0.4, SW_X + dx + 0.4, -0.25, 0.25, plate_z - SW_DEPTH, plate_z - 5.5)
                    for dx in (-3, 0, 3)])
    lever = box(SW_X - 1.5 - 1.2, SW_X - 1.5 + 1.2, -1.0, 1.0, plate_z, OUTER + 2.5)
    return bevel_place(plate + body + pins + lever)


FIT_TEST_HEIGHT = 2.0  # grid wall height kept in the fit-test piece


def fit_test():
    """Thin slice of a shell: pocket, rim and the bottom of the grid, no skin.

    Cheap to print. Press a real panel in: if it seats flat and no LED touches
    a wall, the full shells fit.
    """
    return face_shell() ^ box(-OUTER, OUTER, -OUTER, OUTER, Z_BACK - 1, Z_GRID + FIT_TEST_HEIGHT)


# face placements: rotation (degrees, applied to a +Z shell) for each cube face
FACES = {
    "front": [0, 0, 0],  # +Z
    "back": [180, 0, 0],  # -Z
    "top": [-90, 0, 0],  # +Y
    "bottom": [90, 0, 0],  # -Y
    "right": [0, 90, 0],  # +X
    "left": [0, -90, 0],  # -X
}


# The port edge is the bottom front edge: -Y on the front shell, +Y on the
# bottom shell (its local +Y points to the front of the cube).
PORT_FACES = {"front": -1, "bottom": 1}


def unplace(m, rot):
    """Inverse of Manifold.rotate(rot) for the single-axis rotations above."""
    return m.rotate([-a for a in rot])


def to_trimesh(m):
    mesh = m.to_mesh()
    t = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts))
    return t


def main():
    out = os.path.dirname(os.path.abspath(__file__))
    shell = face_shell()
    cut = port_cut()

    parts = {"shell.stl": shell, "fit_test.stl": fit_test()}
    placed = {}
    for name, rot in FACES.items():
        if name in PORT_FACES:
            p = face_shell(PORT_FACES[name]).rotate(rot) - cut
            parts[f"shell_port_{name}.stl"] = unplace(p, rot)
        else:
            p = shell.rotate(rot)
        placed[name] = p

    for fname, m in parts.items():
        t = to_trimesh(m)
        t.export(os.path.join(out, fname))
        print(f"{fname:24s} watertight={t.is_watertight} volume={t.volume / 1000:.1f} cm3 "
              f"bbox={np.round(t.extents, 2)}")

    for fname, m in (("part_usb_c_socket.stl", usb_socket_model()), ("part_slide_switch_ss12f15.stl", switch_model())):
        to_trimesh(m).export(os.path.join(out, fname))
        print(f"{fname:24s} (reference only, placed in cube coordinates)")
    cube = sum_all(list(placed.values()))
    t = to_trimesh(cube)
    t.export(os.path.join(out, "assembly_preview.stl"))
    print(f"assembly_preview.stl     edge={t.extents[0]:.1f} mm  (outer cube {2 * OUTER:.1f} mm)")


if __name__ == "__main__":
    main()
