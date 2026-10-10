#!/usr/bin/env python3
"""Voxel Shell enclosure for the Gravity Cube.

Six identical face shells with 45 degree mitred edges hold six WS2812 LED
panels. Each shell is one translucent part: a pocket for the panel, a light
grid and a closed diffuser skin on the front. Two variants carry a USB-C
socket and a slide switch on the bottom front edge.

Two panel profiles (VOXEL_PROFILE environment variable):
  8x8    65 x 66 mm rigid 8x8 panels   -> case/voxel-shell/        (default)
  16x16  160 x 160 mm flexible 16x16   -> case/voxel-shell-16x16/

Coordinates: mm, cube centred on the origin. A shell is modelled with its
face normal along +Z; the front (diffuser) face is at z = OUTER.

    pip install manifold3d trimesh numpy rtree
    python3 generate.py                        # 8x8
    VOXEL_PROFILE=16x16 python3 generate.py    # 16x16
"""

import math
import os

import numpy as np
import trimesh
from manifold3d import Manifold

PROFILE = os.environ.get("VOXEL_PROFILE", "8x8")
if PROFILE not in ("8x8", "16x16"):
    raise SystemExit(f"unknown VOXEL_PROFILE {PROFILE!r}; use 8x8 or 16x16")
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = HERE if PROFILE == "8x8" else os.path.join(os.path.dirname(HERE), "voxel-shell-16x16")

# ---------------------------------------------------------------- panel
if PROFILE == "8x8":
    # Rigid boards, 65 x 66 mm. In shell coordinates the 65 mm side runs along X
    # and the 66 mm side along Y. The 8x8 LED grid (8.125 mm pitch, outer LEDs
    # 1.56 mm from the 65 mm edges) is assumed centred, so the 66 mm direction
    # has 0.5 mm extra margin on each side.
    GRID_N = 8  # LEDs per row
    PANEL_X = 65.0  # PCB width
    PANEL_Y = 66.0  # PCB length
    PCB_T = 1.6  # PCB thickness
    GRID_SPAN = 65.0  # LED grid span: GRID_N x pitch
else:
    # Flexible 16x16 panels, 160 x 160 mm, 10 mm pitch (outer LEDs 2.5 mm from
    # the edge). A flexible panel will not stay flat on its own, so it is glued
    # to a 160 x 160 x 1 mm backing sheet (aluminium or FR4). Panel + glue +
    # sheet make a 1.6 mm stack, the same as the rigid boards; the port and pin
    # geometry along the edges is then identical to the 8x8 shell.
    GRID_N = 16
    PANEL_X = 160.0
    PANEL_Y = 160.0
    PCB_T = 1.6  # flexible PCB (~0.3-0.5 mm) + adhesive + 1 mm backing sheet
    GRID_SPAN = 160.0
PITCH = GRID_SPAN / GRID_N  # LED pitch (8.125 / 10)
EDGE_MARGIN = (PANEL_X - GRID_SPAN) / 2 + (PITCH - 5.0) / 2  # panel edge to the outer LED bodies

# ---------------------------------------------------------------- shell
FIT = 0.25 if PROFILE == "8x8" else 0.4  # clearance around the panel in its pocket (resin prints vary by ~0.1-0.2)
PANEL_GAP = 1.5  # gap between the back of a panel and its neighbour's edge
RELIEF = 1.3  # grid walls stop this far above the PCB (clears the capacitors)
SKIN = 1.0  # diffuser skin thickness (print-service minimum wall)
# PCB front to diffuser skin; with the 1.5 mm panel gap, the 1.6 mm panel stack
# and the skin, the outer face sits EDGE_DEPTH beyond the panel edge.
EDGE_DEPTH = 11.8
GRID_DEPTH = EDGE_DEPTH - PANEL_GAP - PCB_T - SKIN  # 7.7
WALL = 1.6  # grid wall thickness (measured along the face)
RIM = 4.8  # unlit rim at each edge of the outer face
# Funnel cells: each LED's cell starts at the LED grid on the PCB and widens
# towards the skin, so the pixels spread over LIT_SPAN of the outer face.
# Along the port edge the cells above the USB-C socket cannot spread all the
# way: in PORT_COLS the first PORT_ROWS rows share the space between
# PORT_EDGE and the normal funnel row line instead.
PORT_ROWS = 1  # only the outermost row is shortened
PORT_COLS = (2, 3, 4, 5) if PROFILE == "8x8" else (6, 7, 8, 9)  # above the USB-C socket and the switch
CLAMP_RING = 0.6  # width of the ring that presses on the PCB margin

H = GRID_SPAN / 2  # half of the LED grid (32.5)
HX, HY = PANEL_X / 2, PANEL_Y / 2  # PCB half sizes (32.5, 33.0)
HMAX = max(HX, HY)  # clearances use the long side, so a panel can face either way
# outer walls of the edge cells: at least 0.5 mm in from the panel edge (on the
# 8x8 the 66 mm side gives that; the square 16x16 panel needs the cells pulled in)
GRID_EDGE = min(H, HMAX - 0.5)
Z_BACK = HMAX + PANEL_GAP  # back of the PCB / back of the shell (34.5)
Z_PCB = Z_BACK + PCB_T  # front of the PCB (36.1)
Z_GRID = Z_PCB + RELIEF  # bottom of the grid walls
Z_SKIN = Z_PCB + GRID_DEPTH  # back of the diffuser skin
LED_H = 1.6  # WS2812 5050 package height
# cell walls stay vertical up to here, clear of the LEDs; at least 1.1 mm above the
# bottom of the grid so the ledge where the outer wall turns outward is >= 1 mm
Z_KINK = max(Z_PCB + LED_H + 0.2, Z_GRID + 1.1)
OUTER = Z_SKIN + SKIN  # front face (44.8 -> 89.6 mm cube / 91.8 -> 183.6 mm cube)
BIG = max(60.0, OUTER + 20)  # "far enough" for helper cutting boxes
LIT_SPAN = 2 * (OUTER - RIM)  # lit width on the outside of each face (80 / 174 mm)
PORT_EDGE = OUTER - 9.8  # how far (from the face centre) the cells above the ports reach (35 / 82)

# ---------------------------------------------------------------- pins
PIN_D = 2.1  # for 2 mm dowels or 1.75 mm filament
PIN_DEPTH = 3.0  # per side; must stay clear of the funnel cells
PIN_ALONG = (25.0,) if PROFILE == "8x8" else (25.0, 60.0)  # pin positions along each edge: +-each
PIN_AT = OUTER - 7.8  # where the pin axis meets the mitre (x = z = PIN_AT): 37 / 84

# ---------------------------------------------------------------- USB-C + switch (cube coordinates; front = +Z, bottom = -Y)
# All twelve cube edges carry a 45 degree bevel BEVEL_S deep (measured square
# to the facet). It stays inside the unlit rim and turns the knife edge where
# a mitre meets the skin into a solid corner. The socket and switch sit on
# the bevel of the bottom front edge and point diagonally into the wedge
# between the front and bottom panels. Part geometry below is written as if
# the facet were a front face at z = OUTER with its centre line at y = 0;
# bevel_place() turns it onto the facet.
BEVEL_S = 2.5  # facet depth from the edge; meets each face BEVEL_S * sqrt(2) from the edge
#
# USB-C: snap-in socket with flying leads. Flange 15.8 x 9.3 x 2.0, body 9.0
# deep behind it. The flange stays on the outside; the body goes through a
# cut-out in a 1.6 mm wall (like the thin panel the socket is made for) and its
# snap wings open into the wider pocket behind it.
USB_FLANGE = (15.8, 9.3, 2.0)
USB_BODY_DEPTH = 9.0
USB_CUTOUT = (14.6, 8.6)  # check against your socket's body; flange laps 0.6 mm
USB_WALL = 1.6
USB_X = -9.2  # socket centre along the edge
# Slide switch: SS12F15-style, plate 19.6 x 5.5 with M2 holes 11.5 apart,
# 7.8 mm from the plate to the pin tips, 3 mm lever travel.
SW_PLATE = (19.6, 5.5)
SW_DEPTH = 7.8
SW_PLATE_T = 0.6  # metal mounting plate thickness
SW_BODY_W = 8.6  # switch body length behind the plate
SW_HOLE_PITCH = 11.5
SW_SCREW_D = 0  # M2 screw holes through the wall (0 = glue; no room for nuts behind the plate)
SW_SLOT = (6.6, 2.6)  # lever opening: 3 mm travel + lever + clearance; 2.6 tall (2.0 lever) keeps 1 mm to the faces
SW_WALL = 2.1  # the plate pocket runs past the facet; 2.1 keeps its corner >= 1 mm under the faces
SW_X = 10.0
PART_FIT = 0.2  # clearance per side
MIN_WALL = 1.0  # print-service minimum wall (PCBWay review asked for 1.0 mm)
SW_BODY_H = 3.6  # switch body height across the facet
WIRE_SLOT = (8.0, 6.0)  # wire channel from each pocket into the cube, along the diagonal;
# 4.4 tall so it takes out the board-pocket lips where it passes instead of leaving slivers
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


def _rotation_to(n):
    """3x3 rotation taking +Z onto the unit vector n."""
    z = np.array([0.0, 0.0, 1.0])
    n = np.asarray(n, float) / np.linalg.norm(n)
    v = np.cross(z, n)
    c = float(z @ n)
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def chamfered_cube():
    """The outer cube with all twelve edges bevelled at 45 degrees."""
    k = OUTER * math.sqrt(2) - BEVEL_S  # facet distance from the centre
    c = box(-OUTER, OUTER, -OUTER, OUTER, -OUTER, OUTER)
    for a, b in ((0, 1), (0, 2), (1, 2)):
        for sa in (-1, 1):
            for sb in (-1, 1):
                n = np.zeros(3)
                n[a], n[b] = sa, sb
                n /= np.linalg.norm(n)
                m = np.hstack([_rotation_to(n), (n * k).reshape(3, 1)])
                c = c - box(-200, 200, -200, 200, 0, 200).transform(m)
    return c


def pin_holes():
    holes = []
    for along in [s * a for a in PIN_ALONG for s in (-1, 1)]:
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
    """Skin-side boundaries for GRID_N cells between the two given half-widths."""
    lo, hi = -n_edge_lo, n_edge_hi
    return [lo + k * (hi - lo) / GRID_N for k in range(GRID_N + 1)]


def _rect(x, y, z):
    w = WALL / 2
    return [[xx, yy, z] for xx in (x[0] + w, x[1] - w) for yy in (y[0] + w, y[1] - w)]


def _cell(xb, yb, xt, yt):
    """Light cell: straight beside the LED (up to Z_KINK), then a funnel to its skin rectangle."""
    straight = Manifold.hull_points(np.array(_rect(xb, yb, Z_GRID - 0.01) + _rect(xb, yb, Z_KINK + 0.01)))
    funnel = Manifold.hull_points(np.array(_rect(xb, yb, Z_KINK) + _rect(xt, yt, Z_SKIN)))
    return straight + funnel


def face_shell(port_side=0, pins=True):
    """One mitred face shell. port_side = -1 / +1 shortens the outer cells along the
    -Y / +Y edge in PORT_COLS, leaving room for the USB-C socket."""
    s = frustum() ^ chamfered_cube()
    ax, ay = HX + FIT, HY + FIT
    # PCB pocket
    s = s - box(-ax, ax, -ay, ay, Z_BACK - 1, Z_PCB)
    # relief over the LEDs and capacitors, leaving a ring on the PCB margin
    rx, ry = HX - CLAMP_RING, HY - CLAMP_RING
    s = s - box(-rx, rx, -ry, ry, Z_PCB - 0.01, Z_GRID)
    s = s - sum_all(light_cells(port_side))
    if pins:
        s = s - pin_holes()
    return s


def light_cells(port_side=0):
    """The GRID_N x GRID_N funnel cells of one face (as solids to subtract)."""
    led = [-H + k * PITCH for k in range(GRID_N + 1)]
    led[0], led[-1] = -GRID_EDGE, GRID_EDGE
    wl = LIT_SPAN / 2
    xt = _funnel(wl, wl)
    yt_all = _funnel(wl, wl)
    cells = []
    for i in range(GRID_N):
        yt = list(yt_all)
        if port_side and i in PORT_COLS:
            n = PORT_ROWS
            if port_side < 0:
                lo, hi = -PORT_EDGE, yt_all[n]
                for k in range(n + 1):
                    yt[k] = lo + k * (hi - lo) / n
            else:
                lo, hi = yt_all[GRID_N - n], PORT_EDGE
                for k in range(n + 1):
                    yt[GRID_N - n + k] = lo + k * (hi - lo) / n
        for j in range(GRID_N):
            cells.append(_cell((led[i], led[i + 1]), (led[j], led[j + 1]), (xt[i], xt[i + 1]), (yt[j], yt[j + 1])))
    return cells


def bevel_place(m):
    """Move geometry from the facet frame (surface at z = OUTER, centre line y = 0) onto the bevel."""
    c = OUTER - BEVEL_S / math.sqrt(2)
    return m.translate([0, 0, -OUTER]).rotate([45, 0, 0]).translate([0, -c, c])


def _blunt(x0, x1, v_edge, min_t=MIN_WALL):
    """Square off the 45 degree wedges where a pocket wall at v = +-v_edge (facet
    frame) runs out through the front or bottom face, so the shell is at least
    min_t thick there. Returns the material to remove, in cube coordinates."""
    c = math.sqrt(2) * v_edge
    y_c = (c - OUTER) + min_t * math.sqrt(2)  # front face: cut back to here
    v, d = BIG - 20, BIG - 10
    front = bevel_place(box(x0, x1, v_edge, v, OUTER - d, OUTER + 10)) ^ box(x0, x1, -BIG, y_c, -BIG, BIG)
    bottom = bevel_place(box(x0, x1, -v, -v_edge, OUTER - d, OUTER + 10)) ^ box(x0, x1, -BIG, BIG, -y_c, BIG)
    return front + bottom


def port_cut():
    """USB-C socket and slide switch cavities in cube coordinates (on the bottom front bevel)."""
    f = PART_FIT
    # USB-C: cut-out through the wall, wider pocket behind for body and snap wings
    cw, ch = USB_CUTOUT
    cut = box(USB_X - cw / 2, USB_X + cw / 2, -ch / 2, ch / 2, OUTER - USB_WALL - 1, OUTER + 1)
    pw = USB_FLANGE[0]  # pocket as wide as the flange, so every opening stays covered
    usb_back = OUTER - USB_WALL - USB_BODY_DEPTH + USB_FLANGE[2] - 0.5
    cut = cut + box(USB_X - pw / 2, USB_X + pw / 2, -ch / 2, ch / 2, usb_back, OUTER - USB_WALL)
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
    # the snap wings only need the wall in a band around the middle; beyond it the
    # wall would taper to nothing under the faces, so open it (the flange covers it)
    band = BEVEL_S + USB_WALL - MIN_WALL * math.sqrt(2)  # faces run along v + w = BEVEL_S in the facet frame
    for sv in (1, -1):
        lo, hi = sorted((sv * band, sv * (BIG - 20)))
        cut = cut + box(USB_X - pw / 2, USB_X + pw / 2, lo, hi, OUTER - USB_WALL - 0.01, OUTER + 1)
    cut = bevel_place(cut)
    cut = cut + _blunt(USB_X - pw / 2, USB_X + pw / 2, ch / 2)
    # where a wire channel crosses the rim that rests on the board edge, clear that rim
    # strip too: otherwise the channel leaves a thin fin of it standing
    r0, r1 = HMAX - CLAMP_RING - 0.01, HMAX + FIT + 0.01
    for x in (USB_X, SW_X):
        cut = cut + box(x - ww / 2, x + ww / 2, -r1, -r0, Z_PCB - 0.01, Z_GRID)   # front shell
        cut = cut + box(x - ww / 2, x + ww / 2, -Z_GRID, -Z_PCB + 0.01, r0, r1)   # bottom shell
    return cut


def rounded_rect(w, h, r, depth):
    """Rounded rectangle prism, centred in XY, extruded along +Z from 0 to depth."""
    c = Manifold.cylinder(depth, r, r, SEG)
    return Manifold.batch_hull([c.translate([sx * (w / 2 - r), sy * (h / 2 - r), 0]) for sx in (-1, 1) for sy in (-1, 1)])


USB_C_PORT = (8.94, 3.26)  # USB-C receptacle opening (standard)
USB_LEAD_D = 1.2  # lead outer diameter
USB_LEAD_LEN = 5.5  # lead stubs drawn behind the body (they continue into the cube)


def usb_socket_parts():
    """Reference model of the snap-in USB-C socket, as separate parts for colouring.

    Facet frame first (front face of the flange at OUTER + flange thickness),
    then moved onto the bevel. Sizes from the supplier drawing: flange
    15.8 x 9.3 x 2.0, 11.0 deep overall, 15.6 across the snap wings.
    """
    fw, fh, ft = USB_FLANGE
    front = OUTER + ft
    back = front - USB_BODY_DEPTH - ft
    bw, bh = USB_CUTOUT[0] - 0.4, USB_CUTOUT[1] - 0.4
    # clear housing: rounded flange + rounded body
    housing = rounded_rect(fw, fh, 2.6, ft).translate([USB_X, 0, OUTER])
    housing = housing + rounded_rect(bw, bh, 1.4, OUTER - back).translate([USB_X, 0, back])
    # snap wings: thin ramps on both sides, widest just behind the 1.6 mm wall
    wing_z0, wing_z1 = OUTER - USB_WALL - 0.2, OUTER - USB_WALL - 3.6
    for sx in (-1, 1):
        x_in = USB_X + sx * bw / 2
        x_out = USB_X + sx * 15.6 / 2
        pts = []
        for y in (-1.8, 1.8):
            pts += [[x_in, y, wing_z0], [x_out, y, wing_z0], [x_in, y, wing_z1]]
        housing = housing + Manifold.hull_points(np.array(pts))
    pw, ph = USB_C_PORT
    mouth = rounded_slot(pw, ph, 7.0).translate([USB_X, 0, front - 7.0 + 0.01])
    housing = housing - mouth
    # metal receptacle shell and the tongue inside it
    shell = rounded_slot(pw, ph, 6.6) - rounded_slot(pw - 0.5, ph - 0.5, 6.7).translate([0, 0, -0.05])
    tongue = box(-3.3, 3.3, -0.35, 0.35, 0, 5.2)
    metal = (shell + tongue).translate([USB_X, 0, front - 6.6 - 0.05])
    # leads: black and red, side by side, out of the back of the body
    leads = {}
    for colour, dx in (("black", -1.5), ("red", 1.5)):
        r = USB_LEAD_D / 2
        leads[colour] = Manifold.cylinder(USB_LEAD_LEN + 0.5, r, r, SEG).translate([USB_X + dx, 0, back - USB_LEAD_LEN])
    parts = {"housing": housing, "metal": metal, "lead_black": leads["black"], "lead_red": leads["red"]}
    return {k: bevel_place(v) for k, v in parts.items()}


def usb_socket_model():
    """The whole USB-C socket as one solid, placed in cube coordinates (for fit checks)."""
    return sum_all(list(usb_socket_parts().values()))


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


# the fit-test piece keeps the grid walls up to Z_KINK, where they are still vertical


def fit_test():
    """Thin slice of a shell: pocket, rim and the bottom of the grid, no skin.

    Cheap to print. Press a real panel in: if it seats flat and no LED touches
    a wall, the full shells fit.
    """
    # straight outer sides (no mitre), so the flat top meets them at 90 degrees, not a sharp 45
    # (no dowel holes: the test piece does not need them, and they would break out of its sides)
    return face_shell(pins=False) ^ box(-Z_BACK, Z_BACK, -Z_BACK, Z_BACK, Z_BACK - 1, Z_KINK)


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
    out = OUT_DIR
    os.makedirs(out, exist_ok=True)
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

    refs = {"part_usb_c_socket.stl": None, "part_slide_switch_ss12f15.stl": switch_model()}
    usb = usb_socket_parts()
    refs["part_usb_c_socket.stl"] = usb["housing"]
    refs["part_usb_c_metal.stl"] = usb["metal"]
    refs["part_usb_c_lead_black.stl"] = usb["lead_black"]
    refs["part_usb_c_lead_red.stl"] = usb["lead_red"]
    for fname, m in refs.items():
        to_trimesh(m).export(os.path.join(out, fname))
        print(f"{fname:24s} (reference only, placed in cube coordinates)")
    cube = sum_all(list(placed.values()))
    t = to_trimesh(cube)
    t.export(os.path.join(out, "assembly_preview.stl"))
    print(f"assembly_preview.stl     edge={t.extents[0]:.1f} mm  (outer cube {2 * OUTER:.1f} mm)")


if __name__ == "__main__":
    main()
