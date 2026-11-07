# Course dimensions (from `dronehacks-course-v3.obj`)

Measured from the OBJ mesh. Units are metres with inches in brackets. Z is up and the origin is the centre of the floor.
**The layout is preliminary.** Re-check this file whenever the organizers post a change.

**How this was derived:** I read each named mesh group's vertices and computed gaps, widths and angles from them. The OBJ does not label gates, so **Gate A and Gate B are inferred** from the geometry and the floor-line extent. Verify them visually in the GLB or STEP before relying on them.

Drone footprint: 13 x 13 in (0.33 m). With props, the circumscribed width is about 0.47 m (13 in x sqrt 2). Height is unknown.

---

## Global

| Item | Value |
|---|---|
| Net | 6.096 x 6.096 m (20 x 20 ft), 3.048 m (10 ft) tall |
| Walls and barrier height | 1.727 m (68 in) |
| Wall thickness | about 0.10 m (4 in) |
| Barrier | y = 0.406 to 0.508, running along x |
| Chicane walls | angled **20 degrees from the y axis** (not axis-aligned) |

---

## Maze (north half, y > 0.5)

Dividers run along y. All are 0.10 m thick and 1.727 m tall.

| Divider | x range | y range |
|---|---|---|
| M1 | -1.981 to -1.880 | 0.508 to 2.121 |
| M2 | -0.864 to -0.762 | 1.435 to 3.048 |
| M3 | 0.254 to 0.356 | 0.508 to 2.121 |

| Passage | Width | Free margin each side (frame 0.33 m / props 0.47 m) |
|---|---|---|
| Launch corridor (west wall to M1) | 1.067 m (42 in) | 0.37 / 0.30 m |
| M1 to M2 corridor | 1.016 m (40 in) | 0.34 / 0.27 m |
| M2 to M3 corridor | 1.016 m (40 in) | 0.34 / 0.27 m |
| Gap above M1 (to north wall) | 0.927 m (36.5 in) | 0.30 / 0.23 m |
| Gap below M2 (to barrier) | 0.927 m (36.5 in) | 0.30 / 0.23 m |
| Gap above M3 (to north wall) = **Gate A (inferred)** | 0.927 m (36.5 in) | 0.30 / 0.23 m |

The floor line runs from the launch mark north, east over M1, south between M1 and M2, east under M2, north between M2 and M3, then east over the top of M3. It ends at x of about 0.33, which is why I read the gap above M3 as Gate A.

Launch mark: centre (-2.515, 0.914), 0.152 m (6 in) square.

---

## Tunnel

| Item | Value |
|---|---|
| Cross-section | 1.478 m wide (58 in) x 1.82 m tall (72 in) |
| Length | 1.52 m (60 in) |
| Footprint | x 1.575 to 3.048, y -0.305 to 1.219 |
| Floor | raised about 0.025 m (1 in) |

Enter from the north (y = 1.219), exit at the south (y = -0.305). Width margin is about 0.57 m each side (0.5 m with props).

---

## Chicane and landing area (south half)

C1, C2b and C2a are parallel to each other, tilted 20 degrees.

| Wall | From (north end) | To (south end) |
|---|---|---|
| C1 | (1.48, 0.40), joins the barrier | (0.93, -1.11) |
| C2b | (0.16, -0.55) | (-0.20, -1.51) |
| C2a (window wall) | (-0.20, -1.51) | (-0.75, -3.03), touches the south net wall |
| A1 approach wall | x -1.026 to -0.925 | y from 0.406 down to -0.622 |

C2b and C2a form one continuous wall.

| Passage | Width | Free margin each side (frame / props) |
|---|---|---|
| **C1 to C2b corridor** (perpendicular, the narrowest corridor on the course) | **0.864 m (34 in)** | 0.27 / 0.20 m |
| **Gate B (inferred)**, C2b north tip to barrier south face | **about 0.94 m (37 in)** | 0.30 / 0.23 m |

### Window (in C2a)

| Item | Value |
|---|---|
| Width (along the wall) | **0.914 m (36 in)** |
| Height | **0.914 m (36 in)** |
| Sill height | 0.305 m (12 in) |
| Top edge height | 1.219 m (48 in) |
| Centre height | 0.762 m (30 in) |
| Centre (x, y) | about (-0.47, -2.27) |
| Wall thickness | about 0.10 m (4 in) |

The window wall is tilted 20 degrees, so a drone flying straight west crosses it at an angle. Plan the crossing perpendicular to the wall.

| Margin | Frame (0.33 m) | With props (0.47 m) |
|---|---|---|
| Horizontal, each side | 0.29 m | 0.22 m |
| Vertical, each side | (0.914 minus drone height) / 2 | not known until height is measured |

---

## Landing pad

Centre (-2.042, -0.792). Ring names are radii in inches.

| Ring | Diameter |
|---|---|
| r9 | 0.457 m (18 in) |
| r20 | 1.016 m (40 in) |
| r30 (outer, the scoring ring) | 1.524 m (60 in) |

The whole drone must be inside the outer ring. The 0.33 m frame leaves about 0.6 m of slack from the centre. The return needs a landing within 2 ft (0.61 m) of the launch mark.

---

## Boulders

All are 0.457 m (18 in) across and 0.533 m (21 in) tall.

| Boulder | Centre (x, y) |
|---|---|
| 1 | (2.012, -2.469) |
| 2 | (2.530, -1.554) |
| 3 | (0.792, -2.347) |
| 4 | (-2.560, -2.499) |
| 5 | (-1.341, -2.377) |

Flying at window mid-height (0.76 m) clears the tops by about 0.23 m, but that only helps if the drone isn't also near a wall.

---

## Things to confirm

1. **Barrier 3 is named "cut to 55 in"** but its mesh is 1.727 m (68 in) tall, the same as the other walls. This matters for the return, which flies over the barrier. Ask the organizers which height is correct.
2. **Gate A and Gate B** are inferred, as noted above.
3. **Drone height and true footprint with props.** These decide the vertical margin at the window.
4. **Window and gap sizes** are modelled dimensions. Real built walls may differ by a few centimetres, and the layout is still preliminary.

## Tightest spots, ranked

1. Window: 0.914 m wide, 0.914 m tall.
2. C1 to C2b corridor: 0.864 m wide and angled. This is narrower than the window in width, but it's a long corridor, so pose error accumulates.
3. Maze gaps and Gate B: about 0.93 m.
4. Maze corridors: about 1.02 m.
