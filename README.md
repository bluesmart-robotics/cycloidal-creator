<p align="center">
  <img alt="cycloidgen - parametric cycloidal drive design. A 15:1 cycloidal drive turning: the disc orbits on its eccentric and rolls round the ring pins." src="https://raw.githubusercontent.com/medinstech/cycloidgen/main/docs/banner.svg" width="100%">
</p>

<p align="center">
  A desktop app that takes a handful of parameters — or a set of requirements —<br>
  runs the drive live in 2D and 3D, checks it, sizes it, and writes DXF, SVG, STEP,<br>
  STL, a coloured 3MF assembly, a looping animation, a bill of materials and a<br>
  PDF dossier.
</p>

<p align="center">
  <a href="https://github.com/medinstech/cycloidgen/actions/workflows/tests.yml"><img alt="tests" src="https://img.shields.io/github/actions/workflow/status/medinstech/cycloidgen/tests.yml?branch=main&style=flat-square&label=tests"></a>
  <a href="https://github.com/medinstech/cycloidgen/releases/latest"><img alt="latest release" src="https://img.shields.io/github/v/release/medinstech/cycloidgen?style=flat-square&color=0d00ff&label=release"></a>
  <a href="https://github.com/medinstech/cycloidgen/blob/main/pyproject.toml"><img alt="Python 3.10 – 3.14" src="https://img.shields.io/badge/python-3.10%20%E2%80%93%203.14-0d00ff?style=flat-square"></a>
  <a href="https://github.com/medinstech/cycloidgen/blob/main/LICENSE"><img alt="Apache-2.0" src="https://img.shields.io/badge/license-Apache--2.0-0d00ff?style=flat-square"></a>
</p>

<p align="center">
  <a href="https://github.com/medinstech/cycloidgen/releases/latest"><b>Download</b></a>
  &nbsp;·&nbsp; <a href="#run-it"><b>pip install cycloidgen</b></a>
  &nbsp;·&nbsp; <a href="https://github.com/medinstech/cycloidgen/blob/main/docs/api.md">Python API</a>
  &nbsp;·&nbsp; <a href="#checks">The checks</a>
  &nbsp;·&nbsp; <a href="https://github.com/medinstech/cycloidgen/blob/main/CHANGELOG.md">Changelog</a>
</p>

![the application](https://raw.githubusercontent.com/medinstech/cycloidgen/main/docs/app-drawing.png)

<p align="center">
  Selecting a check tells you what it tests, why it matters and what to change;<br>
  the parameters it names light up in the panel on the left.
</p>

---

- **Exact geometry.** The conjugate profile in closed form, verified to sit at
  exactly the pin radius from the pin-centre locus — envelope deviation 0.0 µm.
- **Either member as the output.** Bolt the housing down and take `N:1`
  reversed off the carrier, or bolt the carrier down and take `N+1:1` off the
  turning housing. The mounting face, the bearing speeds and the part that
  turns on screen all follow.
- **A datasheet, not a drawing.** Contact stress, torque capacity, efficiency,
  torsional stiffness, lost motion, transmission error, PV and running
  temperature, lubrication regime, mass and inertia, bearing life.
- **Seventy-four checks that explain themselves.** Each says what it tests, what
  goes wrong physically when it fails, and which parameter to move — and lights
  that parameter up in the panel.
- **Requirements in, geometry out.** Say ratio, torque, speed and envelope; get
  a shortlist that passes every check, with the trade-offs side by side. Or say
  what motor you have and let it pick the reduction: a motor is a torque-speed
  curve, not the number on its label, and what it can turn is what the drive is
  worth.
- **Nothing is asserted that is not verified.** 1,521 tests, and where two parts
  of the app describe the same gearbox they are checked against each other —
  the 3D mesh against the volume the exported solid encloses, the export
  manifest against the files that land on disk.

**Preliminary sizing numbers, not a certification.** See
[*How far to trust the analysis*](#how-far-to-trust-the-analysis).

**Jump to** — [Run it](#run-it) · [Three ways to use it](#three-ways-to-use-it) ·
[What it produces](#what-it-produces) · [The geometry](#the-geometry) ·
[What it tells you](#what-it-tells-you) · [Checks](#checks) ·
[In the app](#in-the-app) · [Tests](#tests) · [Performance](#performance) ·
[Contributing](#contributing)

![overview](https://raw.githubusercontent.com/medinstech/cycloidgen/main/docs/drawing.png)

The drawing is a simulation, not an outline. The dots are where the disc touches
each ring pin and the arrows are the load it carries there, both off the same
kinematics the checks and the datasheet use; the faint wavy ring is the path one
point on the disc rim travels over a full output revolution. Twenty-one turns of
the input, one of the output.

![the 3D tab](https://raw.githubusercontent.com/medinstech/cycloidgen/main/docs/app-3d.png)

<p align="center">
  X-ray on and a disc picked: the housing and its end plates go translucent, and<br>
  the strip under the viewport reads the disc's line off the bill of materials.
</p>

![assembly](https://raw.githubusercontent.com/medinstech/cycloidgen/main/docs/assembly.png)

The 3D view turns on the same crank as the drawing, so the mechanism is the same
mechanism in both. It opens on a quick mesh built from the closed-form profile,
which is what lets it follow a slider; once a design has stood still for a
moment, the solids the STEP file is made of are tessellated in the background
and take its place — the same parts in the same places, with the edge breaks,
the lead-ins and the spline disc the quick mesh does not draw. A build with the
CAD kernel stripped out keeps the quick mesh.

That second picture is the viewport itself, cropped out of the running window
rather than redrawn. On a machine with no OpenGL the same scene comes out of a
software painter instead, which is what the PDF report embeds.

## Run it

Python 3.10 – 3.14, on Windows, Linux or macOS.

```bash
pip install cycloidgen
cycloidgen
```

That is the whole thing, window and all, on all three platforms. It is a large
install — most of a gigabyte, nearly all of it Qt and the OCCT CAD kernel — so a
virtual environment is worth the two extra lines.

There are also [builds that need no Python at all](#standalone-build-and-installer),
on the [releases page](https://github.com/medinstech/cycloidgen/releases/latest).
None of them are signed, so each operating system stops the first launch and
each one wants something different:

- **Windows** — installer. SmartScreen wants *More info ▸ Run anyway*.
- **macOS** — disk image, Apple silicon. Drag it to Applications, then run
  `xattr -dr com.apple.quarantine /Applications/cycloidgen.app` **before you
  open it**. That order matters: on macOS 15 and later, opening it first raises
  a dialog whose default button is *Move to Trash*, and taking it deletes the
  install. There is no *Open Anyway* in that dialog — it appears in *System
  Settings ▸ Privacy & Security* only after a launch has been blocked — and
  right-click ▸ *Open* no longer bypasses it either. With the flag cleared
  first there is no dialog at all. On an Intel Mac, use pip.
- **Linux** — AppImage. One file, `chmod +x`, double-click. If it will not
  start the machine is missing FUSE 2: install it (`libfuse2` on Debian and
  Ubuntu) or run the file with `--appimage-extract-and-run`, which unpacks it
  instead and needs nothing.

Signing is a certificate away rather than a rewrite —
see [RELEASING.md](https://github.com/medinstech/cycloidgen/blob/main/RELEASING.md).

To work on it rather than with it, install the checkout in place:

```powershell
py -3.12 -m venv .venv                          # Windows
.venv\Scripts\python -m pip install -e ".[dev]"
.venv\Scripts\python -m cycloidgen
```

```bash
python3 -m venv .venv                           # Linux, macOS
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/python -m cycloidgen
```

Headless, without opening the window:

```bash
python -m cycloidgen --ratio 29 --out ./my_gearbox
python -m cycloidgen --design saved.json --out ./x --no-solids
python -m cycloidgen --ratio 29 --no-cam-bearing --no-shaft-bearings --out ./x
python -m cycloidgen --ratio 20 --output-from ring --out ./x   # the housing turns
python -m cycloidgen --list-presets                            # what --preset takes
python -m cycloidgen --preset nema23-machined-21 --out ./x     # a whole drive
```

Or put a whole grid of designs through it and get a table rather than a folder
of parts — every combination, every number, for a spreadsheet or a fitting
script:

```bash
python -m cycloidgen --ratio 21 \
    --vary lubricant="None (dry)" \
    --vary lubricant="Grease NLGI 2 (EP, moly)" \
    --vary surface_roughness_um=0.2:12:8 \
    --csv study.csv
```

`--vary` repeats: twice for one field is two values of one axis, once each for
two fields is a grid. Numeric fields also take `lo:hi:steps`. Designs that fail
a check stay in the table with their error codes, because where the feasible
region *ends* is most of what a study is for. Without `--csv` it prints a
summary instead.

Or state what you need and let it find the geometry:

```bash
python -m cycloidgen --optimise --ratio 29 --torque 20 --rpm 1500 \
    --max-od 120 --process "CNC machined" \
    --disc-material "Steel 4140 (hardened)" --rollers --out ./x
```

## Three ways to use it

**Give it a design.** Set the twenty-odd parameters and the app tells you what
is wrong with them — this is the original mode and everything below still
applies to it.

**Start from a preset.** The six ratio presets are geometry and nothing else.
The rest are whole drives built round a motor — a NEMA 17 in a printed drive,
a NEMA 23 or 34 in a machined one, an 8108 outrunner in a robot joint, its can
bolted to a hub on the input shaft, which runs through the motor's hollow
centre — with the motor's curve off its class's
datasheets, the material and the process, a duty it carries, and the parts
round the motor that make it buildable: plates thick enough for the motor's
register to leave the bearing a lip and its screws a full thread. A machined
one is the smallest drive that carries its duty at a safety factor of 1.5 and
outlasts what its motor can put through the ratio: 83 and 90 mm on a NEMA 23,
111 mm on a NEMA 34, 86 mm on the 87 mm outrunner. A printed one is 85 mm, twice
its NEMA 17, with 10 mm discs, and rated at the 0.35 or 0.3 Nm the printed
contact model gives it at the same safety factor - a continuous rating against
an allowable that assumes the plastic creeps. What says it will not break is
its momentary torque, about 10 Nm, over what its motor can put through it.
Each is held by the tests to
analysing with no error, nothing said about its motor, its mounting or its
bearings, a capacity over its rated torque and its motor's margin. The
*Preset* box in the app, or `--preset` on the command line.

**Give it requirements.** Ratio, torque, speed, envelope, process, materials, and
what to optimise for; the search returns a shortlist of geometries that pass
every check, with the trade-offs side by side. `Ctrl+R` in the app, or
`--optimise` on the command line.

**Give it a motor.** State the torque curve and the reduction stops being part
of the question: the app works out which reductions that motor can drive your
load with, searches across them, and ranks the results together. A reduction is
a means, and the job is a torque at a speed. *Work it out from the motor* in the
search dialog, or `--ratio-from-motor --out-rpm 10` on the command line, which
reads the curve off the design you pass with `--design`.

**Give it a grid, or call it from Python.** `--vary` puts a design through
every combination of whatever you name and returns a table. The analysis
underneath is an ordinary importable library — no display, no Qt, no
matplotlib — and [docs/api.md](https://github.com/medinstech/cycloidgen/blob/main/docs/api.md) is what it looks like from the
outside: the spec, the analysis, the exporters, the sweep, the grid and the
search, with every example in it executed by the test suite so it cannot go
stale.

Nine free dimensions is too many to grid, so most of them are *derived* from the
relationships good cycloidal geometry actually obeys — eccentricity from the
shortening coefficient, pin radius from the undercut limit, bore from the shaft,
bolt circle as a fraction of the band that is genuinely available. What is left
is six continuous knobs and two discrete ones, screened in closed form before
anything expensive runs. A search that finds nothing reports *why*: `1,400 of
1,500 candidates were over your diameter limit` is a usable answer, an empty
list is not.

## What it produces

Four groups, chosen together or separately — in the app's **Outputs** tab, on
the command line with `--only`, or from the **Export ▾** menu in the header,
which writes everything or everything but the solids.

| File | Group | Contents |
|---|---|---|
| `NOTICE.txt` | always | What these files are not: preliminary sizing from stated models, geometry that has not been proven against a made part. Written into every bundle whichever groups are selected, because it is the sheet that travels with a STEP file to somebody who never ran the app |
| `disc.dxf` | drawings | Drawing of the whole drive: disc profile as a closed LWPOLYLINE sampled to the chord tolerance, plus bore, output holes, ring pins and housing, on separate layers |
| `disc.svg` | drawings | Same drawing, 1 unit = 1 mm |
| `dxf/` | drawings | One cutting file per part — each disc on its own, the ring plate, a carrier drilling template drilled for the *press fit* rather than for the disc's running hole, and the two end plates with their tie bolts and motor pattern |
| `sketch/` | drawings | The same disc outline as **one closed cubic spline** instead of the cutting file's ~1,800 vertices — a curve a CAD package can offset, dimension and build features against. Fitted to the design's own chord tolerance and stamped with the deviation it reached, with the centre marks and bolt circle the disc never had. Not a cutting file: two outlines in one file is a part cut twice |
| `sheets/` | drawings | **Dimensioned drawings**, an A3 PDF per part for the shop that makes it: a view from outside and a section, every size, which face a seat is bored from and how deep, and the tolerances that matter — each bearing seat's ISO 286 fit, chosen from whether its load turns round the outer ring, and each hole pattern's position tolerance, from the hole and the screw through it. Datums, a title block, ISO first angle. The two end plates so far |
| `assembly.step` | solids | Full gearbox: housing and its two end plates, ring pins, phased discs, eccentric shaft, output flange with its bearing boss, every bearing that fits, coloured. Bought parts drawn as the parts: a ball bearing as two rings with raceways and a pitch circle of balls, a drawn cup as its cup and needles, the tie bolts threaded with a broken head and a drilled socket, the nuts chamfered. Inside corners where a bearing ring sits are radiused under the ring's own chamfer |
| `step/` | solids | Each part as its own solid, in its own frame, carrying its own name and colour inside the file — so a part imported into your own assembly arrives as `housing` rather than as whatever the kernel calls it. A disc's flank is the same spline as `sketch/`, not facets: one face per flank, with a vertex on every lobe root and crest to dimension from |
| `stl/` | solids | Each part separately — STL has no assembly structure. A multi-disc stack gets `disc_1.stl`, `disc_2.stl`, … because **the discs are not the same part** (see below). A printed part — PLA, PETG, ABS, PA12 or resin — comes ready to print: laid on whichever face leaves the least to support, standing on the bed, and on FDM with its bed edge broken back 0.3 mm so the squashed first layer does not stand proud of the profile |
| `assembly.3mf` | solids | The same meshes in one container, with what STL cannot carry: every part where it assembles, in its own colour, named for its material. Identical discs are one object placed twice; different ones are two objects. Made parts only — the bearings and the bolts are bought |
| `bom.csv` | data | Every part: quantity, material, size, mass, make or buy, and the bearing designations the sizing study picked |
| `report.json` | data | Every parameter, derived value, load, stiffness, temperature, mass and finding as plain data |
| `report.pdf` | data | Drawing and 3D view, geometry, checks, contact stress, stiffness and backlash, PV and temperature, mass, bill of materials, bearings, and an exploded build order |
| `motion.gif` | animation | The drive turning, looping, with contacts and contact forces — for a document, a chat or an issue. See below for why it loops without a jump |

The **Outputs** tab lists all of that for the current design *before* anything is
written — every file by name, what it is for, and where it will land — then
fills in the sizes afterwards and opens any of them on a double-click. The same
list is available headlessly:

```bash
python -m cycloidgen --ratio 29 --list-outputs
python -m cycloidgen --ratio 29 --only drawings,data --out ./x
```

That list is not a second copy of the truth. `cycloidgen/export/manifest.py`
declares every deliverable once; `write_bundle`, the Outputs tab and
`--list-outputs` all read it, and a test compares the declaration against the
files that actually appear on disk — including this table.

**Every file says which build wrote it.** A saved design, the JSON report, the
PDF and every DXF carry the version, because none of the numbers here are
measurements: they are a model's answers, and the model gets better between
releases. So a design saved by an earlier build says so when you open it —
which quantities may have moved, and that the inputs still mean what they
meant — instead of quietly reporting a different mass than the one you wrote
down. [RELEASING.md](https://github.com/medinstech/cycloidgen/blob/main/RELEASING.md) says which version digits are allowed to
move a number and which one is defined as the digit that cannot.

**What the drawing sheets add, and where it comes from.** A cutting file is an
outline and its holes; it cannot say that a seat is bored from the inside face
to a depth, or what the bore a bearing is pressed into is held to. The sheets
in `sheets/` do, and every tolerance on them is derived rather than picked:

- *A bearing seat's fit follows from whether its load turns round the outer
  ring.* A ring whose load turns rolls round its bore and creeps unless
  interference holds it; one whose load stands still may slide. In a cycloidal
  drive every internal load points where the crank points, so every seat in the
  housing sees its load turn - the shaft supports included, which in most
  machines are the textbook stationary case. They get N7, or M7 where the load
  is light (at or under 0.05 C, the lower of the makers' thresholds). The limits
  are worked out from ISO 286-1's rules - a grade's width and the deviation
  that places it - and a test holds them to ISO 286-2's printed tables.
- *A hole pattern's position tolerance is the clearance its screw has.* Holes
  of `H` in two parts take a screw of `F` however far apart their axes are, up
  to `H - F`; so each hole may sit anywhere in a zone that wide, at maximum
  material, where the screw floats through every part, and half that where its
  far end is held in a thread. The test checks it by sampling the screw through
  two holes offset by the zone, and a hundredth more.
- *The section is the solid's section.* It is drawn from the same numbers the
  sheet dimensions, and a test asks the OCCT solid, point by point either side
  of every line, whether there is metal where the drawing says there is; a line
  drawn a quarter of a millimetre off shows up.

What they do not do yet: only the two end plates have a sheet. The fits are the
makers' tables for a steel or cast-iron housing, and an aluminium plate grows
away from its bearing as it warms - the sheet says which table it used and does
not adjust for it. A printed part gets no fit at all, because a printer holds
none; its sheet says so and states the clearance the design assumed instead.

**Why the animation loops without a jump.** A GIF restarts whether or not the
mechanism is back where it started, and one that is not reads as a fault in the
drive rather than in the file. Different parts close at different times. The
disc, the ring contacts and the force arrows close after a *single* input turn —
one turn walks the disc on by exactly one lobe pitch, and a profile with `N`
lobes is unchanged by that rotation. The output carrier does not: it advances
`360/N` per input turn and repeats every `360/output_pin_count`, so it needs
`N / gcd(N, output_pin_count)` turns — five for the 15:1 preset, seven for the
21:1, fifty-nine for the 59:1. When the exact period does not fit the frame
budget the answer is *not* as many turns as will fit: any whole turn leaves the
disc closed, so the run to choose is the one whose carrier lands closest to a
hole pitch, and at 29:1 that is five turns (2.1° out) rather than ten (4.1°).

Making that true of the pixels rather than only of the arithmetic took two
fixes, both of which are improvements in their own right. The chord-tolerance
sample count is now rounded up onto a whole number of samples per lobe, so the
sampled polygon has the disc's own symmetry instead of sampling each lobe from a
slightly different place. And the disc outline is drawn as a closed path rather
than a polyline with its first point repeated, so its seam is a join like any
other — a seam travels round the rim as the disc turns, and it was changing a
handful of pixels every frame at a place where nothing was happening.

## The geometry

With `R` the ring-pin circle radius, `Rr` the pin radius, `E` the eccentricity,
`N` the lobe count, `Np = N+1` pins and `K1 = E*Np/R`:

```
D(t) = sqrt(1 + K1² − 2·K1·cos(N·t))
x(t) =  R·cos(t) − E·cos(Np·t) − Rr·(cos(t) − K1·cos(Np·t)) / D(t)
y(t) = −R·sin(t) + E·sin(Np·t) + Rr·(sin(t) − K1·sin(Np·t)) / D(t)
```

The output pin holes are the pin diameter plus twice the eccentricity. The
generating curve is a hypotrochoid, the disc flank its equidistant offset at
`Rr`, which is what "rolls on the inside" means in the formulae above — the sign
of `y(t)` is not free.

## Which member is the output

A cycloidal drive is a **three-shaft** machine, not a two-shaft one. The crank
is always the input; the ring the pins sit in and the carrier the pins through
the disc holes stand on are interchangeable. Ground either one and the other is
the output — and which you ground is a design decision, not a point of view.
Both are built and sold, so `output_member` picks between them and everything
downstream follows.

| | **Carrier output** (`output pin carrier`) | **Ring output** (`ring housing`) |
|---|---|---|
| Epicyclic name | planetary | star |
| Bolted down | ring housing and both end plates | the frame: base, carrier, pins, end cap |
| Turns | carrier, pins and boss | housing barrel and both end plates |
| Reduction | `N` (the lobe count) | `Np = N+1` (the pin count) |
| Direction | against the input | with the input |
| Motor bolts to | the input end plate | the carrier's base |
| Load attaches to | the boss on the axis | bolts through the input end plate |
| Output pins | cantilevers off one plate | beams between two |
| Housing carried by | — (it is the ground) | a bearing at each end of the frame |
| Cam bearing speed | `(N+1)/N ×` input | exactly the input speed |
| Disc rotation in the ground frame | `1/N` of the input | none — it only orbits |

The extra tooth of reduction is free: the *same parts*, bolted down at the other
end. What it costs is the shaft end — a turning barrel has nowhere to grip, so
the load bolts to a face.

**The frame.** With the ring as the output the grounded member is not a plate
but a cage, and it has to be: the pins standing on the carrier need something to
land in, and the barrel turning around them needs to be held at both ends. So
the carrier grows a base on the outside and an **end cap** on the far end of the
pins, and those pins become the frame's own fasteners — which is exactly how the
printed micro drives of this kind are built. Two things fall out of it and
neither is cosmetic:

- **The output pins stop being cantilevers.** A pin built into one plate carries
  `F·L`; the same pin caught at both ends carries about `F·a·b/L`. On the 21:1
  preset with two discs that is 45.3 MPa of fully-reversed bending against
  19.9 MPa — a factor of 2.3 on the part whose fatigue check is usually the
  first thing to fail.
- **The housing is carried rather than hung.** One bearing locates a barrel; it
  does not hold one against a moment, and a wheel or a pulley on a turning
  barrel is exactly a moment. There is a main output bearing on each of the
  frame's two bosses, and the two shaft supports move inside them.

It costs length — the barrel has to cover the cap, so a 21:1 goes from 40 mm to
63 mm — and it costs the end cap itself.

Two things follow from grounding the other member and are worth stating because
they are easy to get wrong. First, every relative motion inside the drive is
**unchanged** — grounding a member adds one rigid rotation to all of it at once,
which is exactly what `GearSpec.frame_spin` is, and it is why the 3D view draws
one mechanism from one mesh. Second, the crank is then no longer turning at the
input speed: it runs at `(N+1)/N` of it, so every sliding speed in the drive
passes through `GearSpec.crank_rate` on its way from rpm to mm/s.

The motor face has to move, and that is not cosmetic — a motor bolted to a plate
that turns is a gearbox nobody can build. So on a ring-output drive the carrier
grows a base at the end of its boss, the motor's pattern and register go into
that, and the input end plate gets an output bolt circle instead. The base is
a pad the motor's size and an end plate's thickness - what its register, its
screws' seats and the shaft support's lip need - with a flange round it the
housing's size and only as thick as the four screws that bolt it down need,
tapped up into it from the machine's side on the motor's flats, where they
are out of its corners' way. Through a coupling the motor stands on a mount
and the base stays a whole plate. The boss
stands only as far past the output end plate as the tie screws' heads on that
turning plate need to clear the base, and a millimetre more — off the carrier
it stands 8 mm, for a coupling to grip; `output_boss_protrusion` overrides
either. Those output
bolts share the tie bolts' circle, because that is the one radius on that plate
with barrel wall behind it to thread into; they are held half a pitch away from
them, and `OUTPUT_BOLT_CLASH` says so when the two counts do not interleave.

The tie bolts themselves are socket head cap screws, named by the size their
hole is the clearance for — a 4.5 mm hole is an M4 — at ISO 4762's next
preferred length past the stack, a nut and two threads. Heads go on the output
end plate and nuts on the input one: a counterbore will not fit the wall the
bolt circle runs down, and the input plate's outside is where a motor goes. On
a ring-output drive that face is the one the load bolts to instead, so the input
plate is tapped for the screws and there are no nuts: each screw is the longest
preferred length that has one and a half diameters of thread in a metal plate,
or two in a polymer one, and stops half a millimetre short of the face. A plate
too thin for that keeps its nuts, and `TIE_NUTS_ON_OUTPUT_FACE` says how far
proud of the face they stand and how thick a plate would take the thread.

A printed part's own thread strips long before the screw is loaded, so a
polymer drive can ask for heat-set inserts (`heat_set_inserts`): knurled brass,
pressed in hot, wherever a screw threads into plastic — the tie screws into
the turning plate, from the face that meets the barrel, and the four that bolt
the base down, from below. The screw then needs only one and a half diameters
in it, as in metal. An insert's hole is wider than the screw's and wants
1.5 mm of plastic round it, which a round housing's bolt circle down the middle
of a 6 mm wall has not got; there the plate is tapped as before and
`INSERTS_DO_NOT_FIT` names the wall, or the square housing, that would take
them. The base's flange thickens to the insert's length, half a millimetre for
the plastic it displaces and half a millimetre over the hole's end.

**What puts the plates on the ring's axis is two dowels at each end, not the
bolts.** The plates carry the shaft supports and the output bearing, so they
decide where the crank's axis is, and the ring pins are in the barrel; a bolt
in a clearance hole locates nothing, and an M4 in 4.5 mm lets a plate sit half
a millimetre off - against a machined drive's few hundredths of profile
clearance. So each joint has two ISO 8734 dowels on the tie-bolt circle, the
largest that leaves a quarter of the wall either side (3 mm in the default
6 mm wall), pressed into the barrel and a tight slide in the plate. They sit a
quarter of a tie-bolt pitch round, clear of the tie bolts and of the output
bolts half a pitch round, and the second is placed clear of the motor's
pattern and never where a turn or a flip would bring the pair back onto
itself: a plate goes on one way only. Machined, the holes are reamed on
assembly with the barrel, the plate's seat indicated concentric with the pin
circle within the profile clearance; printed, the plate's holes are printed
the design's hole allowance over the pin. `plate_dowels` turns them off, and
`PLATES_NOT_LOCATED` then says how far off the crank's axis can sit.

**A square housing** (`housing_shape`) is the outline a gearhead on a stepper
has: flat sides to bolt to or stand it on, as wide across the flats as the round
housing is across, the corners rounded to the wall. Its tie bolts go into the
corners - four of them, whatever `housing_bolt_count` says - each on its corner
arc's centre, where the arc and both flats are a wall away. A bolt there has the
wall less its own radius round it rather than half of what is left either side
of a bolt down the middle of the wall: 3.75 mm in the default wall, against
0.75. The output bolts and the dowels stay on the wall's middle circle, the first
output bolt on a flat; the dowels are placed round that circle, the first where
there is most metal and the second nearest opposite, never where one of the
eight ways a square plate turns or flips onto its bolts would bring the pair
back. Mass, cooling area and inertia are worked from the outline's own area,
perimeter and polar moment; the ring's torsional stiffness is still that of the
round it contains, which the corners only add to.

> **Sign warning.** The equivalent `psi` formulation needs a *leading minus*:
> `psi(t) = -atan2(sin(N·t), R/(E·Np) - cos(N·t))`. The positive-sign variant is
> widespread online and is wrong - it deviates by millimetres and the disc
> interferes with the pins by about 1 mm, which seizes the drive. `tests/test_profile.py`
> pins both the correct form and the trap.

Everything above is verified numerically rather than trusted:

- **Envelope property** - every profile point sits exactly `Rr` from the
  pin-centre locus (deviation 0.0000 µm). This is what makes the disc roll
  instead of dig in.
- **Meshing sweep** - a full input revolution runs with under 2 µm residual
  interference, while a ratio one tooth away jams by 450-730 µm. That proves the
  profile and the motion law together.
- **Undercut limit** - `Rr < 1/max(-kappa)` of the locus. The locus curvature
  collapses to `(A + B·u)/(C − D·u)^1.5` with `u = cos(N·t)`, so the extreme is a
  closed-form root rather than a 40 000-point scan — which is what makes the
  design search affordable. Cross-checked against the brute-force result.
- **Assembly fit** - every output pin stays inside its hole through a full
  revolution, on every disc; the disc clears the housing bore.

## Clearance: which way the offset goes

Both clearance levers have to *shrink* the disc. It reaches `R − Rr + E` from its
own centre, so growing the generating roller (`equidistant`) and shrinking the
generating pin circle (`pin_circle`) each pull the profile inward. Two of the
three modes originally had the pin-circle sign backwards and cut a ~200 µm
*interference* instead of a clearance; `both` cancelled itself out to almost
exactly zero. Nothing noticed, because every other test ran on the default mode.

So the clearance is no longer assumed — it is **measured**, as the distance from
each pin centre to the manufactured profile less the pin radius
(`core.kinematics.mesh_gaps`). `PROFILE_INTERFERENCE` is an error and
`CLEARANCE_NOT_DELIVERED` a warning, and every mode is tested to open a real gap.

That measured gap then feeds the load model, which answers the question the old
`README` could only flag as a limitation:

- an **equidistant** offset gives every pin the same normal gap, so a contact
  only comes into mesh once the disc has turned far enough to close it. Contacts
  with a short moment arm need much more rotation than the long ones — so at low
  torque a handful of pins carry everything, and the peak force is several times
  the ideal share. Capacity is derated accordingly — at the capacity itself,
  the torque where the busiest pin reaches its allowable *with the pins sharing
  as they do at that torque*. The safety factor is read at the rated torque,
  with that torque's sharing, and the two are not one number scaled: a steel
  drive rated at a tenth of its capacity has twice the pins in mesh at the
  capacity that it has at its duty.
- more torque pulls more pins in; a tighter process pulls more pins in. Both are
  visible in the datasheet as *pins carrying load*, and both are tested.

## Multi-disc stacks: the discs are different parts

A disc sitting on crank phase `p` has to rotate by `p/N` to stay meshed with the
ring. But every disc is coupled to the *same* output carrier, so every disc must
also share the carrier's rotation. Both can only hold if each disc's output-hole
pattern is turned back against its lobes by `-p/N`.

So in a two-disc drive the second disc is the first one with its holes moved half
a lobe pitch. They are **not interchangeable** — swapping them jams the drive.
They would only be identical with `2*N` output pins, which is normally far too
many, and the app tells you (`DISCS_DIFFER`) when that is not the case. The
per-part files, the STLs and the bill of materials all keep them apart.

This was wrong in the first version and is now pinned by `tests/test_assembly.py`.
The visible symptom is worth knowing: in the drawing, the green output pin must
stay inside its blue hole at every crank angle. When it pokes out, the hole
pattern is at the wrong angle.

## What it tells you

Beyond the geometry checks, every design gets a datasheet:

- **Torque capacity**, both on the ideal load share and derated for what
  clearance actually does to it.
- **Momentary torque** — what it survives for a moment rather than runs at: a
  hard stop, a crash, the motor stalled against something that will not give.
  The torque at which the first part starts to yield, and which part: the ring
  contact and the output pins in their holes at 1.67 times the softer body's
  yield (a line contact's first yield is below its surface — Johnson, *Contact
  Mechanics*, §4.2), the output pins in bending, the disc web in shear, and
  every bearing the torque loads against its static rating. With a motor
  stated, `MOTOR_OVERLOADS_DRIVE` says when it can put more than that through
  the ratio standing still, and what current to hold the driver to.
- **What the motor can actually deliver**, if you state one. Everything else in
  the datasheet is what the drive can *take*; this is what it will be *given*,
  and on most designs here it is the smaller number. A motor is a curve rather
  than a number, and the number it is sold on is the one at standstill: torque
  falls with speed because the supply runs out of voltage to force current into
  the winding against its own back-EMF, and it reaches zero at a speed set by
  the bus as much as by the motor — halve the supply and you halve the top
  speed. Out of it come the two ceilings a design should be read against: the
  output torque this motor and this reduction can deliver, and the output speed
  past which they cannot.
- **Torsional stiffness** in Nm/arcmin and **lost motion** in arcmin, from
  Hertzian line-contact springs (Johnson's elastic approach) on the ring and
  output stages, in series with the parts they are mounted in — the ring pin
  seats, the carrier pins in bending, the carrier plate, the disc body, the
  housing barrel and the input shaft. Every one of those is reported on its own
  line, because "everything else" as a single number is a number to distrust,
  and because on most designs the softest part is a surprise. On a printed drive
  the two halves are comparable, which already costs the answer a third. The
  better the mesh, the worse the imbalance: a ground steel drive stiffens its
  contacts by two orders of magnitude and its cantilevered carrier pins by
  nothing at all, and ends up an order of magnitude softer than its own mesh.
- **Transmission error** — the ripple in output angle under a steady load, which
  is the number that decides whether a drive can *position*. Lost motion is the
  play before the output moves; this is what it does once it is moving, as the
  mesh hands load from one contact to the next. Each stage is swept over its own
  period, and they are not the same: the output stage repeats every
  `2·pi·N/(n·(N−1))` of crank, about two and a half lobe pitches, and measuring
  it over a lobe pitch instead reports about half the ripple that is there.
  A phased disc stack cancels much of it — the discs ride opposite halves of the
  same cycle — which a stiffness average cannot see, because phasing leaves the
  mean alone and only moves the ripple.
- **What your tolerance costs.** Everything else in the app places the pins
  exactly, and with a uniform clearance that means they all come into mesh
  together. Enter the true position your shop holds and the load sharing is
  solved over a *batch* of rings drawn from that tolerance zone — a single
  number does not say where each pin went, worst case is a ring nobody will
  build, and nominal is a ring nobody has built. Out comes the middle ring and
  the bad one: stiffness, load concentration, lost motion and transmission
  error, each with its tail. Past the point where the tolerance approaches the
  clearance the pins interfere and the drive binds, and the app says so rather
  than quietly reading an interfering pin as a free preload.
- **PV and running temperature.** PV is the wear limit, and it is what actually
  finishes a printed drive — a PLA disc can sit well inside its stress allowable
  and still wear round in an afternoon. Quoted on the projected-area convention
  the published limits use, *not* the Hertzian peak; the two differ by several
  times and are not interchangeable.
- **Mass, inertia, unbalance.** Off the real lobed section, not a cylinder,
  with the bought parts on the scale too: every bearing, estimated from its
  envelope, and the tie bolts and nuts, from their ISO dimensions — each held
  to published masses in the tests — and the plate dowels, which are ground
  cylinders. The inertia the motor turns, in three
  terms: the shaft and its cams, the discs, and the output member through the
  square of the ratio — where a carrier is almost nothing and a turning
  housing is most of the total. And the shaking force a single disc throws —
  the disc, the cam it rides and the bearing between them, all orbiting
  together — rising as the square of speed. Evenly phased discs cancel the
  force and leave a couple.
- **Disc web stress.** The ligament beside the output holes is the thinnest
  structural member in the drive and nothing else was looking at it.

## Checks

Seventy-four of them. Errors block export; warnings do not; several are readings
rather than tests.

**Profile** — `K1_TOO_HIGH` · `K1_HIGH` · `UNDERCUT` · `UNDERCUT_MARGIN` ·
`PROFILE_SELF_INTERSECT` · `PROFILE_INTERFERENCE` · `CLEARANCE_NOT_DELIVERED` ·
`PIN_RADIUS_SUGGESTION`

**Layout** — `PIN_OVERLAP` · `PIN_SPACING` · `HOLE_HITS_BORE` ·
`HOLE_BREAKS_RIM` · `THIN_INNER_WEB` · `THIN_OUTER_WEB` ·
`OUTPUT_HOLES_OVERLAP` · `OUTPUT_HOLE_SPACING` · `ECCENTRIC_TIGHT` ·
`DISCS_DIFFER`

**Manufacturing** — `CLEARANCE_DEFICIT` · `HOLE_CLEARANCE_DEFICIT` ·
`TOOL_RADIUS` · `PIN_POSITION`

**Load and stress** — `PRESSURE_ANGLE` · `HERTZ_STRESS_RING` ·
`HERTZ_STRESS_MARGIN` · `HERTZ_STRESS_OUTPUT` · `LOAD_CONCENTRATION` ·
`WEB_SHEAR` · `WEB_SHEAR_MARGIN` · `FATIGUE_LIFE` · `FATIGUE_MARGIN` ·
`FATIGUE_NOT_MODELLED`

**Precision** — `TORSIONAL_STIFFNESS` · `STRUCTURAL_COMPLIANCE` ·
`LOST_MOTION` · `TRANSMISSION_ERROR`

**Wear, heat and life** — `LOW_EFFICIENCY` · `LUBRICATION_REGIME` ·
`PV_LIMIT_RING` · `PV_MARGIN_RING` · `PV_LIMIT_OUTPUT` · `PV_LIMIT_CAM` ·
`OVERTEMP` · `RUNNING_HOT` · `SHORT_BEARING_LIFE` · `NO_BEARING_FITS` ·
`BEARING_DOES_NOT_FIT` · `BEARINGS_OMITTED` · `SEAT_HAS_NO_SHOULDER`

**Mounting** — `HOUSING_BOLT_CLASH` · `OUTPUT_BOLT_CLASH` ·
`MOTOR_SHAFT_TOO_BIG` · `MOTOR_SHAFT_MISMATCH` · `MOTOR_SHAFT_SHORT` ·
`MOTOR_SCREW_SEAT` · `MOTOR_HOLLOW_TOO_SMALL` · `COUPLING_DOES_NOT_FIT` ·
`MOTOR_FACE_CLASH` · `MOTOR_RADIAL_LOAD` · `TIE_NUTS_ON_OUTPUT_FACE` ·
`INSERTS_DO_NOT_FIT` · `PLATES_NOT_LOCATED` · `PLATE_DOWEL_CLASH`

**The motor** — `MOTOR_OPERATING_POINT` · `MOTOR_TORQUE_SHORT` ·
`MOTOR_SUPPLY_VOLTAGE` · `MOTOR_OVERLOADS_DRIVE`

**The duty cycle** — `DUTY_CYCLE` · `DUTY_RATING_MISMATCH` ·
`DUTY_MOTOR_SHORT` · `DUTY_BEARING_LIFE`

**Dynamics and mass** — `SINGLE_DISC_UNBALANCE` · `UNBALANCE_FORCE` · `MASS`

Selecting a check in the app highlights the parameters it is actually about, so
you are not left guessing which of twenty-odd numbers it wants you to change.
That list is not maintained by hand: the codes are parsed out of the calls that
raise them, and a check added without an explanation fails the suite.

`PIN_RADIUS_SUGGESTION` is worth knowing about: the equivalent contact radius
works out to `R_eq = Rr·(1 − Rr/rho_c)`, a parabola peaking at `Rr = rho_c/2`.
Contact stress is therefore lowest at half the critical pin radius, and the app
suggests it.

## In the app

- **Drawing** — the live 2D simulation. A crank slider and playback at a chosen
  speed, and four overlays that can be turned on independently: contact points
  sized by the share of load they carry, contact forces to scale, the traced
  path of a point on the disc rim, and ring pin numbers. Input and output angles
  are read out under the drawing, so the reduction is something you watch rather
  than something you are told. A pinned reference design shows underneath as a
  ghost outline.

  ![the drive turning](https://raw.githubusercontent.com/medinstech/cycloidgen/main/docs/motion.gif)

  Everything drawn comes off `core.kinematics`, the module that was verified
  against a full-revolution meshing simulation — the picture cannot tell a
  different story from the datasheet. Force arrows are scaled against the worst
  force over a whole lobe pitch rather than against the current frame, so an
  arrow that grows means the load grew and not that the scale moved under it.

  That animation is a file the app writes, not a screen recording: seven input
  turns of the 21:1 preset, which is exactly its period.

- **3D** — the assembled drive on the same crank. Drag to orbit, right-drag to
  pan, wheel to zoom, standard viewpoints, an explode slider, per-group
  visibility so you can take the housing off and watch the mesh — the tab opens
  with the end plates already off, because with them on the assembled view is a
  closed cylinder with a shaft out of one end — and a capped
  section plane — the cut reads as solid material with the cut faces a shade
  darker, not as a hollow casting. Turning **Edges** on draws the part's
  features — rims, hole lips, the join between a cylinder and its end — and not
  the triangulation underneath them.

  Click a part and it is tinted, and the strip under the viewport reads its
  line on the bill of materials: what it is, how many, out of what, how big,
  what it weighs, whether it is made or bought, and the note that goes with it —
  the input end plate is one of two "Housing end plates", bored for the shaft
  support and not interchangeable with the other. A drag still orbits; only a
  click that does not move picks.

  **X-ray** turns the housing and its end plates translucent, so the stack, the
  pins and the bolts read through them without taking anything away, and a
  click goes through them to the part behind.

  **Measure** puts the clicks on the model instead: two points on any surface,
  the line between them with its length written beside it, and the distance
  and its three components under the viewport, in the units the rest of the
  window reads in. A third click starts again, and anything that moves the
  parts clears it rather than leave a line between two places nothing is any
  more. Like the section plane it needs the hardware view, which knows where in
  the model a click landed.

  **Clashes** tints red every part a clash check is about — a profile into its
  pins, a hole into a bore, a bolt into a pocket, a motor's spigot onto a
  bearing — and picking one of them adds what the check says to the strip.
  Selecting a check in the list does the same for that check alone. It reads
  the checks rather than testing the meshes against each other, because every
  bearing in its seat and every pin in its pocket touches by design, and
  telling a fit from a clash is what the checks are for.

  The bearings are in there too, at the size the sizing study picked rather than
  the size of the space they go in, and as bearings — rings and balls, or a
  drawn cup and its needles: one on the cam inside each disc bore, rollers over
  the ring and output pins where a roller was selected, and the two shaft
  supports where the end plates belong. Drawn, a bearing is about two thirds
  steel and a third room, which is the share the mass model weighs it at off
  published masses; a test holds the two to each other. Where a bearing is not drawn it is
  because the model has nothing for it to sit in, and its note in the schedule
  says which case it is.

  They also switch off one at a time, from the **…** beside the group: they sit
  in four different places, and looking at the cam bearing down its bore means
  putting the others away rather than putting all of them away.

  **Mounting** says how it bolts to the world. Pick a motor frame and the input
  end plate gets that frame's register and bolt pattern — a *square* for a NEMA,
  because four holes on a circle of the same size land where the motor has
  nothing — and the app checks what that pairing can get wrong: a pattern that
  falls into the bore or off the rim, and, when you have chosen to let the motor
  carry the crank, whether its own bearings are up to it. The motor is drawn
  too, as a part of the view rather than of the drive: a body the frame's size
  standing off its face, its spigot in the register and its shaft turning with
  the input - and on the bill of materials, at no mass, because someone has to
  buy one.

  Driven directly, the motor sits flush on that face and its shaft goes into
  the drive's: the input shaft stops short of the spigot and is bored from that
  end for the motor's shaft, a D on the motor's D, so the torque goes through
  the flat and not through a set screw nobody can reach once the motor is on.
  The checks are that the input shaft keeps a wall round the bore - an
  error where the motor's shaft is the thicker and there is no bore to cut -
  and that enough of the motor's shaft gets into it. The motor's screws are let into
  the face the discs run against, because that face has nothing to spare: a
  NEMA 8 to 17 is threaded, so its screws go in from the drive's side with
  their heads in counterbores; a NEMA 23 or 34 has a flange, so they go in
  from the motor's side into nuts sitting in hexagonal pockets. Both are ISO
  screws at the longest preferred length that does not bottom out, on the
  bill and weighed, and a plate too thin to hold them is a finding.

  Through a coupling - *motor drives the shaft* off - the motor stands on a
  mount instead: a tube round the coupling, part of the plate it rises from,
  with a window either side for a key to reach the coupling's screws, and on
  its end a flange the motor's size carrying its pattern and its spigot. The
  coupling is the smallest of the common beam sizes that takes both shafts, or
  a plain sleeve, reported, where none does; the drive's shaft runs into its
  near half, and the motor stands off far enough for its own shaft to meet it
  in the middle. Nothing is let into the face the discs run on: the screws'
  heads or nuts sit under the flange, and on a threaded face the flange is as
  thick as makes the shortest preferred screw through it fill the motor's
  threads. The mount is weighed with its plate, the coupling as aluminium and
  in the input's inertia, and both are on the bill. The checks are that the
  tube clears the motor's screws and that enough of the motor's shaft gets past
  the flange into the coupling.

  The output end of this topology is a boss on the axis rather than a bolt
  face, so what it needs is somewhere for a coupling to grip; it now stands
  proud of the plate instead of coming out flush.

  That is the *picture*. Whether the drive actually has them, and which ones,
  is a design decision, under **Bearings** in the parameter panel. Each seat can
  be left on `auto` — the smallest catalogue part that fits it and lasts the
  life you ask for — or given a designation, which is what you want when a
  bearing you already own is doing the deciding. A named part is checked against
  its seat and never quietly swapped for one that fits; if it will not go in,
  the check names the dimension that is wrong and the 3D view leaves it out
  rather than drawing it shrunk to size.

  Whichever part goes in, its seat is bored to it. A seat is sized as *room* and
  the part chosen is the smallest that fits the room, which is rarely the
  room's own size — a 19 mm 6800 for a 22 mm shaft-support seat, a 24 mm HK1808
  for the 21:1's 26 mm disc bore — so the disc, the plates, the carrier's boss
  and the end cap are cut to the outside of the bearing in each, in the STEP,
  the meshes, the 3D view and the cutting DXFs alike, and weighed that way. A
  seat left without a bearing keeps its room.

  And each seat stops short of the outside face. What is left is a lip the
  outer ring bears on, its hole at the bearing's pitch circle so that it clears
  the inner ring turning against it; the main output bearings sit against the
  inside of their plates so that the lip gets all the spare length, and the
  part that turns with the inner ring is the shoulder on the other side. A seat
  with no room left for a lip - a plate thinned to the bearing's width, or a
  motor's register cut into the same face - is reported rather than drawn
  without one.

  The shaft is located against those lips by two sleeves, one from each end
  cam to the support beyond it, so thrust either way runs cam, sleeve, inner
  ring, outer ring, lip; and a washer between each pair of discs holds the gap
  the stack was designed with, reaching from outside the disc bore to inside
  the output holes however the discs have orbited. All three are made parts,
  with their own files and their own lines on the bill of materials.

  Where the shaft stands out of the input plate for a coupling it carries a
  flat, a tenth of its diameter deep - a stepper's own D-cut on a 5 mm shaft is
  half a millimetre - so a set screw drives through a face rather than biting
  a round shaft and walking. A ring-output drive gets none, its driven end
  being inside the carrier's boss where no set screw can reach - unless a
  motor is coupled to it, and then the flat is on the end that runs down
  through the carrier's base into the coupling.

  Three of the five load paths can also be built without a bearing at all, and
  plenty of drives are. Switching one
  off takes the part out of the schedule, the BOM, the STEP and the 3D view, and
  changes the physics with it: a drive with no cam bearing runs its disc bore
  straight on the cam, which grows to fill it, and pays the sliding coefficient
  at the fastest contact in the machine. The load path stays on the schedule
  either way, saying what is carrying it instead — a motor's own bearings do not
  stop the crank reaction existing.

  A seat with no bearing in it also says what is *between* its surfaces. It is a
  plain sliding contact, the film across it is already solved to get the
  efficiency its friction coefficients, and the regime it lands in is what
  separates an L10 column reading `—` from a wear rate: on a ground steel drive
  with moly grease in it the cam journal floats and the fixed ring pin builds a
  film an order of magnitude short of the roughness it has to clear. A seat
  with a bearing *in* it is given no such sentence, because what that would take
  is ISO 281's `a_ISO` — published curves over a viscosity ratio that needs a
  reference-viscosity chart of its own, none of which is here, and a factor
  invented to stand in for it would be exactly the kind of number this refuses.

  ![exploded](https://raw.githubusercontent.com/medinstech/cycloidgen/main/docs/exploded.png)

  It renders on the GPU through VTK — which costs no new dependency, because
  the CAD kernel that writes your STEP files already brings it. Depth buffer,
  smooth shading - off feature-angle normals on the quick mesh, and off the
  solid's own surface normals, as OCCT works them out, on the CAD one, so a
  flat face is flat to the last digit and reflects as one - screen-space
  ambient occlusion sized off the drive so a pin sits
  *in* its pocket, and multisampled edges. The shading is physical: every
  material has a finish, a metal is drawn in its own colour — ground 4140 dark,
  polished bearing steel bright, milled aluminium pale — and a printed part
  keeps its diagram colour with its material's surface, PA12 chalky and PETG
  glossy. A metal looks like metal only by what it reflects, so the view lights
  it with a photographic studio — a key, a fill and a rim strip — generated in
  code rather than shipped as a picture. `CYCLOIDGEN_VTK_PBR=0` goes back to
  plain Phong shading in diagram colours. Geometry is uploaded once per
  design; turning the crank sets one transform per part, so a frame is bounded
  by the display's refresh rate and a 59:1 drive with sixty pins costs exactly
  what a 15:1 one does.

  There is a **software fallback** — the same scene, back-face culled and
  painted back to front with `QPainter` — for a build with the kernel stripped
  out, a machine with no OpenGL, or a remote session that will not forward it. A
  flat-shaded gearbox beats a tab that shows an error. It is also what draws the
  3D views in the PDF, where a vector figure is worth more than a screenshot,
  and what makes the projection testable on a machine with no display at all.

  **macOS gets a different viewport, not a different renderer.** VTK's own Qt
  widget builds its GL context directly on the view `winId()` returns, with
  `WA_PaintOnScreen` set — an attribute Qt documents as X11-only, on a platform
  whose views have been layer-backed and mandatorily so since 10.14. The first
  render blocks the main thread as the tab opens and takes the application with
  it, and because that is a process dying rather than an exception, the fallback
  cannot catch it and step in. So macOS runs a widget of our own over
  `vtkGenericOpenGLRenderWindow`, rendering *inside* the context Qt already owns
  — no second context, no native handle to get wrong. `CYCLOIDGEN_VTK_QTGL`
  selects that backend either way; `CYCLOIDGEN_VTK=0` drops any machine back to
  the software painter, and is the first thing to try if the 3D tab misbehaves.

  The quick mesh is verified against the volume of the solid that gets
  exported, part by part, so the picture and the STEP file are the same
  gearbox. The CAD mesh that replaces it is those solids, and is held to them
  the other way: every part closed, enclosing its solid's volume, and at every
  crank angle exactly where the quick mesh puts it. `CYCLOIDGEN_CAD_VIEW=0`
  keeps the view on the quick mesh, for a machine where a few seconds of a busy
  core after each edit is too many.

- **Outputs** — every file an export writes, by name, with what it is for and
  where it will land, *before* anything is written. Sizes fill in afterwards and
  a double-click opens any of them.
- **File ▸ Export animation** (`Ctrl+Shift+E`) writes whichever view you are
  looking at as a looping GIF — the drawing with the overlays you have ticked,
  or the assembly from the angle, explode and part visibility you have set. It
  renders off the GUI thread with a real progress bar and a Cancel that leaves
  no half-written file, and it uses the appearance you are in rather than the
  print surface the bundled `motion.gif` gets.
- **Loads**, **Efficiency**, and a **Datasheet** tab with everything above.
- **Trade study** — sweep any one parameter and watch torque capacity,
  efficiency, lost motion and mass move together, on their own real units, with
  the infeasible band shaded rather than silently dropped.

  ![trade study](https://raw.githubusercontent.com/medinstech/cycloidgen/main/docs/tradestudy.png)

  Read off that chart: pin radius buys torque capacity up to a genuine optimum
  and then gives it back — the `PIN_RADIUS_SUGGESTION` check in closed form —
  while costing mass the whole way and doing essentially nothing for efficiency
  or backlash. The one step in the mass curve is at 6 mm, where a catalogue
  needle roller first fits over the pin: the solid 12 mm pin becomes an 8 mm
  dowel in a sleeve, and a sleeve of rollers is lighter than the steel it
  stands in for. Note the flat panels start at zero. A quantity that moves by a
  tenth of a percent gets an axis that says so, rather than one scaled to make
  the noise look like a decision.
- **Compare** — pin a design, change things, and see exactly what moved and by
  how much. Running the optimiser pins the design it replaced automatically.
- **Appearance** follows the desktop and can be overridden per user
  (View ▸ Appearance). Both palettes are contrast-tested rather than asserted,
  and chrome and figures switch together — a chart on a white slab inside a dark
  window is the thing following the desktop theme was meant to prevent. The
  light mode is tinted paper, mixed from the brand's own blue, rather than
  white; the PDF still prints on white, because a tint on every figure is ink
  someone pays for and gains nothing on paper.

  Structure is drawn rather than implied by shadow, and the brand blue is spent
  only where it means *this one* — the primary action, the focused field, the
  selected row and tab, a ticked box. It used to be on every group heading, tab
  and rule as well, and at that point it stops being emphasis. Toggles in a
  toolbar — the overlays, the part groups, the severity filter — are chips
  rather than ticked boxes for the same reason: eight filled squares of the
  brand blue in a row stop reading as eight choices. There is one primary
  action, **Export**, in the corner of the header beside the readings that
  decide whether it is worth pressing.
- **Checks filter** — severity toggles carrying their own counts, because a
  design routinely produces a dozen findings of which ten are notes and the two
  that block an export sit somewhere in the middle.
- **Units** (View ▸ Units) — millimetres or decimal inches, for everything you
  *read*: the parameter fields, the datasheet, the checks list, the comparison
  table and the drawing's own title. Everything you *hand over* stays
  millimetres — DXF, STEP, STL, the JSON report and the PDF — because a CAD file
  whose units follow a preference is a CAD file nobody can trust. Internally the
  design is millimetres and never leaves them: a switch reloads the widgets from
  the spec rather than converting what is in them, so toggling the menu twenty
  times cannot move a number.
- **Explain this check** — select a finding and the panel beside it says what
  the check tests (the relation, in the notation above), what goes wrong
  physically when it fails, what to change and in which direction, and how many
  times clear of the limit you are. The parameters it names are highlighted in
  the panel at the same time. `cycloidgen/core/explain.py` declares one
  explanation per check code; a test parses the source for the calls that raise
  codes, so a check cannot be added without one or left behind when it goes.
- **Log** — everything the app would otherwise print to a terminal you do not
  have. Checks as they appear and clear, searches with their shortlists, sweeps,
  exports, plus every Python warning, stray stderr write, and any exception that
  escapes a worker thread. Timestamped, level-filtered, copyable. The tab marks
  itself when something arrives while you are looking elsewhere. It earned its
  keep immediately: the first run surfaced a `RuntimeWarning` from a negative
  log term in the contact-stiffness solver that had been silently producing NaNs
  on degenerate geometry.
- **Advanced** — the parameter panel opens on the seventeen fields a design is
  made with: ratio and pin geometry, disc, output pins, shaft, process,
  materials, motor frame, speed and torque. The other forty-four — clearances
  and tolerances, bearing designations, motor electrics, mounting bolts,
  lubrication — are behind the **Advanced** switch beside the search box (also
  View ▸ Advanced parameters, `Ctrl+Shift+A`). Hidden is not ignored: they keep
  their values, the analysis uses them, and the checks still test them. A search
  finds them with the switch off, and a check that points at one shows it.
- Undo/redo, recent files, a parameter search box, and parameter groups that
  fold behind their headings — a search or a selected check opens whichever
  group it needs. The session reopens on the design, tab, crank angle, open
  groups, the Advanced switch and the panel split you left. The split is stored as
  a *proportion*: remembering pixels hands a narrower screen most of its width
  to the parameter panel. The analysis runs off the GUI thread with generation
  numbering, so a slow result can never overwrite a newer one.

Preferences live wherever the platform puts them, or in a file of your choosing
via `CYCLOIDGEN_SETTINGS` — which is what makes a portable install possible, and
what lets the test suite run without rearranging your actual application.

**Updates.** **Help ▸ Check for updates…** asks GitHub which release is the
newest; **Help ▸ Check for updates automatically** does the same once a day and
stays quiet unless there is something to say. The first run asks whether it may,
and until that is answered no request is made by any path. What a check sends is
a request for a version number — nothing about your designs, your machine or
your files, none of which the application collects. This matters more here than
it does for most tools: the analysis models change between releases, so a design
sized against an old build can disagree with the current one about the same
drive, and nothing else in the application can tell you that.

What it offers depends on how the copy was installed. A pip install gets the
upgrade command with the running interpreter named in it. An AppImage or a disk
image is a file you placed yourself, so those go to the release page. The Windows
installer can be fetched with a progress bar, checked against its published
length, and started — the app closes, Windows asks permission, and preferences
and the restored session survive it. The build is unsigned, so SmartScreen warns
exactly as it does for an installer you downloaded yourself; the download is
checked for being whole, not for being ours. *Skip this version* silences one
release and no more; `CYCLOIDGEN_NO_UPDATE_CHECK` silences the feature entirely,
for a distribution whose users update through their package manager.

## How far to trust the analysis

The ideal load model treats the disc as rigid and each contact as a linear
spring, so force is proportional to moment arm and only the pushing half of the
pins carries load — the classical Kudryavtsev/Lehmann assumption. The stiffness
model *does* see clearance and is what the capacity derating comes from, and it
counts the parts around the mesh rather than calling them rigid — but it counts
them as ideal parts. Joints, fits and fasteners are not in it, so a real drive
measures softer again, and two of the six terms rest on an assumption the
geometry does not settle (see `analysis/compliance.py`, which states both).
Pin position error is modelled only if you enter a tolerance; with none entered
the ring is perfect, which no ring is. Transmission error is the
drive's own share only — clearance take-up and deflection, both of which it
solves; pin position error, profile error and runout are the manufacturing half
and are not modelled, so a real drive measures worse. The thermal model is a single
lumped body in still air with no conduction into whatever the gearbox is bolted
to. Bearing catalogue values are nominal metric-series figures, and a bearing's
mass is estimated from its envelope rather than read from a maker's table. Of
the bolts the output's are not weighed: they have no length until the drive
knows what they go into. The inertia at the input
leaves out the pin rollers' spin about their own pins and the rings of the
shaft's bearings, a few grams each at a few millimetres, and the load, which
the drive does not know. PV limits are
dry-against-steel design-guide values. The lubrication model builds a
Dowson-Hamrock film for a *line* contact and applies it to the two conforming
ones as well, where the formula was not derived — their film is capped at the
radial clearance, which is a real bound rather than a fitted one, but a
close-fitting journal is properly a hydrodynamic problem and this is not that.
Starvation, grease channelling and the transient film at start-up are not
modelled, and all three make it worse, so a contact it places in the boundary
regime is there.

**These are preliminary sizing numbers, not a certification.** Calibrate against
a physical prototype before committing to a design.

## Layout

```
cycloidgen/
├── units.py    what lengths are *shown* in; everything inside is millimetres
├── update.py   whether a newer release exists and how this copy would get it;
│               no Qt, so the comparison is testable without a display
├── notice.py   what this tool does not claim, in one place
├── core/       spec (the one source of truth), profile, kinematics, validate,
│               motor (the face it bolts to and the torque it makes),
│               duty (what the drive is asked to do, over time), fasteners
│               (the ISO bolts and nuts it is held together with), fits (ISO
│               286 limits, and the fit a bearing seat takes), presets (the
│               ratios, and whole drives built round a motor), designfile
│               (a saved design, and which build saved it), explain (what each
│               check tests, why, and what to change), guide (what each
│               parameter is and how to choose it)
├── analysis/   mechanics (Hertz), stiffness (contacts, backlash, transmission
│               error), compliance (the parts around the mesh, as springs),
│               tolerance (where the pins actually are), lubrication (the
│               film, and the friction it earns), fatigue, thermal, mass,
│               efficiency, bearings, motor (what is asked of it against what
│               it has), duty (the design against a whole cycle)
├── design/     optimise (requirements -> geometry, and the reduction from a
│               motor), sweep (trade studies), batch (`--vary`: many designs
│               in, one table out)
├── viz/        3D geometry and rendering maths, no Qt: mesh, cadmesh (the
│               exported solids, tessellated for the view), appearance
│               (each material's finish, and the studio), scene (the
│               software projection), vtkbridge (mesh -> VTK polydata),
│               clashes (which parts a check is about), tessellate
├── export/     manifest (what a bundle contains), dxf, svg, solid (OCCT),
│               threemf (the coloured assembly), printing (the face a part
│               prints on, and its bed edge), sheets (the dimensioned drawings),
│               bom, animation (the looping GIF)
├── report/     plots (shared by UI and PDF), build, drafting (views,
│               dimensions and a title block on an A3 sheet)
└── ui/         PySide6 window, 3D viewer, outputs tab, declarative field table,
                folding parameter groups, optimiser and duty-cycle dialogs,
                trade-study tab, undo/redo history, log panel, branding
                (palette and stylesheet), plotbar (the trimmed matplotlib
                toolbar), updates (the check, and asking before it runs)
docs/           the API reference and the README's figures
tools/          what draws the banner, the icon, the logo assets, the
                installer's bitmaps and the screenshots
tests/          1,521 tests; the envelope, pin-in-hole, clearance-sign,
                mesh-versus-solid and animation-closes tests matter most
```

Two boundaries in there are load-bearing. **`core` and `viz` do not import Qt**,
which is what lets the geometry and the renderer be tested without a display and
what keeps one piece of code drawing both the window and the PDF. And
**`export/manifest.py` declares every output file exactly once** — the writer,
the Outputs tab, `--list-outputs` and the table above all read it.

## Tests

```bash
.venv\Scripts\python -m pytest -q
```

1,521 tests, about 700 s. Most of that is CadQuery writing solids; the pure
analysis tests run in a few seconds. The Qt tests run headless
(`QT_QPA_PLATFORM=offscreen`, set by the test modules themselves) and redirect
preferences into a temporary file, so the suite cannot rearrange your own
application.

Three of them are worth calling out because they are the ones that keep
separate representations of the same gearbox honest:

- **The 3D mesh against the exported solid.** Every part's mesh volume is
  computed by the divergence theorem and compared against what OCCT says the
  STEP body encloses, to within faceting error - a bearing against the room it
  takes, since the quick mesh draws it as that and the STEP file as the
  bearing, which a test of its own holds to the steel it is weighed as. The
  same test checks that each
  part is a *closed* surface — the vector areas of its faces must cancel
  exactly, which catches a missing end cap, a wall built for the wrong number of
  edges, or a loop wound the wrong way round. It has already earned its keep: it
  found the eccentric shaft double-counting the barrel inside its own cams.
- **The mass model against the exported solid.** Each made part's mass, at its
  density, against the volume of its STEP body, part by part; and the shaft's
  and the turning output member's inertia against the second moment OCCT
  integrates over the same bodies. Its first run found the carrier's boss
  weighed flush with an end plate it stands 8 mm proud of.
- **The manifest against the disk.** An export is compared against what the
  manifest promised — exactly those files and no others — including the table in
  this README.

CI runs `ruff`, then the suite on Linux, Windows and macOS across the oldest and
newest supported Python, then separately exports a full bundle and runs a design search
from the command line — the tests cover the pieces, that job proves the whole
thing still runs end to end.

Lint rules live in `pyproject.toml` so they mean the same thing locally and in
CI. `ruff format` is deliberately not part of it: the repository has a
hand-aligned style the formatter would rewrite wholesale, and a lint that hunts
bug shapes is worth more than one that enforces a preference.

The README figures come out of the same plotting code the app and the PDF use,
so they cannot drift from what it actually draws:

```bash
.venv\Scripts\python docs\make_figures.py
```

The two screenshots of the window are taken by a second script, and it needs a
real desktop session rather than the offscreen platform the tests use — twice
over. Offscreen Qt has no fonts, so every label renders as tofu; and the 3D
tab's viewport is a native OpenGL surface that `QWidget.grab` composites *around*,
leaving a black hole where the gearbox was. So the window is opened, driven —
hero design, overlays on, a check selected, the camera set, X-ray on and a
disc picked — and photographed through `PrintWindow` with
`PW_RENDERFULLCONTENT`, which asks Windows for the window's own rendering
including native children and does not care what is stacked on top of it:

```powershell
.venv\Scripts\python tools\make_screenshots.py
```

The banner at the top is drawn by a third. Its disc is fitted by the routine
that fits the STEP export's spline and converted exactly into SVG curves, and
its motion is the pose law the drawing turns on, written as three steady
rotations — each loops on its own, so the banner plays as one unending output
revolution with no seam, in CSS and without a script, which is all GitHub and
PyPI will run:

```bash
.venv\Scripts\python tools\make_banner.py
```

## Performance

A full re-analysis is about 110 ms — every check, every load, the stiffness
model, transmission error and the whole datasheet, on each parameter edit. It
was 106 ms when it did a fraction of that work, and three things bought the room
the rest of it went into: the mesh sweep is computed once and shared instead of
three modules each running their own, the profile, the undercut limit and the
measured mesh clearances are cached, and the self-intersection test uses a
uniform-grid broad phase instead of sampling every other segment against all the
rest — which was also skipping over crossings it should have caught.

Entering a pin position tolerance is the one thing that costs real time, about
half a second, because it stops analysing *a* gearbox and starts analysing a
batch of them. It is off unless you ask for it.

Animation runs on a 33 ms tick and both views fit inside it. The drawing costs
about 10 ms a frame, down from 25: the artists are built when the *design*
changes and repositioned when the *angle* does, so a crank move no longer
rebuilds a couple of hundred patches and re-runs `tight_layout`. The 3D view
sets seven transforms and hands them to the card, so it is bounded by vsync
rather than by us; the software fallback does the whole projection in numpy in
about 1 ms and spends the rest painting, for roughly 13 ms a frame.

Neither view is redrawn while its tab is hidden — a hidden matplotlib canvas
will still honour `draw_idle` with a full render, which is ten milliseconds a
frame spent on something nobody can see — and neither rebuilds geometry for a
change that did not touch it. The mesh cache is keyed on the fields the geometry
actually depends on, so changing a material or the rated torque does not send a
fresh copy of the assembly to the graphics card. That key is a hand-written list
and therefore a liability, so a test perturbs *every* field of `GearSpec` in turn
and requires that an unchanged key really does mean an unchanged mesh.

The live animation loops over the mechanism's own period, which is `lobes`
input revolutions — one output revolution — and not one turn of the crank. After
a single input turn the disc and the carrier have moved on by 360/lobes and are
not back where they started, which is what used to put a visible jump in the
loop every four seconds. The *exported* animation cannot afford that many turns
at a legible frame rate and picks its run differently — see **Why the animation
loops without a jump** above.

## Standalone build and installer

Four artefacts come out of a release, and they are not alternatives:

| | who it is for | platforms |
|---|---|---|
| **Installer** (`.exe`) | somebody who wants to run the app and does not have Python | Windows |
| **Disk image** (`.dmg`) | the same person, on a Mac | macOS 11+, Apple silicon |
| **AppImage** | the same person, on Linux | x86-64, glibc 2.35 and up |
| **Wheel** on PyPI | anybody with Python 3.10 – 3.14 — the only route on an Intel Mac, and the only way to `import cycloidgen` | Windows, Linux, macOS, x86-64 and arm64 |

There is no cross-platform installer format, so the first three are separate
programs doing one job three ways. NSIS writes an installer that unpacks the
bundle into Program Files and registers it. An AppImage installs nothing — it
*is* the bundle, in a squashfs image behind a small runtime that mounts it. A
`.dmg` splits the difference: a disk image that opens to the application beside
a shortcut to `/Applications`, and installing is dragging one onto the other.

Apple silicon only on the Mac, because every x86-64 macOS runner GitHub offers
is now a paid *larger runner* and the free Intel image was retired. An Intel Mac
gets `pip install cycloidgen`, which is the same application.

```powershell
.\packaging\release.ps1                        # lint, tests, bundle, installer
.\packaging\release.ps1 -FastPack -SkipTests    # quick internal build
```

```bash
python -m PyInstaller cycloidgen.spec --noconfirm   # Linux and macOS
bash packaging/appimage.sh                          # -> releases/*.AppImage
bash packaging/macos.sh                             # -> releases/*.dmg
```

Or the two steps by hand:

```bash
.venv\Scripts\python -m PyInstaller cycloidgen.spec --noconfirm
dist\cycloidgen\cycloidgen.exe                         # the window
dist\cycloidgen\cycloidgen-cli.exe --list-outputs      # the command line
makensis /INPUTCHARSET UTF8 packaging\cycloidgen.nsi   # needs NSIS 3.x
```

The Windows bundle carries **two executables over one analysis**, which is the
`pythonw.exe` / `python.exe` arrangement and for the same reason.
`cycloidgen.exe` is windowed — it is what the shortcuts point at, and a console
build there put a black window behind the application every time somebody opened
it from the Start menu. `cycloidgen-cli.exe` is the console one, for headless
runs. A single windowed build would have taken the command line away and taken
it away badly: a frozen windowed process has no stdout at all, so `--version`
would not print nothing, it would raise.

The AppImage and the `.app` carry **one**. `console` is a Windows subsystem flag
— a field in the PE header saying whether the loader allocates a console — and
Linux has no equivalent; every process gets whatever streams it was handed. A
second binary there would not be a second behaviour, only a second copy of the
same one under a name implying a difference that does not exist. On macOS the
flag means a third thing — whether Launch Services sees a windowed application
or a terminal program — and it costs nothing, because a process started from a
shell has stdio whatever its bundle says. That is why
`cycloidgen.app/Contents/MacOS/cycloidgen --version` still prints, and why the
release workflow can ask the bundle it just built what it is.

That last sentence holds for the bundle the workflow builds and not for the one
a user downloads. Gatekeeper blocks the `exec` as well as the double-click, so
on a quarantined install the same command hangs for about ten seconds and
prints nothing — it is not a broken build, and it answers normally once the
quarantine flag is cleared.

The installer upgrades in place — it clears the previous version first, because
unpacking a PyInstaller bundle *over* another one leaves modules and DLLs from
the old version that load in preference to the new ones and then fail in ways
that look like an application bug. It waits if the application is running,
registers properly with Add/Remove Programs, and leaves your preferences and
last design alone when uninstalled. English and Turkish.

Versions come from one line in `cycloidgen/__init__.py`: the wheel, the About
box, the executable's file properties, the installer's filename and the release
workflow all read that one, and `tests/test_version.py` fails if a second copy
appears or if the changelog has no section for it. See
[RELEASING.md](https://github.com/medinstech/cycloidgen/blob/main/RELEASING.md).

Verified working, GUI and CLI, including STEP/STL export. Two things about the
PyInstaller spec are load-bearing and easy to break:

- The entry point is `launcher.py`, not `cycloidgen/__main__.py`. PyInstaller
  runs its entry script as a top-level module, which breaks relative imports.
- The DLLs that `OCP`'s extension links against stay in the package directory
  when PyInstaller relocates the `.pyd` to the bundle root, so the spec copies
  them to the root as well. Without it the exe dies with `DLL load failed`.
- `casadi` is not bundled, and `packaging/rthook_casadi.py` stands in for it.
  CadQuery imports its constraint solver on the way in whatever you use it for,
  and this application places every part by an explicit transform — so the
  import is satisfied and the optimiser is left out. Reading a name off the
  stand-in works, which is all the two annotations that run at import time need;
  calling one raises and says why.

The bundle is about 790 MB. It was 1.2 GB until the dependencies CadQuery
declares but this application never imports were measured rather than assumed —
`numba` with LLVM behind it, `trame`, and `casadi` — which came to 444 MB, or
36% of it. That the CAD kernel was the weight was the obvious guess and it was
wrong: OCCT is 152 MB, and the optimiser that arrived beside it was larger.

About 123 MB of VTK is also never loaded, which is measurable but not yet
removed: unlike the above it cannot be checked from the test suite, because
excluding a VTK module is a missing DLL in the frozen build rather than a failed
import in a virtual environment.

## Where it is going

One thing is worth more than everything else on the list: **calibration against
real hardware**. Every number here is a first-principles estimate with a stated
model and stated limits. That is honest, and it is also the ceiling — the moment
there is a table of predicted versus measured, this stops being a calculator and
becomes a calibrated instrument.

The half of that which was not waiting on a lathe is in: fitting the model's
free constants against measurements is a script over a grid, and the grid, the
importable analysis and the [documented API](https://github.com/medinstech/cycloidgen/blob/main/docs/api.md) are all here now.

After that, in rough order: bearing life under combined load
and misalignment; and more kinds of drive — compound and multi-stage, RV-type,
a pinwheel output. On the output side, dimensioned drawing sheets with
tolerances and a title block, which is what a shop actually wants, and STEP
AP242 with PMI. [Open an issue](https://github.com/medinstech/cycloidgen/issues) if one of
those is what stands between you and using it — that moves it up the list.

## Contributing

[CONTRIBUTING.md](https://github.com/medinstech/cycloidgen/blob/main/CONTRIBUTING.md) covers setup, the house style, the settled
questions, and what a change looks like here. The short version: numbers are
verified rather than asserted, comments say *why*, and `ruff format` is
deliberately not part of the build.

The most valuable contribution needs no code at all — **build one of these and
measure it**.

Also: [CHANGELOG.md](https://github.com/medinstech/cycloidgen/blob/main/CHANGELOG.md) · [RELEASING.md](https://github.com/medinstech/cycloidgen/blob/main/RELEASING.md) ·
[SECURITY.md](https://github.com/medinstech/cycloidgen/blob/main/SECURITY.md) · [CODE_OF_CONDUCT.md](https://github.com/medinstech/cycloidgen/blob/main/CODE_OF_CONDUCT.md)

## License

Apache-2.0. Copyright 2026 Medinstech. See [LICENSE](https://github.com/medinstech/cycloidgen/blob/main/LICENSE).

The Medinstech name and logos under `cycloidgen/ui/assets/`, and the wordmark
the banner at the top is drawn with, are trademarks and are **not** covered by
that licence — see [NOTICE](https://github.com/medinstech/cycloidgen/blob/main/NOTICE). They are declared in a
single module (`cycloidgen/ui/branding.py`) so that replacing them in a fork is
one obvious change. The application icon is not one of them: it is a cycloidal
disc drawn from the profile equations by `tools/make_icon.py`, it is Apache-2.0
like the rest of the source, and a fork may keep it.

The numbers this produces are preliminary sizing estimates, not a certification —
see *How far to trust the analysis* above. Apache-2.0 disclaims warranty for a
reason, and that disclaimer is meant literally here: validate against a physical
prototype before anything load-bearing depends on it.

<p align="center">
  <a href="https://medinstech.com">
    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/medinstech/cycloidgen/main/cycloidgen/ui/assets/wordmark-white.png">
      <img alt="Medinstech" src="https://raw.githubusercontent.com/medinstech/cycloidgen/main/cycloidgen/ui/assets/wordmark-blue.png" height="64">
    </picture>
  </a>
</p>
