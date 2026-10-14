"""Draw the README banner: the drive turning, out of the equations it is cut from.

    python tools/make_banner.py

Writes ``docs/banner.svg``.  Nothing in it is drawn by hand.  The disc is a
closed cubic B-spline from :func:`cycloidgen.core.profile.profile_spline`, the
routine the STEP export fits its disc with, held to a tenth of a pixel rather
than to the design's machining tolerance and converted span for span into the
Bezier curves SVG speaks - the conversion is exact, so the banner is the
profile to within that tenth of a pixel.  The pins, holes, bore and housing are the preset's own dimensions, and the
motion is the pose law in :func:`cycloidgen.core.kinematics.disc_pose`.

The motion is plain CSS, so it plays wherever the README is read - GitHub and
PyPI both load the file as an image, which runs no script.  That rules out
computing a frame at a time, and a keyframe list long enough to close the
mechanism's period would be a megabyte.  Neither is needed, because the pose law
is a sum of steady rotations:

* the disc centre is ``rotate(-phi)`` applied to ``(E, 0)``: a group turning
  once per crank turn, holding a group offset by the eccentricity;
* inside that, a group turning ``+phi`` cancels the orientation the first one
  dragged along - the disc orbits, it does not spin with the crank;
* inside that, a group turning ``+phi / N`` - the reduction itself.

Each of those is periodic on its own, so each loops on its own without a jump
however long the whole mechanism takes to come round, and the stack plays as one
unending output revolution rather than as a clip.  A second disc is the same
stack started half a turn of crank on, with its holes turned back by its
``disc_hole_phases`` entry - which is why the two discs are different parts.

The text is live text in the reader's own system font rather than outlines of
one: a banner on GitHub is read in Segoe UI on Windows and in San Francisco on
a Mac, and it should look like it belongs in either.  Everything that has to
line up with it is therefore centred on it rather than measured against it.

The company's wordmark, name and colour come from :mod:`cycloidgen.ui.branding`
and the assets it loads, like everywhere else, so a fork that rebrands there
gets its own banner on the next run rather than ours.
"""
from __future__ import annotations

import base64
import io
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:                    # runnable without installing
    sys.path.insert(0, str(ROOT))

from cycloidgen.analysis.bearings import seat_bores  # noqa: E402
from cycloidgen.core.explain import EXPLANATIONS  # noqa: E402
from cycloidgen.core.kinematics import disc_pose  # noqa: E402
from cycloidgen.core.profile import disc_profile, profile_spline  # noqa: E402
from cycloidgen.core.spec import GearSpec, preset  # noqa: E402
from cycloidgen.report import plots  # noqa: E402
from cycloidgen.ui.branding import BRAND_BLUE, COMPANY, asset, lighten, mix  # noqa: E402

OUT = ROOT / "docs" / "banner.svg"

W, H = 1280, 400
#: Where the drive sits and how big its housing is drawn, in pixels.
CX, CY, HOUSING_PX = 1012.0, 192.0, 156.0
#: Seconds per crank turn.  The output then takes ``lobes`` times that, which on
#: the 15:1 is half a minute - slow enough to be the reduction, fast enough that
#: somebody who watches for a few seconds sees the carrier pins move.
CRANK_S = 2.4

SANS = ("'Segoe UI Variable Display','Segoe UI',system-ui,-apple-system,"
        "BlinkMacSystemFont,'Helvetica Neue',Helvetica,Arial,sans-serif")
MONO = ("'Cascadia Mono',Consolas,'SF Mono',Menlo,'DejaVu Sans Mono',"
        "'Liberation Mono',monospace")


#: The company wordmark, as tall as it is drawn.  The white one: the banner is
#: a dark card on either GitHub theme.
LOGO_PX = 58.0
#: Where its top edge sits.
LOGO_TOP = 64.0


def logo_uri(height: float) -> tuple[str, float]:
    """The wordmark as a data URI, at twice the height it is drawn, and its width.

    Inlined because a README banner is loaded as an image, and an image may not
    fetch anything else - a linked logo would be a broken one.  Twice the height
    so it stays sharp on a high-density screen, and no more, because the whole
    of it is carried in the file on every view of the page.
    """
    image = Image.open(asset("wordmark-white.png")).convert("RGBA")
    width = height * image.width / image.height
    image = image.resize((round(2 * width), round(2 * height)), Image.LANCZOS)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG", optimize=True)
    return ("data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode(),
            width)


def drive() -> GearSpec:
    """The 15:1 preset as it ships.

    Not the README's machined 21:1: at banner size twenty-one lobes and 1.4 mm of
    eccentricity are a fine ripple, and fifteen lobes on 1.6 mm is the smallest
    count whose lobes still read as lobes at a glance.  A preset rather than a
    tuned one, so the banner is a drive the app passes rather than a drawing.
    """
    return preset(15)


def f(v: float) -> str:
    """A coordinate, short.  Millimetres at this scale want no more than this."""
    text = f"{v:.3f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def circle_path(x: float, y: float, r: float) -> str:
    """A circle as path data, so it can be a hole in another shape's fill."""
    return (f"M{f(x + r)} {f(y)}A{f(r)} {f(r)} 0 1 0 {f(x - r)} {f(y)}"
            f"A{f(r)} {f(r)} 0 1 0 {f(x + r)} {f(y)}Z")


def spline_path(control: np.ndarray) -> str:
    """A closed uniform cubic B-spline as Bezier segments - exactly, not fitted.

    Span ``i`` is governed by control points ``i-1 .. i+2`` and starts at
    ``(P[i-1] + 4 P[i] + P[i+1]) / 6``, which is the basis
    :func:`~cycloidgen.core.profile.evaluate_closed_bspline` evaluates.
    """
    p = np.asarray(control)
    prev, nxt, nxt2 = np.roll(p, 1, axis=0), np.roll(p, -1, axis=0), np.roll(p, -2, axis=0)
    start = (prev + 4.0 * p + nxt) / 6.0
    c1 = (2.0 * p + nxt) / 3.0
    c2 = (p + 2.0 * nxt) / 3.0
    end = (p + 4.0 * nxt + nxt2) / 6.0
    parts = [f"M{f(start[0, 0])} {f(start[0, 1])}"]
    parts += [f"C{f(a[0])} {f(a[1])} {f(b[0])} {f(b[1])} {f(c[0])} {f(c[1])}"
              for a, b, c in zip(c1, c2, end, strict=True)]
    return "".join(parts) + "Z"


def disc_path(spec: GearSpec, outline: str, hole_phase: float) -> str:
    """One disc in its own frame: outline, bore and output holes, one fill."""
    bore = seat_bores(spec).disc / 2.0
    hole = spec.output_hole_diameter / 2.0
    parts = [outline, circle_path(0.0, 0.0, bore)]
    for k in range(spec.output_pin_count):
        a = 2.0 * math.pi * k / spec.output_pin_count + hole_phase
        parts.append(circle_path(spec.output_bolt_circle_radius * math.cos(a),
                                 spec.output_bolt_circle_radius * math.sin(a), hole))
    return "".join(parts)


def rim_point(spec: GearSpec) -> np.ndarray:
    """A lobe crest in the disc's own frame - a point that is actually on the rim.

    Half a lobe pitch along the profile parameter: ``cos(N t) = -1`` there,
    which is the crest.  At ``t = 0`` the profile is at a *root*, so a marker
    placed at the crest radius on the disc's x axis would ride three millimetres
    clear of the material.
    """
    t = np.array([math.pi / spec.lobes])
    return disc_profile(t, spec.effective_R, spec.effective_Rr,
                        spec.eccentricity, spec.lobes)[0]


def trace_path(spec: GearSpec, point: np.ndarray) -> str:
    """Where ``point`` goes over one output turn: ``lobes`` turns of the crank."""
    phis = np.linspace(0.0, 2.0 * math.pi * spec.lobes, 48 * spec.lobes,
                       endpoint=False)
    centre, delta = disc_pose(phis, spec.eccentricity, spec.lobes)
    c, s = np.cos(delta), np.sin(delta)
    x = c * point[0] - s * point[1] + centre[:, 0]
    y = s * point[0] + c * point[1] + centre[:, 1]
    return ("M" + "L".join(f"{f(a)} {f(b)}" for a, b in zip(x, y, strict=True))
            + "Z")


def chips(labels: list[str], x: float, y: float) -> str:
    """Pills laid out left to right, sized for a monospace face.

    The width is an estimate - the reader's monospace font decides - so each
    label is centred in its pill and the pill is padded for the widest of the
    faces in ``MONO``.  A narrower face leaves a little more air, never less.
    """
    out, cursor = [], x
    for label in labels:
        width = len(label) * 8.9 + 34.0
        out.append(
            f'<rect x="{f(cursor)}" y="{f(y)}" width="{f(width)}" height="32" '
            f'rx="16" class="chip"/>'
            f'<text x="{f(cursor + width / 2)}" y="{f(y + 21)}" class="chiptext">'
            f"{label}</text>")
        cursor += width + 10.0
    return "".join(out)


def build(spec: GearSpec) -> str:
    with plots.using_theme("dark"):
        blue, orange, green = plots.theme()["series"]

    scale = HOUSING_PX / spec.housing_outer_radius
    lobes, E = spec.lobes, spec.eccentricity
    crank, output = CRANK_S, CRANK_S * lobes
    deg = math.degrees

    # Tight enough that the curve is the curve at this size: a tenth of a pixel.
    spline = profile_spline(spec.effective_R, spec.effective_Rr, E, lobes,
                            tol=0.1 / scale)
    outline = spline_path(spline.control_points)
    crest = rim_point(spec)
    bore = seat_bores(spec).disc / 2.0

    def disc_stack(i: int, body: str) -> str:
        """Disc ``i`` on its own crank phase - see the module docstring."""
        p = spec.disc_phases[i]
        return (f'<g transform="rotate({f(-deg(p))})"><g class="crank">'
                f'<g transform="translate({f(E)} 0) rotate({f(deg(p))})">'
                f'<g class="uncrank"><g transform="rotate({f(deg(p) / lobes)})">'
                f'<g class="output">{body}</g></g></g></g></g></g>')

    back = "".join(
        disc_stack(i, f'<path d="{disc_path(spec, outline, spec.disc_hole_phases[i])}" '
                      f'class="disc-back"/>')
        for i in range(1, spec.disc_count))
    front = disc_stack(0, (
        f'<path d="{disc_path(spec, outline, spec.disc_hole_phases[0])}" '
        f'class="disc" fill-rule="evenodd"/>'
        f'<circle cx="{f(crest[0])}" cy="{f(crest[1])}" r="1.1" class="marker"/>'))

    ring_pins = "".join(
        f'<circle cx="{f(spec.pin_circle_radius * math.cos(a))}" '
        f'cy="{f(spec.pin_circle_radius * math.sin(a))}" r="{f(spec.pin_radius)}"/>'
        for a in 2.0 * np.pi * np.arange(spec.pin_count) / spec.pin_count)
    output_pins = "".join(
        f'<circle cx="{f(spec.output_bolt_circle_radius * math.cos(a))}" '
        f'cy="{f(spec.output_bolt_circle_radius * math.sin(a))}" '
        f'r="{f(spec.output_pin_diameter / 2.0)}"/>'
        for a in 2.0 * np.pi * np.arange(spec.output_pin_count) / spec.output_pin_count)

    shaft_r = spec.input_shaft_diameter / 2.0
    cam_r = bore - 1.0
    housing_r = spec.housing_outer_radius
    od = f"{2.0 * housing_r:g}"
    dim_x = CX - HOUSING_PX - 30.0
    top, bottom = CY - HOUSING_PX, CY + HOUSING_PX

    ink_soft = mix("#ffffff", BRAND_BLUE, 0.22)
    logo, logo_w = logo_uri(LOGO_PX)
    rule_x = 72.0 + logo_w + 20.0
    accent = lighten(BRAND_BLUE, 0.42)
    disc_hi = lighten(blue, 0.30)
    # Shaded about the disc's own centre.  A linear gradient turns with the disc,
    # and a highlight that walks round the part reads as the light moving.
    disc_mid = lighten(blue, 0.16)

    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" role="img" aria-labelledby="t d">
<title id="t">cycloidgen - parametric cycloidal drive design</title>
<desc id="d">A {spec.ratio}:1 cycloidal drive turning: the disc orbits on its eccentric, rolls round {spec.pin_count} ring pins and turns the {spec.output_pin_count} output pins one revolution for every {lobes} of the input. Drawn by tools/make_banner.py from the same profile the application exports.</desc>
<defs>
<linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
<stop offset="0" stop-color="#05041a"/><stop offset=".55" stop-color="#0a0736"/><stop offset="1" stop-color="#110a66"/>
</linearGradient>
<radialGradient id="glow" cx="{f(CX)}" cy="{f(CY)}" r="{f(HOUSING_PX * 1.75)}" gradientUnits="userSpaceOnUse">
<stop offset="0" stop-color="{BRAND_BLUE}" stop-opacity=".62"/><stop offset=".55" stop-color="{BRAND_BLUE}" stop-opacity=".16"/><stop offset="1" stop-color="{BRAND_BLUE}" stop-opacity="0"/>
</radialGradient>
<radialGradient id="fade" cx="{f(CX)}" cy="{f(CY)}" r="{f(HOUSING_PX * 2.6)}" gradientUnits="userSpaceOnUse">
<stop offset="0" stop-color="#fff"/><stop offset="1" stop-color="#000"/>
</radialGradient>
<mask id="gridmask"><rect width="{W}" height="{H}" fill="url(#fade)"/></mask>
<pattern id="grid" width="24" height="24" patternUnits="userSpaceOnUse" x="{f(CX % 24)}" y="{f(CY % 24)}">
<path d="M24 0H0V24" fill="none" stroke="#fff" stroke-opacity=".07" stroke-width="1"/>
</pattern>
<radialGradient id="discfill" cx="0" cy="0" r="{f(spec.housing_outer_radius * 0.8)}" gradientUnits="userSpaceOnUse">
<stop offset="0" stop-color="{disc_mid}" stop-opacity=".96"/><stop offset="1" stop-color="{blue}" stop-opacity=".9"/>
</radialGradient>
<clipPath id="card"><rect width="{W}" height="{H}" rx="20"/></clipPath>
</defs>
<style>
.crank{{animation:cw {crank:g}s linear infinite}}
.uncrank{{animation:ccw {crank:g}s linear infinite}}
.output{{animation:ccw {output:g}s linear infinite}}
@keyframes cw{{to{{transform:rotate(-360deg)}}}}
@keyframes ccw{{to{{transform:rotate(360deg)}}}}
@media (prefers-reduced-motion:reduce){{.crank,.uncrank,.output{{animation:none}}}}
.disc{{fill:url(#discfill);stroke:{disc_hi};stroke-width:.32}}
.disc-back{{fill:none;stroke:{disc_hi};stroke-opacity:.5;stroke-width:.4}}
.marker{{fill:#fff}}
.sans{{font-family:{SANS}}}
.mono{{font-family:{MONO}}}
.chip{{fill:#fff;fill-opacity:.06;stroke:#fff;stroke-opacity:.16}}
.chiptext{{font-family:{MONO};font-size:14px;fill:{ink_soft};text-anchor:middle}}
</style>
<g clip-path="url(#card)">
<rect width="{W}" height="{H}" fill="url(#bg)"/>
<rect width="{W}" height="{H}" fill="url(#grid)" mask="url(#gridmask)"/>
<rect width="{W}" height="{H}" fill="url(#glow)"/>

<g fill="none" stroke="#fff" stroke-opacity=".22" stroke-width="1" stroke-dasharray="22 5 3 5">
<path d="M{f(CX - HOUSING_PX - 18)} {f(CY)}H{f(CX + HOUSING_PX + 18)}M{f(CX)} {f(top - 18)}V{f(bottom + 18)}"/>
</g>
<g stroke="#fff" stroke-opacity=".3" stroke-width="1" fill="none">
<path d="M{f(CX - 10)} {f(top)}H{f(dim_x - 7)}M{f(CX - 10)} {f(bottom)}H{f(dim_x - 7)}M{f(dim_x)} {f(top + 9)}V{f(bottom - 9)}"/>
</g>
<g fill="#fff" fill-opacity=".45">
<path d="M{f(dim_x)} {f(top)}l-3.5 10h7z"/><path d="M{f(dim_x)} {f(bottom)}l-3.5 -10h7z"/>
</g>
<text transform="translate({f(dim_x - 9)} {f(CY)}) rotate(-90)" class="mono" font-size="13" fill="#fff" fill-opacity=".55" text-anchor="middle">&#216;{od}</text>

<g transform="translate({f(CX)} {f(CY)}) scale({f(scale)} {f(-scale)})">
<circle r="{f(housing_r)}" fill="#fff" fill-opacity=".035" stroke="#fff" stroke-opacity=".38" stroke-width=".42"/>
<circle r="{f(spec.pin_circle_radius)}" fill="none" stroke="#fff" stroke-opacity=".13" stroke-width=".3" stroke-dasharray="1.6 1.2"/>
{back}
{front}
<path d="{trace_path(spec, crest)}" fill="none" stroke="#fff" stroke-opacity=".28" stroke-width=".22"/>
<g fill="{orange}">{ring_pins}</g>
<g class="output"><g fill="{green}">{output_pins}</g></g>
<g class="crank">
<circle cx="{f(E)}" r="{f(cam_r)}" fill="#0b0933" stroke="{disc_hi}" stroke-opacity=".7" stroke-width=".35"/>
<circle r="{f(shaft_r)}" fill="#15124a" stroke="#fff" stroke-opacity=".55" stroke-width=".35"/>
<path d="M0 0H{f(cam_r * 0.92)}" stroke="#fff" stroke-opacity=".75" stroke-width=".45" stroke-dasharray="1.6 .9"/>
<circle cx="{f(E)}" r=".9" fill="#fff"/>
</g>
</g>
<text x="{f(CX)}" y="{f(bottom + 30)}" class="mono" font-size="13" fill="#fff" fill-opacity=".5" text-anchor="middle">{spec.ratio}:1  &#183;  {lobes} lobes / {spec.pin_count} pins  &#183;  ring fixed</text>

<image href="{logo}" x="72" y="{f(LOGO_TOP)}" width="{f(logo_w)}" height="{f(LOGO_PX)}"><title>{COMPANY}</title></image>
<path d="M{f(rule_x)} {f(LOGO_TOP + 10)}V{f(LOGO_TOP + LOGO_PX - 10)}" stroke="#fff" stroke-opacity=".28"/>
<text x="{f(rule_x + 20)}" y="{f(LOGO_TOP + LOGO_PX / 2 + 4.5)}" class="sans" font-size="13" font-weight="600" letter-spacing="3.6" fill="{accent}">OPEN SOURCE</text>
<text x="68" y="212" class="sans" font-size="86" font-weight="700" letter-spacing="-2" fill="#fff">cycloidgen</text>
<text x="72" y="256" class="sans" font-size="26" fill="{ink_soft}">Parametric cycloidal drive design</text>
{chips(["2D + 3D, live", f"{len(EXPLANATIONS)} checks", "DXF · STEP · STL · 3MF", "PDF dossier"], 72.0, 292.0)}
</g>
<rect x=".5" y=".5" width="{W - 1}" height="{H - 1}" rx="19.5" fill="none" stroke="#fff" stroke-opacity=".08"/>
</svg>
"""


def main() -> int:
    spec = drive()
    svg = build(spec)
    OUT.write_text(svg, encoding="utf-8", newline="\n")
    print(f"  {OUT.name}  {spec.ratio}:1  {len(svg.encode()) / 1024:.0f} kB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
