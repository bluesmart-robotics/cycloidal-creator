"""The cycloidal disc profile.

The closed forms here were verified numerically before being written down:
the generated profile sits at distance exactly ``Rr`` from the pin-centre locus
at every point (envelope deviation 0.0000 um), which is the defining property of
the conjugate profile.  ``tests/test_profile.py`` keeps that property as a
permanent regression test.

Sign warning
------------
The equivalent ``psi`` formulation needs a *leading minus*::

    psi(t) = -atan2(sin(N*t), R/(E*Np) - cos(N*t))

The positive-sign variant is widespread online and is wrong: it deviates by
millimetres and the disc interferes with the pins.  We use the atan2-free
algebraic form below, which has no branch issues and is faster.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from .spec import GearSpec

__all__ = [
    "DiscProfile",
    "ProfileSpline",
    "critical_radius",
    "disc_profile",
    "distance_to_polyline",
    "locus_curvature",
    "moment_arm",
    "pin_locus",
    "pin_locus_derivatives",
    "profile_curvature",
    "profile_from_spec",
    "profile_normal",
    "profile_spline",
    "profile_spline_from_spec",
    "sample_count_for_chord_tolerance",
    "sampled_profile",
]


def _K1(R: float, E: float, pins: int) -> float:
    return E * pins / R


def pin_locus(t: np.ndarray, R: float, E: float, lobes: int) -> np.ndarray:
    """Path a ring-pin centre traces in the disc's own frame.  Shape (n, 2)."""
    p = lobes + 1
    return np.column_stack([R * np.cos(t) - E * np.cos(p * t),
                            -R * np.sin(t) + E * np.sin(p * t)])


def pin_locus_derivatives(t: np.ndarray, R: float, E: float, lobes: int
                          ) -> tuple[np.ndarray, np.ndarray]:
    """Analytic first and second derivatives of :func:`pin_locus`."""
    p = lobes + 1
    d1 = np.column_stack([-R * np.sin(t) + E * p * np.sin(p * t),
                          -R * np.cos(t) + E * p * np.cos(p * t)])
    d2 = np.column_stack([-R * np.cos(t) + E * p * p * np.cos(p * t),
                          R * np.sin(t) - E * p * p * np.sin(p * t)])
    return d1, d2


def locus_curvature(t: np.ndarray, R: float, E: float, lobes: int) -> np.ndarray:
    """Signed curvature of the pin-centre locus, using the outward normal."""
    d1, d2 = pin_locus_derivatives(t, R, E, lobes)
    num = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    return num / np.power(d1[:, 0] ** 2 + d1[:, 1] ** 2, 1.5)


def _curvature_coefficients(R: float, E: float, lobes: int
                            ) -> tuple[float, float, float, float]:
    """Coefficients of the closed-form locus curvature.

    Expanding the derivatives collapses every trigonometric term into a single
    ``cos(N*t)``::

        kappa(u) = (A + B*u) / (C - D*u)**1.5,      u = cos(N*t)

    which turns curvature from a sampled search into arithmetic.
    """
    p = lobes + 1
    return (-(R * R + E * E * p ** 3),      # A
            R * E * p * (p + 1),            # B
            R * R + E * E * p * p,          # C
            2.0 * R * E * p)                # D


@lru_cache(maxsize=512)
def critical_radius(R: float, E: float, lobes: int, n: int = 0) -> float:
    """Largest pin radius the profile tolerates before it folds on itself.

    The offset curve degenerates where ``1 + Rr * kappa == 0``, so the limit is
    ``Rr < 1 / max(-kappa)``.

    Solved exactly rather than sampled.  With ``kappa`` written as a function of
    ``u = cos(N*t)`` the stationary condition is linear in ``u``, so the extreme
    is one division away and the answer costs nothing - which matters, because
    the design search evaluates this tens of thousands of times.  ``n`` is
    accepted and ignored for callers written against the sampled version.
    """
    a, b, c, d = _curvature_coefficients(R, E, lobes)
    if c - d <= 0.0:                       # K1 >= 1: the locus has cusps
        return 0.0

    def neg_kappa(u: float) -> float:
        return -(a + b * u) / (c - d * u) ** 1.5

    star = -(2.0 * b * c + 3.0 * d * a) / (b * d) if b * d != 0 else 0.0
    worst = max(neg_kappa(-1.0), neg_kappa(1.0))
    if -1.0 <= star <= 1.0:
        worst = max(worst, neg_kappa(star))
    return float("inf") if worst <= 0 else float(1.0 / worst)


def profile_normal(t: np.ndarray, R: float, E: float, lobes: int) -> np.ndarray:
    """Unit outward normal, shared by the locus and the disc profile."""
    p = lobes + 1
    k1 = _K1(R, E, p)
    a = np.cos(t) - k1 * np.cos(p * t)
    b = -(np.sin(t) - k1 * np.sin(p * t))
    d = np.sqrt(1.0 + k1 * k1 - 2.0 * k1 * np.cos(lobes * t))
    return np.column_stack([a / d, b / d])


def disc_profile(t: np.ndarray, R: float, Rr: float, E: float, lobes: int) -> np.ndarray:
    """The cycloidal disc outline in the disc frame.  Shape (n, 2).

    Verified: ``min_u |profile(t) - pin_locus(u)| == Rr`` for every t.
    """
    p = lobes + 1
    k1 = _K1(R, E, p)
    d = np.sqrt(1.0 + k1 * k1 - 2.0 * k1 * np.cos(lobes * t))
    x = R * np.cos(t) - E * np.cos(p * t) - Rr * (np.cos(t) - k1 * np.cos(p * t)) / d
    y = -R * np.sin(t) + E * np.sin(p * t) + Rr * (np.sin(t) - k1 * np.sin(p * t)) / d
    return np.column_stack([x, y])


def profile_curvature(t: np.ndarray, R: float, Rr: float, E: float, lobes: int) -> np.ndarray:
    """Signed curvature of the disc profile itself (needed for contact stress).

    Offsetting a curve by ``-Rr`` along its normal maps ``kappa -> kappa/(1 + Rr*kappa)``.
    """
    k = locus_curvature(t, R, E, lobes)
    return k / (1.0 + Rr * k)


def moment_arm(t: np.ndarray, R: float, Rr: float, E: float, lobes: int) -> np.ndarray:
    """Perpendicular distance from the disc centre to each contact normal.

    ``h = Q x n`` is invariant under the disc's own rotation, so it can be
    evaluated purely in the disc frame.
    """
    q = disc_profile(t, R, Rr, E, lobes)
    n = profile_normal(t, R, E, lobes)
    return q[:, 0] * n[:, 1] - q[:, 1] * n[:, 0]


@lru_cache(maxsize=512)
def sample_count_for_chord_tolerance(R: float, Rr: float, E: float, lobes: int,
                                     tol: float, lo: int = 720, hi: int = 200_000) -> int:
    """Points needed so the polyline never deviates from the true curve by ``tol``.

    A chord of length L across a curve of radius rho has sagitta ``L^2/(8*rho)``.
    Sampling is uniform in the parameter, not in arc length, so the binding point
    is where ``speed^2 * curvature`` peaks - not simply where the curve is
    tightest.  Solving ``(speed*dt)^2 * kappa / 8 <= tol`` for dt gives the count.

    Then rounded *up* onto a whole number of samples per lobe, which can only
    reduce the chord error and buys a property worth more than the handful of
    points it costs: the sampled polygon then has the disc's own symmetry.
    ``q(t + 2*pi/lobes) = rot(-2*pi/lobes) . q(t)`` holds for the curve, and with
    a count that divides by the lobe count it holds for the *vertices* too - so
    turning the drawn disc by one lobe pitch gives back the same polygon rather
    than one sampled a fraction of a step along.  ``hi`` is a ceiling on the
    expensive part and is allowed to be passed by less than one lobe.
    """
    t = np.linspace(0.0, 2.0 * np.pi, 8192, endpoint=False)
    h = 1e-6
    d1 = (disc_profile(t + h, R, Rr, E, lobes) -
          disc_profile(t - h, R, Rr, E, lobes)) / (2.0 * h)
    speed = np.hypot(d1[:, 0], d1[:, 1])
    kappa = np.abs(profile_curvature(t, R, Rr, E, lobes))
    worst = float((speed ** 2 * kappa).max())
    if worst <= 0.0:
        return lo + (-lo) % lobes
    dt = np.sqrt(8.0 * tol / worst)
    count = int(np.clip(np.ceil(2.0 * np.pi / dt), lo, hi))
    return count + (-count) % lobes


def distance_to_polyline(points: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Shortest distance from each of ``points`` to a closed polyline.

    Exact for the polygon, so the only error left is the chord error of the
    sampling.  That matters here: these distances are used as mesh clearances of
    a few hundredths of a millimetre, where a nearest-*vertex* answer would be
    wrong by more than the quantity being measured.
    """
    a = poly
    b = np.roll(poly, -1, axis=0)
    ab = b - a                                        # (n, 2)
    ap = points[:, None, :] - a[None, :, :]           # (m, n, 2)
    denom = np.maximum((ab * ab).sum(axis=1), 1e-30)  # (n,)
    u = np.clip((ap * ab[None, :, :]).sum(axis=2) / denom, 0.0, 1.0)
    closest = a[None, :, :] + u[:, :, None] * ab[None, :, :]
    d = points[:, None, :] - closest
    return np.sqrt((d * d).sum(axis=2)).min(axis=1)


@dataclass(frozen=True)
class DiscProfile:
    """Sampled disc outline plus the quantities the rest of the app needs."""

    t: np.ndarray
    points: np.ndarray
    R: float
    Rr: float
    E: float
    lobes: int

    @property
    def pins(self) -> int:
        return self.lobes + 1

    @property
    def closed(self) -> np.ndarray:
        """Points with the first vertex repeated, for polyline consumers."""
        return np.vstack([self.points, self.points[:1]])

    @property
    def radii(self) -> np.ndarray:
        return np.hypot(self.points[:, 0], self.points[:, 1])

    @property
    def outer_radius(self) -> float:
        return float(self.radii.max())

    @property
    def root_radius(self) -> float:
        return float(self.radii.min())

    def curvature(self) -> np.ndarray:
        return profile_curvature(self.t, self.R, self.Rr, self.E, self.lobes)

    def area(self) -> float:
        """Enclosed area by the shoelace formula, mm^2 (always positive)."""
        x, y = self.points[:, 0], self.points[:, 1]
        return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))

    def polar_second_moment(self) -> float:
        """Second moment of area about the disc centre, mm^4.

        Green's theorem on the closed polygon.  Needed for the disc's rotational
        inertia, which sets the reflected inertia and the unbalance force.
        """
        x, y = self.points[:, 0], self.points[:, 1]
        x1, y1 = np.roll(x, -1), np.roll(y, -1)
        cross = x * y1 - x1 * y
        ix = float(np.sum(cross * (y * y + y * y1 + y1 * y1))) / 12.0
        iy = float(np.sum(cross * (x * x + x * x1 + x1 * x1))) / 12.0
        return abs(ix + iy)


@lru_cache(maxsize=32)
def sampled_profile(R: float, Rr: float, E: float, lobes: int, n: int) -> DiscProfile:
    """Cached profile sampling.

    The UI rebuilds the same profile on every repaint, every validation pass and
    every animation frame, so this is worth memoising.  The arrays are handed out
    read-only: consumers of a cached object must not scribble on a buffer someone
    else is still holding.
    """
    t = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    pts = disc_profile(t, R, Rr, E, lobes)
    t.flags.writeable = False
    pts.flags.writeable = False
    return DiscProfile(t=t, points=pts, R=R, Rr=Rr, E=E, lobes=lobes)


def profile_from_spec(spec: GearSpec, n: int | None = None) -> DiscProfile:
    """Build the manufacturing profile, i.e. with clearance already applied."""
    R, Rr = spec.effective_R, spec.effective_Rr
    if n is None:
        n = sample_count_for_chord_tolerance(R, Rr, spec.eccentricity, spec.lobes,
                                             spec.dxf_chord_tolerance)
    return sampled_profile(R, Rr, spec.eccentricity, spec.lobes, int(n))


# ------------------------------------------------------- the profile as a curve
#
# The polyline above is for cutting: a chord that never leaves the true curve by
# more than the tolerance, which at 5 um takes about 1,800 vertices.  That is the
# right answer for CAM and the wrong one for a CAD package, where the profile is
# something to build *on* - and 1,800 vertices is not geometry you can reference,
# offset or dimension against, it is a cloud that happens to be in the shape of a
# disc.  What a modelling package wants is one curve.


@dataclass(frozen=True)
class ProfileSpline:
    """The disc outline as a single closed cubic B-spline.

    ``deviation`` is measured against the exact profile rather than argued for,
    because the whole value of the thing is that it is the same disc: a curve
    that is easier to work with and is not the profile is worse than no curve.
    """

    #: Control points of a *closed uniform* cubic B-spline, in order.  The curve
    #: does not pass through them - see :func:`profile_spline`.
    control_points: np.ndarray
    #: Greatest distance from the true profile to this curve, millimetres.
    deviation: float
    #: How many points on the curve were interpolated to get here.  Equal to the
    #: number of control points; kept because it is the knob that was turned.
    samples: int


def _arc_length_table(R: float, Rr: float, E: float, lobes: int,
                      dense: int = 20_000) -> tuple[np.ndarray, np.ndarray]:
    """Cumulative arc length against parameter, for resampling by length.

    Built once and handed to every trial fit: the search runs half a dozen of
    them and this is the expensive half of one.
    """
    t = np.linspace(0.0, 2.0 * np.pi, dense, endpoint=False)
    p = disc_profile(t, R, Rr, E, lobes)
    step = np.hypot(*(np.roll(p, -1, axis=0) - p).T)
    return (np.concatenate([t, [2.0 * np.pi]]),
            np.concatenate([[0.0], np.cumsum(step)]))


def _arc_length_samples(R: float, Rr: float, E: float, lobes: int, n: int,
                        table: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """``n`` points spread evenly along the curve rather than evenly in ``t``.

    This is the whole difference between a spline that fits and one that does
    not, and it is the same fact :func:`sample_count_for_chord_tolerance` is
    built on: the parameter does not move at a constant speed along the profile,
    and it moves slowest where the curve bends hardest.  Interpolating points
    spaced evenly in ``t`` therefore starves exactly the lobe crests that need
    them, and the fit converges like ``n**-1.7`` instead of the ``n**-4`` a cubic
    is capable of.  On the 21:1 that is 252 control points against 504 for the
    same error.

    Arc length is periodic over a lobe pitch just as ``t`` is, so a count that
    divides by ``lobes`` still lands the same points on every lobe and the curve
    keeps the disc's own symmetry.
    """
    t, s = table
    want = np.linspace(0.0, s[-1], n, endpoint=False)
    return disc_profile(np.interp(want, s, t), R, Rr, E, lobes)


def _closed_cubic_control_points(through: np.ndarray) -> np.ndarray:
    """Control points of the closed cubic B-spline that passes through ``through``.

    A cubic B-spline at its knots is ``(C[i-1] + 4 C[i] + C[i+1]) / 6``, so
    interpolation is one cyclic tridiagonal system.  Solved densely, which is
    wasteful and costs microseconds at these sizes.
    """
    n = len(through)
    a = (np.eye(n) * 4.0 + np.roll(np.eye(n), 1, axis=1)
         + np.roll(np.eye(n), -1, axis=1)) / 6.0
    return np.linalg.solve(a, through)


def evaluate_closed_bspline(control: np.ndarray, per_segment: int = 16) -> np.ndarray:
    """Points along a closed uniform cubic B-spline, in order.

    Public because it is what makes the deviation checkable: a control polygon
    is not a curve, and a test that compares control points against the profile
    would be comparing the wrong things.
    """
    n = len(control)
    u = np.linspace(0.0, 1.0, per_segment, endpoint=False)
    basis = np.stack([(1.0 - u) ** 3,
                      3.0 * u ** 3 - 6.0 * u ** 2 + 4.0,
                      -3.0 * u ** 3 + 3.0 * u ** 2 + 3.0 * u + 1.0,
                      u ** 3], axis=1) / 6.0
    span = (np.arange(n)[:, None] + np.array([-1, 0, 1, 2])) % n
    return (basis[None, :, :, None]
            * control[span][:, None, :, :]).sum(axis=2).reshape(-1, 2)


def _deviation(control: np.ndarray, exact: np.ndarray) -> float:
    """Greatest distance from the true curve to the spline, millimetres.

    Not through :func:`distance_to_polyline`, which compares every point against
    every segment: both curves want thousands of points here and the search runs
    half a dozen fits, so that is minutes of arithmetic for an answer of a few
    micrometres.  A tree finds the nearest drawn vertex and the answer is then
    the distance to one of the two segments meeting it - exact for the polygon,
    which is what the other routine buys as well.
    """
    from scipy.spatial import cKDTree

    drawn = evaluate_closed_bspline(control)
    near = cKDTree(drawn).query(exact)[1]
    # Each point takes the nearer of the two segments meeting its closest
    # vertex, and the answer is the worst of those - in that order.  The other
    # order asks a different question and gives a smaller number.
    best = None
    for offset in (-1, 0):
        a = drawn[(near + offset) % len(drawn)]
        b = drawn[(near + offset + 1) % len(drawn)]
        ab = b - a
        u = np.clip(((exact - a) * ab).sum(axis=1)
                    / np.maximum((ab * ab).sum(axis=1), 1e-30), 0.0, 1.0)
        d = exact - (a + u[:, None] * ab)
        to_segment = np.hypot(d[:, 0], d[:, 1])
        best = to_segment if best is None else np.minimum(best, to_segment)
    return float(best.max())


def profile_spline(R: float, Rr: float, E: float, lobes: int, tol: float,
                   hi: int = 4096) -> ProfileSpline:
    """Fit the profile with the fewest control points that stay within ``tol``.

    There is no closed form for this the way there is for the chord count, so it
    is searched: multiply up until the tolerance is met, then bisect back down
    onto the smallest multiple of ``lobes`` that still meets it.  Every step
    measures rather than estimates, and the count that comes out is reported
    beside the deviation it achieved.
    """
    exact = disc_profile(np.linspace(0.0, 2.0 * np.pi, 8192, endpoint=False),
                         R, Rr, E, lobes)
    table = _arc_length_table(R, Rr, E, lobes)

    def fit(n: int) -> tuple[np.ndarray, float]:
        control = _closed_cubic_control_points(
            _arc_length_samples(R, Rr, E, lobes, n, table))
        return control, _deviation(control, exact)

    low, n = 0, 4 * lobes
    control, dev = fit(n)
    while dev > tol and n < hi:
        low, n = n, min(hi, (int(n * 1.5) + lobes - 1) // lobes * lobes)
        control, dev = fit(n)

    # Bisect on whole lobes: a count that does not divide by the lobe count
    # gives up the disc's symmetry for a handful of points.
    best = (control, dev, n)
    while n - low > lobes:
        mid = (low + (n - low) // 2) // lobes * lobes
        if mid <= low:
            break
        control, dev = fit(mid)
        if dev <= tol:
            best, n = (control, dev, mid), mid
        else:
            low = mid
    return ProfileSpline(control_points=best[0], deviation=best[1], samples=best[2])


def profile_spline_from_spec(spec: GearSpec, tol: float | None = None) -> ProfileSpline:
    """The manufacturing profile as one curve, at the design's DXF tolerance."""
    return profile_spline(spec.effective_R, spec.effective_Rr, spec.eccentricity,
                          spec.lobes,
                          spec.dxf_chord_tolerance if tol is None else tol)
