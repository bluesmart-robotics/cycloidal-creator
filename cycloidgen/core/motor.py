"""What turns the input shaft: a mounting face, and a torque against speed.

Two different facts about the same object, and the app has only ever held the
first.  :class:`MotorFrame` is the *face* - a bolt pattern, a register and a
shaft diameter - which is standardised, so knowing the frame size tells you
where the holes go and nothing whatever about what comes out of the shaft.  A
NEMA 17 is sold from 0.09 to 0.65 Nm holding torque, and a NEMA 23 frame carries
brushless motors an order of magnitude apart.  So the curve is stated
separately, and none of it is inferred from the frame.

Why a curve rather than a number
--------------------------------
The rest of the app takes the duty as one output torque at one speed, works the
input torque back through the ratio and the efficiency, and stops - which
quietly assumes the motor delivers whatever the arithmetic asks of it at
whatever speed the arithmetic asks it at.  Neither motor here does.  A stepper's
torque falls with speed because the winding is an inductor and the supply runs
out of voltage to push current into it; a DC motor's falls because its own
back-EMF eats the supply.  Both have a speed past which they make no torque at
all, and it is a property of the *supply voltage* as much as of the motor.

That is the number people are most often wrong about.  Holding torque is quoted
at standstill and is what a stepper is bought on, and a drive designed on it is
designed on a figure the motor does not have at any speed it will run at.

The two models
--------------
**Stepper.**  Torque is proportional to phase current, and the current is what
the supply can force through the winding against its resistance, its inductive
reactance and its own back-EMF::

    V^2 = (K*w_m + I*R)^2 + (w_e*L*I)^2        solved for I, capped at I_rated
    T(n) = T_hold * I / I_rated

with ``w_e = 2*pi*N_r*n/60`` the electrical frequency, ``N_r = steps/4`` the
rotor teeth, and ``K = T_hold / (sqrt(2)*I_rated)`` the back-EMF constant.  That
last identity is the useful one and is worth stating: holding torque is measured
with *both* phases at rated current, so one phase contributes ``T_h/sqrt(2)``,
and the constant relating current to torque in a machine is the same constant
relating speed to back-EMF.  So a stepper's electrical ceiling comes out of its
mechanical rating with nothing else needed::

    n_ceiling = 60*V / (2*pi*K)

Above that the back-EMF alone exceeds the supply and no current flows whichever
way the driver switches.  On 24 V a 0.4 Nm, 1.7 A motor stops at about 1380 rpm;
on 12 V the same motor stops at 690.  Halving the supply halves the top speed,
which is the single most useful thing this model says.

**DC and brushless.**  The straight line between stall and no-load, exact for a
brushed motor on a fixed supply and first-order for a brushless one::

    Kt = 9.5493 / Kv      n_0 = Kv * V      T_stall = Kt * V / R
    T(n) = T_stall * (1 - n/n_0)

The stall end of that line is a fantasy in the sense that matters - it is drawn
at a current the motor cannot survive for more than a few seconds - so the
*continuous* torque is carried separately, at rated current, and it is the one
a drive should be sized on.  A 270 Kv motor on 24 V with 0.1 ohm windings stalls
at 8.5 Nm and can hold 0.7; sizing a gearbox on the first number is how you buy
one that survives the bench and not the robot.

What these ignore
-----------------
The stepper current here is what a driver holding the optimum phase angle could
force.  A step driver without phase advance does less, so published pull-out
curves fall away sooner through the middle of the range than this does - the
ceiling is firm, the shape approaching it is an upper bound.  Mid-band
resonance, which can stall a stepper outright at a few hundred rpm, is not in it
at all.  The brushless line ignores no-load current and iron loss, so it is
optimistic near no-load speed by whatever those cost.  Both take the supply as
stiff and the driver as able to reach the full bus voltage, which a bipolar
chopper does and a unipolar drive does not.  Use the manufacturer's curve where
you have one; this is for the case where you have a motor in a drawer and a
question about it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from pydantic import BaseModel, Field

from .fasteners import MetricSize

__all__ = [
    "COUPLINGS",
    "COUPLING_GAP",
    "COUPLING_SLOTS",
    "COUPLING_SLOT_DEPTH",
    "COUPLING_SLOT_WIDTH",
    "MOTOR_FIELDS",
    "MOTOR_FRAMES",
    "MOTOR_HUB_CLEARANCE",
    "MOTOR_HUB_WALL",
    "MOTOR_MOUNT_FLANGE",
    "MOTOR_MOUNT_WALL",
    "NO_MOTOR",
    "Coupling",
    "MotorCurve",
    "MotorFixing",
    "MotorFrame",
    "MotorKind",
    "coupling_for",
    "curve_from",
    "motor_fixing",
    "mount_flange",
]

#: The eight fields a curve is made of, under the names the spec gives them.
#:
#: One list rather than five copies of the same eight strings.  They are carried
#: by :class:`~cycloidgen.core.spec.GearSpec`, mirrored by the search's
#: requirements, handed on to every candidate the search builds, passed through
#: the search dialog and read off a design file by the command line - and a
#: ninth number added to the model with any one of those places missed is a
#: motor that silently changes on the way past.  ``tests/test_motor.py`` checks
#: this against the spec itself, so the list cannot fall behind the model.
MOTOR_FIELDS: tuple[str, ...] = (
    "motor_kind",
    "motor_supply_V",
    "motor_resistance_ohm",
    "motor_rated_current_A",
    "motor_holding_torque_Nm",
    "motor_inductance_mH",
    "motor_steps_per_rev",
    "motor_kv_rpm_per_V",
)


class MotorFrame(BaseModel):
    """A motor's mounting face, as the standard defines it.

    NEMA frames put four bolts on a **square**, not on a bolt circle, and the
    difference is not cosmetic: drawing four holes on a circle of the same size
    puts every one of them in the wrong place.  Metric flanges do use a circle,
    so both are here and ``square`` says which.

    ``max_radial_N`` is what the motor's *own* bearings will take at the shaft
    end.  It is a typical figure for the frame size rather than a promise about
    your motor - they vary by a factor of two between makers - and it exists so
    that hanging a drive on a motor face can be checked against something
    instead of hoped about.  Read your datasheet before you believe it.

    Nothing here says what the motor *does*.  That is :class:`MotorCurve`, and
    it is deliberately not derived from the frame: the frame is a face, and
    every frame size is sold across an order of magnitude of torque.
    """

    name: str
    bolt_span: float = Field(gt=0, description="square side, or bolt circle dia")
    square: bool = Field(True, description="four on a square, or N on a circle")
    bolt_count: int = Field(4, ge=1)
    bolt_diameter: float = Field(gt=0, description="clearance hole, not the thread")
    pilot_diameter: float = Field(gt=0, description="the register that centres it")
    pilot_depth: float = Field(gt=0)
    shaft_diameter: float = Field(gt=0)
    max_radial_N: float = Field(gt=0, description="typical, at the shaft end")

    # ---- the rest of the motor, as far as fitting it goes ---------------------
    #
    # What the drive has to make room for and fasten into.  The face above says
    # where the holes go; these say what the motor is behind them - which is
    # what decides whether its shaft reaches the cam, where the screws can go
    # in from, and what it looks like bolted on.  Typical for the frame, as the
    # radial rating is: lengths in particular vary by stack height, and a
    # datasheet beats all of them.
    body_width: float = Field(0.0, ge=0, description="square body, across flats")
    body_length: float = Field(0.0, ge=0, description="behind the mounting face")
    shaft_length: float = Field(0.0, ge=0,
                                description="past the mounting face, spigot included")
    shaft_flat: float = Field(0.0, ge=0,
                              description="depth of the D-cut; 0 for a round shaft")
    #: ``tapped``: threads in the motor's face, so the screws go in from the
    #: drive's side.  ``through``: clearance holes through the motor's flange,
    #: so they go in from the motor's side and need something to bite on.
    fixing: str = Field("tapped")
    #: The screw size the face takes, by its metric name.
    screw: str = Field("")
    #: Tapped: how deep the motor's threads go.  Through: the flange thickness
    #: the screws pass.
    fixing_depth: float = Field(0.0, ge=0)

    @property
    def bolt_circle_diameter(self) -> float:
        """Where the bolts actually land, whichever way the pattern is stated.

        A square of side ``s`` puts its corners on a circle of ``s*sqrt(2)``, and
        that is the number every clearance check wants.
        """
        return self.bolt_span * (math.sqrt(2.0) if self.square else 1.0)


#: What a motor face is, by frame size.  Standard NEMA geometry; the radial
#: ratings are typical for the frame rather than any one motor.  ``None`` is a
#: plate with no motor interface at all - a drive driven through a coupling from
#: something this table does not describe.
MOTOR_FRAMES: dict[str, MotorFrame] = {
    m.name: m
    for m in [
        MotorFrame(name="None", bolt_span=1.0, bolt_diameter=1.0,
                   pilot_diameter=1.0, pilot_depth=0.1, shaft_diameter=1.0,
                   max_radial_N=1.0),
        # The small frames thread their face and the large ones put clearance
        # holes through a flange, which decides which side a screw goes in
        # from.  Shafts: a D-cut half a millimetre deep on the small ones, a
        # keyway rather than a flat on a NEMA 34.
        MotorFrame(name="NEMA 8", bolt_span=15.4, bolt_diameter=2.2,
                   pilot_diameter=15.0, pilot_depth=1.6, shaft_diameter=4.0,
                   max_radial_N=10.0, body_width=20.3, body_length=30.0,
                   shaft_length=10.0, shaft_flat=0.0, fixing="tapped",
                   screw="M2", fixing_depth=2.5),
        MotorFrame(name="NEMA 11", bolt_span=23.0, bolt_diameter=2.7,
                   pilot_diameter=22.0, pilot_depth=2.0, shaft_diameter=5.0,
                   max_radial_N=15.0, body_width=28.2, body_length=32.0,
                   shaft_length=20.0, shaft_flat=0.5, fixing="tapped",
                   screw="M2.5", fixing_depth=2.5),
        MotorFrame(name="NEMA 14", bolt_span=26.0, bolt_diameter=3.2,
                   pilot_diameter=22.0, pilot_depth=2.0, shaft_diameter=5.0,
                   max_radial_N=20.0, body_width=35.2, body_length=34.0,
                   shaft_length=20.0, shaft_flat=0.5, fixing="tapped",
                   screw="M3", fixing_depth=3.5),
        MotorFrame(name="NEMA 17", bolt_span=31.0, bolt_diameter=3.4,
                   pilot_diameter=22.0, pilot_depth=2.0, shaft_diameter=5.0,
                   max_radial_N=28.0, body_width=42.3, body_length=40.0,
                   shaft_length=24.0, shaft_flat=0.5, fixing="tapped",
                   screw="M3", fixing_depth=4.5),
        MotorFrame(name="NEMA 23", bolt_span=47.14, bolt_diameter=5.5,
                   pilot_diameter=38.1, pilot_depth=1.6, shaft_diameter=6.35,
                   max_radial_N=75.0, body_width=56.4, body_length=56.0,
                   shaft_length=21.0, shaft_flat=0.5, fixing="through",
                   screw="M5", fixing_depth=5.0),
        MotorFrame(name="NEMA 34", bolt_span=69.58, bolt_diameter=5.5,
                   pilot_diameter=73.03, pilot_depth=2.0, shaft_diameter=14.0,
                   max_radial_N=220.0, body_width=86.0, body_length=80.0,
                   shaft_length=32.0, shaft_flat=0.0, fixing="through",
                   screw="M5", fixing_depth=10.0),
    ]
}

#: The frame name that means "no motor interface on this plate".
NO_MOTOR = "None"

#: How far the input shaft stops short of the motor's spigot, and the bore
#: runs past the motor shaft's tip, mm: room to assemble, nothing more.
MOTOR_HUB_CLEARANCE = 0.5
#: The least wall the input shaft keeps round the motor's shaft when it is
#: bored to take it, mm.  Below this it is a sleeve, not a shaft.
MOTOR_HUB_WALL = 1.0


@dataclass(frozen=True)
class MotorFixing:
    """The screws a motor is held to its plate by.

    ``from_drive``: the motor's face is threaded, so the screws go in from the
    drive's side, through the plate, and their heads sit in counterbores in
    the plate's *inside* face - flush or below it, because that face runs
    against the discs with nothing to spare.  Otherwise they go in from the
    motor's side through its flange, and a nut sits in a hexagonal pocket in
    the same inside face.  Either way something is let into the face the
    discs run on, and ``recess`` and ``recess_depth`` are what is cut for it:
    a counterbore's diameter, or a pocket's width across flats.
    """

    size: MetricSize
    length: float
    from_drive: bool
    recess: float
    recess_depth: float
    plate: float
    #: How much thread holds: in the motor's face, or in the nut.
    engaged: float

    @property
    def fits(self) -> bool:
        """Whether a preferred length holds with enough thread, and the plate
        keeps enough of itself under the recess to clamp."""
        thread = self.size.d if self.from_drive else self.size.nut_m
        return (self.engaged >= MIN_ENGAGEMENT * thread
                and self.plate - self.recess_depth >= MIN_UNDER_RECESS)


#: Thread engaged below which a screw is a pin with ambitions: a share of its
#: diameter in the motor's own threads, or of the nut's height in a nut.
MIN_ENGAGEMENT = 0.8
#: Plate left under a counterbore or pocket, mm, for the screw to clamp on.
MIN_UNDER_RECESS = 1.5


@dataclass(frozen=True)
class Coupling:
    """A shaft coupling, by its outside and its length.

    The common aluminium beam couplings, at the sizes they are sold in: first-
    pass figures for room and fit, like the bearing catalogue's.  ``catalogue``
    is False for one this table could not supply, drawn plain to the shafts it
    has to join so the drive is still a drive - and reported.
    """

    name: str
    outer: float
    length: float
    min_bore: float
    max_bore: float
    catalogue: bool = True


COUPLINGS: tuple[Coupling, ...] = (
    Coupling("D16 x L23", 16.0, 23.0, 3.0, 6.35),
    Coupling("D19 x L25", 19.0, 25.0, 4.0, 8.0),
    Coupling("D25 x L30", 25.0, 30.0, 5.0, 12.0),
    Coupling("D32 x L40", 32.0, 40.0, 8.0, 16.0),
    Coupling("D40 x L50", 40.0, 50.0, 10.0, 20.0),
)

#: Between the two shafts' ends inside a coupling, and between a coupling
#: and anything it turns past, mm.
COUPLING_GAP = 1.0
#: A beam coupling's cuts, drawn as rings round its middle an eighth of its
#: length apart: how many, how wide and how deep, mm.  The real ones are a
#: helix; rings take out about as much metal, and the mass model can weigh
#: what the solid draws.
COUPLING_SLOTS = 3
COUPLING_SLOT_WIDTH = 0.8
COUPLING_SLOT_DEPTH = 1.5
#: The motor mount's tube wall, and the thinnest the flange the motor bolts
#: to on top of it is, mm.
MOTOR_MOUNT_WALL = 3.0
MOTOR_MOUNT_FLANGE = 6.0


def mount_flange(frame: MotorFrame) -> float:
    """How thick a motor mount's flange is for ``frame``, mm.

    At least :data:`MOTOR_MOUNT_FLANGE`.  On a threaded face the screws come up
    through it with their heads underneath, and a preferred length either
    fills the motor's threads or falls short of them by up to a pitch of the
    length series - two millimetres of an M3's three and a half.  So the
    flange is the thickness that makes the shortest preferred screw through
    it fill them exactly.  A flanged motor takes nuts, and any thickness.
    """
    from .fasteners import SCREW_LENGTHS

    if frame.fixing != "tapped":
        return MOTOR_MOUNT_FLANGE
    thread = frame.fixing_depth - 0.3
    return next(n - thread for n in SCREW_LENGTHS
                if n - thread >= MOTOR_MOUNT_FLANGE - 1e-9)


def coupling_for(a: float, b: float) -> Coupling:
    """The smallest catalogue coupling that takes shafts ``a`` and ``b``, or a
    plain sleeve sized off them when none does."""
    small, large = sorted((a, b))
    for c in COUPLINGS:
        if c.min_bore <= small + 1e-9 and large <= c.max_bore + 1e-9:
            return c
    return Coupling(f"sleeve {small:g}/{large:g}", round(2.5 * large, 1),
                    round(3.0 * large, 1), small, large, catalogue=False)


def motor_fixing(frame: MotorFrame, plate: float,
                 recessed: bool = True) -> MotorFixing | None:
    """How ``frame`` is screwed to a plate ``plate`` thick, or None for a frame
    whose screw this build does not know.

    The length is ISO 4762's longest preferred one that does not bottom out:
    in the motor's threads short of their depth, or through the nut short of
    the face the discs run on.  The longest, because thread is what holds.

    Not ``recessed``: the plate is a motor mount's flange, with air under it
    rather than discs, so a head or a nut simply sits on its underside - and a
    screw through a nut is the shortest preferred one that comes out of it.
    """
    from .fasteners import SCREW_LENGTHS, size_named

    size = size_named(frame.screw)
    if size is None:
        return None
    if not recessed:
        if frame.fixing == "tapped":
            longest = plate + frame.fixing_depth - 0.3
            length = max((n for n in SCREW_LENGTHS if n <= longest + 1e-9),
                         default=SCREW_LENGTHS[0])
            return MotorFixing(size=size, length=length, from_drive=True,
                               recess=0.0, recess_depth=0.0, plate=plate,
                               engaged=length - plate)
        need = frame.fixing_depth + plate + size.nut_m + 2.0 * size.pitch
        length = next((n for n in SCREW_LENGTHS if n >= need - 1e-9),
                      SCREW_LENGTHS[-1])
        return MotorFixing(size=size, length=length, from_drive=False, recess=0.0,
                           recess_depth=0.0, plate=plate, engaged=size.nut_m)
    if frame.fixing == "tapped":
        depth = size.head_h + 0.3
        under = plate - depth
        longest = under + frame.fixing_depth - 0.3
        length = max((n for n in SCREW_LENGTHS if n <= longest + 1e-9),
                     default=SCREW_LENGTHS[0])
        return MotorFixing(size=size, length=length, from_drive=True,
                           recess=size.head_d + 0.6, recess_depth=depth,
                           plate=plate, engaged=length - under)
    depth = size.nut_m + 0.2
    through = frame.fixing_depth + plate
    length = max((n for n in SCREW_LENGTHS if n <= through - 0.2 + 1e-9),
                 default=SCREW_LENGTHS[0])
    return MotorFixing(size=size, length=length, from_drive=False,
                       recess=size.nut_s + 0.4, recess_depth=depth, plate=plate,
                       engaged=min(length - (through - depth), size.nut_m))


class MotorKind(str, Enum):
    """Which curve the thing on the input shaft follows.

    ``NONE`` is not "no motor" - something always turns the crank.  It is "no
    curve stated", and it is the default, so a design that has never been told
    what drives it gets exactly the answers it always got.
    """

    NONE = "none"
    STEPPER = "stepper"
    DC = "DC or brushless"


#: Nm/A per rpm/V.  ``Kt = 60/(2*pi*Kv)``, written out because seeing the number
#: is how you catch a Kv entered in rad/s per volt.
_KT_PER_KV = 60.0 / (2.0 * math.pi)


@dataclass(frozen=True)
class MotorCurve:
    """Torque against speed for one motor on one supply.

    Built from :class:`~cycloidgen.core.spec.GearSpec` rather than stored on it,
    so the eight numbers the panel holds stay the source of truth and there is
    no second copy to drift.  Every method takes rpm and returns Nm.
    """

    kind: MotorKind
    #: Bus volts.  Both models scale with it, and it is usually the cheapest
    #: thing in the design to change.
    supply_V: float
    #: Winding resistance: per phase for a stepper, terminal to terminal for a
    #: DC motor.  Whichever one the datasheet prints.
    resistance_ohm: float
    #: The current it will take all day: the driver setting for a stepper, the
    #: continuous rating for a DC motor.
    rated_current_A: float
    #: Stepper: holding torque, both phases at rated current.
    holding_torque_Nm: float = 0.0
    #: Stepper: inductance per phase, mH.
    inductance_mH: float = 0.0
    #: Stepper: full steps per revolution.  200 is 1.8 degrees, 400 is 0.9.
    steps_per_rev: int = 200
    #: DC: no-load speed per volt.
    kv_rpm_per_V: float = 0.0

    # ---------------------------------------------------------------- shared --
    @property
    def modelled(self) -> bool:
        """Whether there is a curve to ask about at all."""
        return self.kind is not MotorKind.NONE

    @property
    def torque_constant_Nm_per_A(self) -> float:
        """Nm per amp, which is also volt-seconds per radian.

        One constant, both directions - that is what makes a stepper's top speed
        computable from its holding torque.  For the stepper it is per phase, so
        the ``sqrt(2)`` that takes the two-phase holding torque down to one
        phase is in it; see the module docstring.
        """
        if self.kind is MotorKind.STEPPER:
            return self.holding_torque_Nm / (math.sqrt(2.0)
                                             * max(self.rated_current_A, 1e-9))
        if self.kind is MotorKind.DC:
            return _KT_PER_KV / max(self.kv_rpm_per_V, 1e-9)
        return 0.0

    @property
    def stall_torque_Nm(self) -> float:
        """Torque at zero speed, at whatever current the supply can push there.

        For a stepper that is the holding torque, unless the supply cannot force
        rated current through the winding even standing still - which is a real
        case on a high-resistance motor and a low bus, and is the one place the
        resistance decides anything on its own.
        """
        if not self.modelled:
            return math.inf
        return self.torque_at(0.0)

    @property
    def ceiling_rpm(self) -> float:
        """The speed past which the motor makes no torque, loaded or not.

        Back-EMF alone reaches the supply voltage here, so no current flows
        whichever way the driver switches.  It is the same quantity for both
        models - ``V/K`` in radians, which is ``Kv*V`` for a DC motor by
        definition - and it is a property of the supply as much as of the motor.
        """
        if not self.modelled:
            return math.inf
        k = self.torque_constant_Nm_per_A
        if k <= 0:
            return math.inf
        return 60.0 * self.supply_V / (2.0 * math.pi * k)

    def torque_at(self, rpm: float) -> float:
        """What the motor can make at this speed, Nm.

        ``inf`` with no curve stated, so a caller that forgets to check
        :attr:`modelled` fails loudly on a plot rather than quietly on a margin.
        """
        if not self.modelled:
            return math.inf
        rpm = abs(rpm)
        if self.kind is MotorKind.STEPPER:
            return self.holding_torque_Nm * self._stepper_current_fraction(rpm)
        n0 = self.kv_rpm_per_V * self.supply_V
        if n0 <= 0 or rpm >= n0:
            return 0.0
        return self._dc_stall_torque() * (1.0 - rpm / n0)

    def continuous_torque_at(self, rpm: float) -> float:
        """What it can make there *and go on making*, Nm.

        The two models differ in kind here and it matters.  A stepper is already
        current limited at its rated current everywhere on the curve, so its
        continuous torque is its curve - there is no second, lower line.  A DC
        motor's curve is drawn at stall current and its continuous line is drawn
        at rated current, and on a brushless motor those are two orders of
        magnitude apart.
        """
        if not self.modelled:
            return math.inf
        if self.kind is MotorKind.STEPPER:
            return self.torque_at(rpm)
        return min(self.torque_at(rpm),
                   self.torque_constant_Nm_per_A * self.rated_current_A)

    def speed_for_torque(self, torque_Nm: float) -> float:
        """The fastest speed at which the motor still makes ``torque_Nm``, rpm.

        What turns "will this work" into "how fast can it go", which is the
        question a drive that fails the torque check actually raises.  Zero if
        it cannot make that torque standing still either.
        """
        if not self.modelled:
            return math.inf
        if torque_Nm <= 0:
            return self.ceiling_rpm
        if self.kind is MotorKind.STEPPER:
            return self._stepper_speed_for(torque_Nm)
        stall = self._dc_stall_torque()
        n0 = self.kv_rpm_per_V * self.supply_V
        if torque_Nm >= stall or n0 <= 0:
            return 0.0
        return n0 * (1.0 - torque_Nm / stall)

    def continuous_speed_for_torque(self, torque_Nm: float) -> float:
        """The same question asked of the continuous line rather than the peak.

        For a stepper the two are one curve and this is
        :meth:`speed_for_torque`.  For a DC motor the continuous rating is a
        horizontal ceiling, so asking for more than it is a flat no at every
        speed rather than a speed limit - which is the honest answer and not the
        one the peak line gives.
        """
        if not self.modelled:
            return math.inf
        if self.kind is MotorKind.STEPPER:
            return self.speed_for_torque(torque_Nm)
        if torque_Nm > self.torque_constant_Nm_per_A * self.rated_current_A:
            return 0.0
        return self.speed_for_torque(torque_Nm)

    # --------------------------------------------------------------- stepper --
    def _electrical_rad_s(self, rpm: float) -> float:
        """Winding frequency, rad/s.  Four full steps to one electrical cycle,
        so a 200-step motor turns 50 electrical cycles per revolution."""
        rotor_teeth = max(self.steps_per_rev, 4) / 4.0
        return 2.0 * math.pi * rotor_teeth * rpm / 60.0

    def _stepper_current_fraction(self, rpm: float) -> float:
        """Phase current as a fraction of rated, 0 to 1.

        Solves ``V^2 = (E + I*R)^2 + (w_e*L*I)^2`` for ``I``, which is a
        quadratic in ``I`` because the back-EMF does not depend on it.  Taking
        the positive root and capping at rated current is the whole model.
        """
        rated = max(self.rated_current_A, 1e-9)
        emf = self.torque_constant_Nm_per_A * 2.0 * math.pi * rpm / 60.0
        if emf >= self.supply_V:
            return 0.0
        reactance = self._electrical_rad_s(rpm) * self.inductance_mH / 1000.0
        a = self.resistance_ohm ** 2 + reactance ** 2
        if a <= 0:
            return 1.0
        b = 2.0 * emf * self.resistance_ohm
        c = emf ** 2 - self.supply_V ** 2
        current = (-b + math.sqrt(max(b * b - 4.0 * a * c, 0.0))) / (2.0 * a)
        return min(current / rated, 1.0)

    def _stepper_speed_for(self, torque_Nm: float) -> float:
        """Invert the stepper curve by bisection.

        The curve falls monotonically and is bounded above by
        :attr:`ceiling_rpm`, so bisection converges to floating point in sixty
        halvings and cannot wander off.  The algebraic inverse exists, but the
        reactance and the back-EMF scale with different constants, so it is a
        page of terms to save microseconds nobody is spending here.
        """
        if torque_Nm > self.holding_torque_Nm:
            return 0.0
        hi = self.ceiling_rpm
        if not math.isfinite(hi):
            return 0.0
        lo = 0.0
        if self.torque_at(lo) < torque_Nm:
            return 0.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if self.torque_at(mid) >= torque_Nm:
                lo = mid
            else:
                hi = mid
        return lo

    # -------------------------------------------------------------------- DC --
    def _dc_stall_torque(self) -> float:
        if self.resistance_ohm <= 0:
            return math.inf
        return (self.torque_constant_Nm_per_A * self.supply_V
                / self.resistance_ohm)


def curve_from(source) -> MotorCurve:
    """Assemble a curve off anything carrying the eight :data:`MOTOR_FIELDS`.

    A spec and a set of search requirements both do, and both wanted the same
    eight-line constructor call written out.  One function instead, so the two
    cannot come to disagree about which field feeds which part of the model -
    which is a mistake that produces a plausible curve rather than an error.
    """
    return MotorCurve(
        kind=source.motor_kind,
        supply_V=source.motor_supply_V,
        resistance_ohm=source.motor_resistance_ohm,
        rated_current_A=source.motor_rated_current_A,
        holding_torque_Nm=source.motor_holding_torque_Nm,
        inductance_mH=source.motor_inductance_mH,
        steps_per_rev=source.motor_steps_per_rev,
        kv_rpm_per_V=source.motor_kv_rpm_per_V,
    )
