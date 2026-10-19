"""Single source of truth for a cycloidal drive design.

Every other module consumes a ``GearSpec``.  All lengths are millimetres,
all angles radians unless a name says otherwise.
"""
from __future__ import annotations

import math
from enum import Enum
from typing import Literal

from pydantic import BaseModel, Field, computed_field, model_validator

# The motor lives in its own module because it is two separate facts - a face to
# bolt to and a torque against speed - and only the first of them is geometry.
# Re-exported here because every consumer already reaches for the spec, and a
# name that moves house is a name that breaks a dozen imports for no gain.
from .duty import DutyCycle
from .fasteners import (
    SCREW_LENGTHS,
    SIZES,
    HeatSetInsert,
    MetricSize,
    dowel_for_wall,
    dowel_length,
    insert_for,
    screw_length,
    size_for_hole,
    size_named,
)
from .motor import (
    COUPLING_GAP,
    MOTOR_FRAMES,
    MOTOR_HUB_CLEARANCE,
    MOTOR_MOUNT_WALL,
    NO_MOTOR,
    ROTOR_CAN_THREAD,
    ROTOR_HUB_RIM,
    ROTOR_HUB_THICKNESS,
    Coupling,
    MotorCurve,
    MotorFixing,
    MotorFrame,
    MotorKind,
    coupling_for,
    curve_from,
    motor_fixing,
    mount_flange,
)


class Material(BaseModel):
    """Elastic, strength and tribological data for one part."""

    name: str
    E_GPa: float = Field(gt=0, description="Young's modulus")
    nu: float = Field(gt=0, lt=0.5, description="Poisson's ratio")
    sigma_contact_MPa: float = Field(
        gt=0, description="allowable Hertzian contact pressure (rough, design guide)"
    )
    density_g_cm3: float = Field(gt=0)
    sigma_yield_MPa: float = Field(
        gt=0, description="yield (metals) or tensile strength (polymers)"
    )
    pv_limit_MPa_m_s: float = Field(
        gt=0,
        description="limiting PV for dry sliding against steel, projected-area "
                    "convention - the wear/melting limit, not a strength limit",
    )
    max_service_temp_C: float = Field(
        gt=0, description="continuous service temperature; polymers creep well below it"
    )
    sigma_ultimate_MPa: float = Field(
        gt=0, description="ultimate tensile strength"
    )
    fatigue_strength_MPa: float | None = Field(
        default=None,
        description="uncorrected fatigue strength for fully reversed bending - the "
                    "endurance limit where the material has one, otherwise the "
                    "strength at the life quoted in MATERIALS. None where no "
                    "defensible value exists, which is every polymer here",
    )

    @property
    def shear_allow_MPa(self) -> float:
        """Von Mises shear allowable."""
        return 0.577 * self.sigma_yield_MPa

    @property
    def has_fatigue_data(self) -> bool:
        """Whether a fatigue life can be estimated for this material at all.

        The alternative to answering "no" here is a number derived from a rule
        that was fitted to wrought metals, applied to a printed polymer whose
        fatigue behaviour depends on layer orientation more than on its tensile
        strength.  That is worse than silence.
        """
        return self.fatigue_strength_MPa is not None


#: Rough design-guide values.  Contact allowables for polymers are conservative
#: because they creep; the steel values assume a hardened, ground surface.  The
#: PV limits are dry-against-steel figures on the projected-area convention, so
#: they are only meaningful against a projected-area pressure - see
#: :mod:`cycloidgen.analysis.thermal`.  100Cr6's service temperature is the
#: dimensional-stability limit of standard bearing steel, not a strength limit.
#: Fatigue strengths are for fully reversed bending and **uncorrected** - the
#: specimen value, before surface, size, temperature and reliability are applied
#: in :mod:`cycloidgen.analysis.fatigue`.  Steels get the endurance limit they
#: actually have, which is why 100Cr6 is not 0.5*Sut: that rule stops holding
#: above about 1400 MPa and the limit flattens off near 700.  Aluminium and
#: bronze have no endurance limit at all, so those are strengths at 5e8 cycles
#: and a design that runs longer than that is outside them.  Polymers get
#: ``None``: printed-part fatigue turns on layer orientation, void content and
#: temperature far more than on tensile strength, and a Marin-corrected steel
#: rule applied to PLA would be a confident number with nothing behind it.
MATERIALS: dict[str, Material] = {
    m.name: m
    for m in [
        Material(name="PLA", E_GPa=3.5, nu=0.36, sigma_contact_MPa=25, density_g_cm3=1.24,
                 sigma_yield_MPa=50, pv_limit_MPa_m_s=0.03, max_service_temp_C=55,
                 sigma_ultimate_MPa=50),
        Material(name="PETG", E_GPa=2.1, nu=0.40, sigma_contact_MPa=30, density_g_cm3=1.27,
                 sigma_yield_MPa=50, pv_limit_MPa_m_s=0.04, max_service_temp_C=70,
                 sigma_ultimate_MPa=50),
        Material(name="ABS", E_GPa=2.2, nu=0.35, sigma_contact_MPa=24, density_g_cm3=1.04,
                 sigma_yield_MPa=40, pv_limit_MPa_m_s=0.05, max_service_temp_C=85,
                 sigma_ultimate_MPa=40),
        Material(name="PA12 (SLS)", E_GPa=1.7, nu=0.40, sigma_contact_MPa=35, density_g_cm3=1.01,
                 sigma_yield_MPa=48, pv_limit_MPa_m_s=0.12, max_service_temp_C=150,
                 sigma_ultimate_MPa=48),
        Material(name="Tough Resin (SLA)", E_GPa=2.7, nu=0.40, sigma_contact_MPa=30, density_g_cm3=1.18,
                 sigma_yield_MPa=45, pv_limit_MPa_m_s=0.04, max_service_temp_C=60,
                 sigma_ultimate_MPa=45),
        Material(name="POM (Delrin)", E_GPa=3.0, nu=0.38, sigma_contact_MPa=40, density_g_cm3=1.41,
                 sigma_yield_MPa=70, pv_limit_MPa_m_s=0.14, max_service_temp_C=90,
                 sigma_ultimate_MPa=70),
        Material(name="Aluminium 7075-T6", E_GPa=71.7, nu=0.33, sigma_contact_MPa=350, density_g_cm3=2.81,
                 sigma_yield_MPa=500, pv_limit_MPa_m_s=0.35, max_service_temp_C=150,
                 sigma_ultimate_MPa=572, fatigue_strength_MPa=159),
        Material(name="Bronze CuSn12", E_GPa=100.0, nu=0.34, sigma_contact_MPa=200, density_g_cm3=8.8,
                 sigma_yield_MPa=150, pv_limit_MPa_m_s=1.75, max_service_temp_C=200,
                 sigma_ultimate_MPa=280, fatigue_strength_MPa=90),
        Material(name="Steel 1045", E_GPa=205.0, nu=0.29, sigma_contact_MPa=800, density_g_cm3=7.85,
                 sigma_yield_MPa=450, pv_limit_MPa_m_s=1.0, max_service_temp_C=250,
                 sigma_ultimate_MPa=625, fatigue_strength_MPa=310),
        Material(name="Steel 4140 (hardened)", E_GPa=210.0, nu=0.29, sigma_contact_MPa=1400, density_g_cm3=7.85,
                 sigma_yield_MPa=900, pv_limit_MPa_m_s=1.4, max_service_temp_C=300,
                 sigma_ultimate_MPa=1100, fatigue_strength_MPa=550),
        Material(name="Bearing steel 100Cr6", E_GPa=210.0, nu=0.30, sigma_contact_MPa=1800, density_g_cm3=7.81,
                 sigma_yield_MPa=1600, pv_limit_MPa_m_s=1.8, max_service_temp_C=120,
                 sigma_ultimate_MPa=2000, fatigue_strength_MPa=700),
    ]
}


class Lubricant(BaseModel):
    """What is between the two surfaces, and what it does under pressure.

    Two properties decide whether a film forms at all.  **Viscosity** is the
    obvious one and it is quoted the way oils are actually sold - kinematic, in
    centistokes, at 40 and 100 C, which is what an ISO VG grade *is*.  Two points
    are enough to fit the temperature curve, and a lubricant's whole behaviour
    here turns on temperature: the drive heats itself, the oil thins, the film
    collapses, and the friction that caused it goes up again.

    The second is **pressure-viscosity**, ``alpha``.  At a loaded contact the oil
    sees a gigapascal or so and stiffens by orders of magnitude, and that - not
    its viscosity in the can - is what holds the surfaces apart.  An oil with a
    low alpha builds a thinner film at the same grade, which is why the synthetic
    entries here are not simply better than the mineral ones.

    ``boundary_mu`` is what the contact costs when the film has failed and the
    asperities are touching: the additive package, not the base oil, and the
    reason an EP grease is worth having in a drive that will spend its life in
    the mixed regime.
    """

    name: str
    viscosity_40C_cSt: float = Field(gt=0, description="kinematic; ISO VG is this number")
    viscosity_100C_cSt: float = Field(gt=0)
    density_g_cm3: float = Field(gt=0, description="to get dynamic viscosity from it")
    alpha_1_per_GPa: float = Field(
        gt=0, description="pressure-viscosity coefficient; what actually builds film"
    )
    boundary_mu: float = Field(
        gt=0, lt=1.0, description="friction once the film has gone and asperities touch"
    )
    max_temp_C: float = Field(gt=0, description="above this it degrades rather than thins")

    @property
    def forms_a_film(self) -> bool:
        """Whether asking for a film thickness means anything for this entry.

        ``False`` for dry, which is not a thin lubricant but the absence of one -
        there is no viscosity to put in the formula and a number would be an
        invention - and for a bonded dry film, which works by shearing at a low
        stress rather than by separating the surfaces.  Both go straight to their
        boundary coefficient, which for the dry entry is the design's own.
        """
        return self.name not in _NO_FILM


#: The lubricant field's value when there is nothing in there.  Dry is the
#: default and the honest one for most printed drives - people build them, run
#: them dry, and wear them out, which is what the PV checks are about.
DRY = "None (dry)"

#: Lubricants by what you would actually put in.  The greases are quoted by
#: their base oil, because that is what forms the film - the thickener holds it
#: in place and does not carry load.  Alphas are typical values for the chemistry
#: at room temperature: mineral oils run high, PAO synthetics lower, and that
#: difference is why a synthetic of the same grade builds a thinner film.
#: ``PTFE dry film`` is not a lubricant in this model's sense at all - it forms
#: no hydrodynamic film ever - but it has a low boundary coefficient, which is
#: the whole reason to use one, so it is here with the viscosity of the base it
#: is carried in and a flag that stops the film formula being applied to it.
LUBRICANTS: dict[str, Lubricant] = {
    lube.name: lube
    for lube in [
        Lubricant(name=DRY, viscosity_40C_cSt=1.0, viscosity_100C_cSt=1.0,
                  density_g_cm3=1.0, alpha_1_per_GPa=1.0, boundary_mu=0.12,
                  max_temp_C=1000.0),
        Lubricant(name="Grease NLGI 2 (lithium/mineral)", viscosity_40C_cSt=110.0,
                  viscosity_100C_cSt=11.0, density_g_cm3=0.89, alpha_1_per_GPa=22.0,
                  boundary_mu=0.11, max_temp_C=120.0),
        Lubricant(name="Grease NLGI 2 (PAO synthetic)", viscosity_40C_cSt=100.0,
                  viscosity_100C_cSt=14.0, density_g_cm3=0.85, alpha_1_per_GPa=15.0,
                  boundary_mu=0.10, max_temp_C=150.0),
        Lubricant(name="Grease NLGI 2 (EP, moly)", viscosity_40C_cSt=180.0,
                  viscosity_100C_cSt=15.0, density_g_cm3=0.90, alpha_1_per_GPa=23.0,
                  boundary_mu=0.06, max_temp_C=130.0),
        Lubricant(name="Oil ISO VG 32", viscosity_40C_cSt=32.0, viscosity_100C_cSt=5.4,
                  density_g_cm3=0.86, alpha_1_per_GPa=20.0, boundary_mu=0.11,
                  max_temp_C=100.0),
        Lubricant(name="Oil ISO VG 100", viscosity_40C_cSt=100.0, viscosity_100C_cSt=11.0,
                  density_g_cm3=0.88, alpha_1_per_GPa=22.0, boundary_mu=0.11,
                  max_temp_C=100.0),
        Lubricant(name="Oil ISO VG 220 (gear)", viscosity_40C_cSt=220.0,
                  viscosity_100C_cSt=19.0, density_g_cm3=0.89, alpha_1_per_GPa=24.0,
                  boundary_mu=0.08, max_temp_C=110.0),
        Lubricant(name="PTFE dry film", viscosity_40C_cSt=1.0, viscosity_100C_cSt=1.0,
                  density_g_cm3=2.2, alpha_1_per_GPa=1.0, boundary_mu=0.05,
                  max_temp_C=250.0),
    ]
}

#: Surfaces that never build a film whatever is put on them, by name.  Kept apart
#: from :data:`DRY` because "there is no oil" and "there is a solid film and no
#: oil" are different answers to the same contact.
_NO_FILM = {DRY, "PTFE dry film"}


#: What a bearing field says when the sizing study is to choose the part.
#: A sentinel rather than an empty string or ``None``: it is a value the user
#: picks from a list beside forty real designations, and "auto" is what that
#: entry has to be called for the list to make sense.
AUTOMATIC = "auto"

#: How far the input shaft stands out past the disc stack at each end, mm.
#: A modelling choice rather than a design input, but four modules were each
#: carrying their own copy of it with a comment pointing at a fifth, and the
#: bearings that sit on that overhang made the drift a fit question rather than
#: a mass one.
SHAFT_OVERHANG = 12.0

#: Axial gap between the output carrier's inner face and the first disc, mm.
#: Small and deliberate: a carrier face flush with the disc would put two
#: surfaces at the same height, which the software renderer cannot arbitrate.
CARRIER_DROP = 1.0


class OffsetMode(str, Enum):
    """How manufacturing clearance is introduced into the theoretical profile.

    Both levers have to *shrink* the disc to open a gap.  The disc reaches
    ``R - Rr + E`` from its own centre, so growing the generating roller and
    shrinking the generating pin circle each pull the profile inward; doing one
    of each is what ``BOTH`` means.  Get a sign wrong and the "clearance" is an
    interference instead.
    """

    EQUIDISTANT = "equidistant"   # grow the roller radius -> uniform normal offset
    PIN_CIRCLE = "pin_circle"     # shrink the pin circle radius -> radial shift
    BOTH = "both"                 # split evenly between the two


class HousingShape(str, Enum):
    """The outline of the housing and its two end plates.

    Round is the shape the drive has always had: a barrel, its tie bolts on a
    circle down the middle of its wall.  Square is the outline a gearhead for a
    stepper has - flat sides to bolt to or stand on - and its corners are where
    the metal is, so the tie bolts go there: four of them, each with a wall's
    worth of metal round it instead of half of what is left either side of a
    bolt on the circle.
    """

    ROUND = "round"
    SQUARE = "square"


class OutputMember(str, Enum):
    """Which member the load is taken from - and so which one is bolted down.

    A cycloidal drive is a three-shaft machine, not a two-shaft one.  The crank
    is always the input, but the *other two* - the ring the pins sit in and the
    carrier the pins through the disc holes sit on - are interchangeable: ground
    either and the remaining one is the output.  Both are built and sold.

    The choice is not a labelling exercise.  It changes the reduction, because
    the two members do not turn at the same rate; it changes the direction,
    because only one of them turns against the crank; and it changes the drive,
    because the motor has to bolt to whichever member is standing still and the
    load to whichever one is not.  See :attr:`GearSpec.ratio` and the rate
    properties beside it for the first two, and the end plates for the third.
    """

    #: Ring bolted down, output off the carrier: the reduction is the lobe
    #: count and the output turns backwards.  The classic industrial layout.
    CARRIER = "output pin carrier"
    #: Carrier bolted down, output off the ring housing: the reduction is the
    #: *pin* count, one more, and the output turns with the input.  What most
    #: printed micro drives do, because the turning part is then the outside of
    #: the gearbox and a pulley or a wheel goes straight on it.
    RING = "ring housing"


def d_section_area(r: float, flat: float) -> float:
    """A circle of radius ``r`` with a flat ``flat`` deep across it, mm^2."""
    if flat <= 0.0:
        return math.pi * r * r
    a = r - flat
    segment = r * r * math.acos(a / r) - a * math.sqrt(max(r * r - a * a, 0.0))
    return math.pi * r * r - segment


class Process(str, Enum):
    FDM = "FDM 3D print"
    SLA = "SLA/resin print"
    SLS = "SLS print"
    CNC = "CNC machined"
    EDM = "Wire EDM / ground"


#: (profile clearance per side mm, hole clearance on diameter mm) design guides.
#: Materials a printer lays down, against the spec's own names for them.  A
#: part in one of these is printed; the rest are made some other way.
PRINT_MATERIALS = frozenset({"PLA", "PETG", "ABS", "PA12 (SLS)",
                             "Tough Resin (SLA)"})

#: The 45 degree break round the bed face of a printed part, by process.
#:
#: FDM squashes its first layer into the bed so that it sticks, and the
#: squashed layer spreads past the part's outline - the elephant's foot.  On a
#: disc that is a lip standing proud of the flank at exactly the place it has to
#: clear the pins by a fifth of a millimetre, and on a bore it is a ring the
#: bearing will not start over.  Breaking the bed edge back by about a layer
#: and a half gives the spread somewhere to go.  The other processes do not
#: squash a first layer, so they get none.
FIRST_LAYER_CHAMFER: dict[Process, float] = {Process.FDM: 0.3}

PROCESS_CLEARANCE: dict[Process, tuple[float, float]] = {
    Process.FDM: (0.22, 0.30),
    Process.SLA: (0.09, 0.12),
    Process.SLS: (0.15, 0.20),
    Process.CNC: (0.03, 0.03),
    Process.EDM: (0.012, 0.015),
}

#: How far a stepper's square body has its corners cut back, as a share of its
#: width - what its section is drawn with, and the base's pad under it.
MOTOR_CORNER = 0.11

#: Air between the carrier's rim - or the end cap's - and the ring pins it
#: turns inside, mm.
CARRIER_RIM_GAP = 0.5

#: The boss's length past the output end plate off the carrier, where a
#: coupling or a clamp hub grips it, mm.
COUPLING_GRIP = 8.0

#: Air between the turning housing's underside and the base that does not on
#: a ring-output drive, mm.  The screws that hold that plate on are let into
#: it, so nothing stands in the gap.
BASE_RUNNING_GAP = 1.0

#: The closed end of a ring-output drive's housing, over its top bearing, mm:
#: the face the driven machine bolts to.
CUP_CEILING = 3.0

#: Air between the end cap's boss - and the shaft's end in it - and that
#: ceiling, which turns over them, mm.
CAP_CLEARANCE = 1.0

#: How much deeper than its dowel a blind dowel hole in a ring drive's
#: housing is, mm, so the dowel bottoms on nothing.
DOWEL_SUMP = 0.5

#: Metal left between a counterbore in the turning plate and the outside of
#: the housing, mm, for the screw size that plate's screws are held to.
COUNTERBORE_LAND = 0.75

#: Wall a heat-set insert needs round its hole in a printed part, mm: the
#: plastic it melts into is what holds it, and a thinner wall bursts.
INSERT_WALL = 1.5

#: How much deeper than an insert its blind hole is, mm, for the plastic it
#: displaces going in.
INSERT_SUMP = 0.5

#: What a tapped tie screw's tip stays under the face it is threaded towards,
#: mm: the face is what the driven machine bolts to, so nothing may reach it.
TAPPED_COVER = 0.5

#: True-position tolerance a process typically holds on a bolt circle of this
#: size, as the diameter of the tolerance zone - the way a drawing states it.
#:
#: Deliberately *not* applied by :meth:`GearSpec.apply_process_defaults`, unlike
#: the clearances.  A clearance is a dimension you choose and the model has
#: always had one; a position tolerance is a claim about what your machine
#: actually holds, and defaulting it to a guess would quietly derate every
#: design in the app on the strength of that guess.  It is a suggestion, offered
#: by the ``PIN_POSITION`` check, and it stays a suggestion until you enter it.
PROCESS_POSITION_TOLERANCE: dict[Process, float] = {
    Process.FDM: 0.30,
    Process.SLA: 0.12,
    Process.SLS: 0.25,
    Process.CNC: 0.05,
    Process.EDM: 0.02,
}

#: RMS surface roughness ``Rq`` in micrometres, by process.  Typical as-made
#: values for the *working* faces, and the number that decides whether a film
#: separates the surfaces or merely floats between the peaks: what matters is
#: film thickness measured against roughness, so a rough surface needs a thick
#: film to reach the same regime.
#:
#: This is where a printed drive loses.  A layered flank is not a slightly worse
#: ground one - it is two orders of magnitude rougher, and no lubricant sold will
#: build a film that clears it.  That is not a fixable defect, it is what the
#: process is, and the model says so rather than reporting a regime it cannot
#: reach.  Unlike the position tolerance this *is* defaulted from the process,
#: because a surface finish is not a claim about your machine: every process here
#: has one whether it is stated or not, and the alternative to a typical figure
#: is no lubrication answer at all.
PROCESS_ROUGHNESS_UM: dict[Process, float] = {
    Process.FDM: 15.0,
    Process.SLA: 3.0,
    Process.SLS: 12.0,
    Process.CNC: 1.6,
    Process.EDM: 0.4,
}


class GearSpec(BaseModel):
    """A complete, self-consistent cycloidal drive definition."""

    model_config = {"validate_assignment": True}

    # ---- core cycloid geometry ------------------------------------------------
    pin_circle_radius: float = Field(50.0, gt=0, description="R - ring pin bolt circle radius")
    pin_radius: float = Field(4.0, gt=0, description="Rr - ring pin radius")
    eccentricity: float = Field(1.5, gt=0, description="E - crank offset")
    lobes: int = Field(11, ge=3, le=200, description="N - lobes on the disc; ring pins = N+1")

    # ---- disc -----------------------------------------------------------------
    disc_thickness: float = Field(8.0, gt=0)
    disc_count: Literal[1, 2, 3] = 2
    disc_gap: float = Field(1.0, ge=0, description="axial gap between stacked discs")
    center_bore_diameter: float = Field(22.0, gt=0, description="eccentric bearing OD")

    # ---- output mechanism -----------------------------------------------------
    #
    # Which member the load comes off is the first question here, not the last:
    # it decides the reduction and the direction before any of the dimensions
    # below are read, and it decides which end of the drive the motor bolts to.
    output_member: OutputMember = Field(
        OutputMember.CARRIER,
        description="which member turns the load - the output pin carrier with "
                    "the ring housing bolted down, or the ring housing with the "
                    "carrier bolted down",
    )
    output_pin_count: int = Field(6, ge=3, le=24)
    output_pin_diameter: float = Field(6.0, gt=0)
    output_bolt_circle_radius: float = Field(30.0, gt=0)

    # ---- housing / shaft ------------------------------------------------------
    housing_wall: float = Field(6.0, gt=0)
    housing_shape: HousingShape = Field(
        HousingShape.ROUND,
        description="round, or square with the tie bolts in its corners")
    input_shaft_diameter: float = Field(10.0, gt=0)
    eccentric_cam_diameter: float | None = Field(
        None, description="cam OD; defaults to the bore less a 4 mm bearing wall"
    )
    output_flange_thickness: float = Field(6.0, gt=0)
    end_plate_thickness: float | None = Field(
        None,
        description="the two plates that close the housing and hold the shaft "
                    "supports and the output bearing; defaults to the housing wall",
    )
    turning_plate_thickness: float | None = Field(
        None, gt=0,
        description="ring output only: the plate under the turning housing and "
                    "the housing's own closed top, which carry the main "
                    "bearings; defaults to the housing wall plus 2 mm - the end "
                    "plate thickness is then the base's, which the motor stands on",
    )
    output_hub_diameter: float | None = Field(
        None,
        description="boss on the output carrier: the main output bearing sits on "
                    "it and a shaft support sits in it; defaults to the shaft plus "
                    "20 mm, which is a bearing wall each side of both",
    )

    # ---- manufacturing --------------------------------------------------------
    process: Process = Process.FDM
    offset_mode: OffsetMode = OffsetMode.EQUIDISTANT
    profile_clearance: float = Field(0.22, ge=0, description="per-side clearance on the profile")
    hole_clearance: float = Field(0.30, ge=0, description="added to hole diameters")
    position_tolerance: float = Field(
        0.0, ge=0,
        description="true-position tolerance zone diameter on the pin holes, "
                    "ring and carrier alike; 0 models a perfectly placed ring",
    )
    dxf_chord_tolerance: float = Field(0.005, gt=0)
    stl_linear_tolerance: float = Field(0.05, gt=0)
    edge_chamfer: float = Field(
        0.5, ge=0,
        description="45 degree break on the outside edges of the made parts and "
                    "a lead-in on their bearing seats; never on the profile, the "
                    "pins or the holes they run in. 0 for none",
    )

    # ---- materials ------------------------------------------------------------
    disc_material: str = "PLA"
    pin_material: str = "Steel 1045"
    housing_material: str = Field(
        "PLA", description="ring, housing and output carrier - printed on most builds"
    )
    shaft_material: str = Field("Steel 1045", description="input/eccentric shaft")
    friction_coefficient: float = Field(
        0.12, gt=0, lt=1.0,
        description="dry sliding coefficient; used as-is when there is no "
                    "lubricant, and as the boundary value a film is compared "
                    "against when there is",
    )
    lubricant: str = Field(
        DRY,
        description="what is between the sliding surfaces; dry is the default, "
                    "so a design that says nothing about lubrication gets the "
                    "answer it always got",
    )
    surface_roughness_um: float | None = Field(
        None, gt=0,
        description="RMS roughness Rq of the sliding faces; defaults to a typical "
                    "figure for the process, because every process has one",
    )
    ring_pins_integral: bool = Field(
        False,
        description="ring pins formed with the housing instead of fitted as "
                    "separate dowels - the printed-drive case",
    )
    ring_pins_are_rollers: bool = Field(
        False, description="ring pins free to rotate (needle rollers) instead of fixed dowels"
    )
    output_pins_are_rollers: bool = Field(
        False, description="output pins carry rotating bushings/rollers"
    )

    # ---- which bearings this drive carries ------------------------------------
    #
    # Three of the five load paths can legitimately be built without a bearing of
    # their own, and plenty of drives are.  These are design decisions and not
    # display options: switching one off changes the geometry, the losses, the
    # bill of materials and what the drive asks of whatever it is bolted to.
    cam_bearing_fitted: bool = Field(
        True,
        description="needle bearing between the cam and the disc bore; off means "
                    "the bore runs directly on the cam, which is a plain journal "
                    "at nearly full input speed",
    )
    shaft_bearings_fitted: bool = Field(
        True,
        description="the drive carries its own input shaft; off means it hangs on "
                    "the driving motor's bearings, which then take the crank "
                    "reaction",
    )
    output_bearing_fitted: bool = Field(
        True,
        description="the drive carries its own output flange; off means the driven "
                    "machine locates it",
    )

    # ---- which bearing goes in each seat --------------------------------------
    #
    # ``AUTOMATIC`` leaves it to the sizing study, which takes the smallest
    # catalogue part that fits the seat and lasts the required life.  Name one
    # instead when the seat is not the only thing deciding: a bearing you already
    # have, one your supplier stocks, or simply a bigger one than the smallest
    # that will do.  A named part is *checked* against its seat rather than
    # quietly replaced - being told a bearing does not fit is the point of
    # asking for it by name.
    #
    # Deliberately plain strings with no validator.  A designation this build
    # does not know should cost a warning on one line of the schedule, not a
    # design file that will not load - which is what a validator would make of
    # opening tomorrow's saved design in today's application.
    cam_bearing: str = Field(AUTOMATIC, description="eccentric cam bearing")
    shaft_bearing: str = Field(AUTOMATIC, description="input shaft supports")
    output_bearing: str = Field(AUTOMATIC, description="main output bearing")
    ring_pin_roller: str = Field(AUTOMATIC, description="ring pin needle rollers")
    output_pin_roller: str = Field(AUTOMATIC, description="output pin rollers")

    # ---- how it bolts to the world --------------------------------------------
    #
    # Both ends, because a gearbox that cannot be attached to anything at either
    # end is a model of a gearbox.  The input face is a motor frame off the
    # table; the output face is whatever the driven machine wants, so it is
    # stated as a pattern rather than chosen from a list.
    motor_frame: str = Field(
        NO_MOTOR,
        description="motor bolted to the input end plate; 'None' for a plain "
                    "plate driven through a coupling",
    )
    motor_drives_the_shaft: bool = Field(
        True,
        description="the input shaft is bored for the motor's and slides onto "
                    "it; off means a coupling between them, the motor on a "
                    "mount over it",
    )
    # The output end of this topology is a boss on the axis, not a face with a
    # bolt circle on it: what goes on there is a coupling, a pulley or a clamp
    # hub.  So what it needs is somewhere to grip, which it did not have - the
    # boss came out flush with the plate.
    output_boss_protrusion: float | None = Field(
        None, ge=0,
        description="how far the output boss stands past the end plate: grip "
                    "for a coupling off the carrier, the gap to the base off the "
                    "ring; None is automatic, see boss_protrusion",
    )
    housing_bolt_count: int = Field(
        6, ge=0, le=24,
        description="tie bolts through both end plates into the barrel; 0 means "
                    "the plates are held on by something this app is not drawing",
    )
    housing_bolt_diameter: float = Field(4.5, gt=0)
    heat_set_inserts: bool = Field(
        False,
        description="where a screw threads into a printed part - a ring drive's "
                    "turning plate and its base - it takes a heat-set insert "
                    "rather than a thread cut in the plastic",
    )
    plate_dowels: bool = Field(
        True,
        description="two dowels locating each end plate on the barrel; off "
                    "leaves only the tie bolts, through clearance, to do it",
    )
    # A ring-output drive turns its housing, and a barrel has nowhere to grip:
    # the boss that is the whole output interface of the carrier-output layout
    # belongs to the *grounded* member here, so the load needs a face of its
    # own.  Only cut when the ring is the output, and half a pitch off the tie
    # bolts so the two patterns in that one plate miss each other.
    output_bolt_count: int = Field(
        6, ge=0, le=24,
        description="bolts a driven machine attaches to on the turning housing's "
                    "end plate; ring output only, and 0 draws none",
    )
    output_bolt_diameter: float = Field(4.5, gt=0)

    bearing_min_life_hours: float = Field(
        5000.0, gt=0,
        description="L10 life a bearing has to reach before the sizing study will "
                    "take it; also what the short-life warning is measured against",
    )

    # ---- duty -----------------------------------------------------------------
    input_rpm: float = Field(1000.0, gt=0)
    output_torque_Nm: float = Field(5.0, gt=0)
    ambient_temp_C: float = Field(20.0, gt=-273.0, description="air around the housing")

    #: What the drive actually does over time, if it has been said.
    #:
    #: Empty is the default and means the rated point above is the whole duty,
    #: which is what every design has meant until now.  Stating a cycle does not
    #: move the rated point or anything derived from it: it adds the four
    #: aggregates a spectrum can answer and a single point cannot - see
    #: :mod:`cycloidgen.core.duty`.
    duty_cycle: DutyCycle = Field(default_factory=DutyCycle)

    # ---- what turns it --------------------------------------------------------
    #
    # Eight numbers off a datasheet, flat rather than nested, because the panel
    # writes spec fields straight and a dotted path would be a second way to
    # address one.  They are assembled into a :class:`MotorCurve` by the
    # property below and read from there by everything else, so the curve has no
    # state of its own to fall out of step.
    #
    # Three of them are shared between the two kinds and that sharing is
    # physical rather than a squeeze: both motors run off the same bus, both
    # datasheets print a winding resistance, and both have a current they will
    # hold all day - the driver setting for a stepper, the continuous rating for
    # a DC motor.  What is not shared is what the current *means*, which is why
    # a stepper's curve is already its continuous curve and a DC motor's is not.
    motor_kind: MotorKind = Field(
        MotorKind.NONE,
        description="which torque-speed curve the input follows; 'none' asks "
                    "nothing of the motor, which is what the app always did",
    )
    motor_supply_V: float = Field(24.0, gt=0, description="bus volts")
    motor_resistance_ohm: float = Field(
        1.5, gt=0,
        description="per phase on a stepper, terminal to terminal on a DC motor",
    )
    motor_rated_current_A: float = Field(
        1.5, gt=0,
        description="the driver setting on a stepper, the continuous rating on "
                    "a DC motor",
    )
    motor_holding_torque_Nm: float = Field(
        0.45, gt=0, description="stepper: both phases at rated current")
    motor_inductance_mH: float = Field(3.0, gt=0, description="stepper: per phase")
    motor_steps_per_rev: int = Field(
        200, ge=4, le=10000,
        description="stepper: full steps per revolution; 200 is 1.8 degrees",
    )
    motor_kv_rpm_per_V: float = Field(
        100.0, gt=0, description="DC: no-load speed per volt")

    # ---------------------------------------------------------------- derived --
    @computed_field  # type: ignore[prop-decorator]
    @property
    def pin_count(self) -> int:
        """Ring pins.  One more than the lobe count for a single-tooth-difference drive."""
        return self.lobes + 1

    # ---- how fast each body turns ---------------------------------------------
    #
    # All of the geometry in this app is modelled in the frame the ring housing
    # sits still in, because that is the frame the profile is generated in and
    # the frame the meshing was verified in.  Grounding the *carrier* instead
    # does not change a single relative motion in the drive - it adds one rigid
    # rotation to every part at once, and that is exactly what
    # :attr:`frame_spin` is.
    #
    # Everything below is per radian of crank angle, so the frame term appears
    # once and cancels wherever two bodies are compared with each other.  That
    # cancellation is the point: a relative speed is a fact about the mechanism
    # and must not depend on which member somebody bolted down, while an
    # absolute one must.
    @property
    def frame_spin(self) -> float:
        """Rigid rotation added to every part, per radian of crank angle.

        Zero with the ring grounded, which is how the model is built.  With the
        carrier grounded it is ``-1/N``, precisely cancelling the carrier's own
        rotation and setting the whole drive turning the other way underneath.
        """
        return 0.0 if self.output_member is OutputMember.CARRIER else -1.0 / self.lobes

    @property
    def shaft_spin(self) -> float:
        """The crank, per radian of crank angle.

        Negative: the disc centre runs to ``(E cos phi, -E sin phi)``, which is
        a clockwise walk, so the cam carrying it turns by ``-phi``.
        """
        return -1.0 + self.frame_spin

    @property
    def disc_spin(self) -> float:
        """The disc, per radian of crank angle.  ``+phi/N`` in the built frame."""
        return 1.0 / self.lobes + self.frame_spin

    @property
    def carrier_spin(self) -> float:
        """The output carrier: the disc's own rotation, and exactly it.

        The pins in the disc's holes are a coupling, not a gear - they pass the
        disc's rotation on at one to one and take out the orbit.  So this is
        :attr:`disc_spin` rather than a second law that would have to be kept
        in step with it.
        """
        return self.disc_spin

    @property
    def ring_spin(self) -> float:
        """The ring housing and the two plates bolted to it."""
        return self.frame_spin

    @property
    def output_spin(self) -> float:
        """Whichever of the two the load is taken from."""
        return (self.carrier_spin if self.output_member is OutputMember.CARRIER
                else self.ring_spin)

    @property
    def crank_rate(self) -> float:
        """Crank angle per unit *input* rotation.

        One with the ring grounded, where the crank is the input and nothing
        else moves under it.  With the carrier grounded the crank is already
        turning at ``(N+1)/N`` of the crank angle, so a given input speed buys
        proportionally less of it - and every sliding speed in the drive is
        stated per unit crank angle, so they all pass through here.
        """
        return 1.0 / abs(self.shaft_spin)

    @property
    def crank_relative_rate(self) -> float:
        """Disc against crank, per unit input speed.

        Two contacts turn out to be the same number and it is worth saying why.
        The cam bearing separates the disc from the crank, so it turns at their
        difference.  The output pins rub because the disc's centre walks a
        circle of radius E *in the carrier's frame*, and the carrier turns with
        the disc - so that walk is at the difference between the disc and the
        crank as well.  One rate, two contacts.

        It is ``1 + 1/N`` per crank radian and not ``1 - 1/N``: the disc turns
        the *opposite* way from the crank, so the two rates add.  Getting that
        backwards understates the fastest contact in the machine, and it
        understates it worst on the low ratios where it matters most.
        """
        return abs(self.disc_spin - self.shaft_spin) * self.crank_rate

    @property
    def disc_speed_ratio(self) -> float:
        """How fast the disc turns in the ground frame, per unit input speed.

        ``1/N`` with the ring grounded.  Zero with the carrier grounded, where
        the disc does not rotate at all - it only orbits - which is why a
        ring-output drive reflects less of the disc's inertia back to the motor.
        """
        return abs(self.disc_spin) * self.crank_rate

    @computed_field  # type: ignore[prop-decorator]
    @property
    def ratio(self) -> int:
        """Reduction, from whichever member the output is taken off.

        With the **ring** grounded it is the lobe count, verified by meshing
        simulation: i == lobe count == pin_count - 1.

        With the **carrier** grounded it is the *pin* count, one more.  Not a
        coincidence and not an approximation: the two members are one tooth
        apart, so grounding the other one moves the reduction by exactly one.
        The extra tooth of reduction is free - the same parts, bolted down at
        the other end.
        """
        return self.lobes if self.output_member is OutputMember.CARRIER else self.pin_count

    @computed_field  # type: ignore[prop-decorator]
    @property
    def output_reverses(self) -> bool:
        """Whether the output turns against the input.

        It does off the carrier and does not off the ring, which is the other
        half of what grounding the other member buys you.  Derived from the two
        spins rather than stated, so it cannot disagree with the picture the 3D
        view draws from the same numbers.
        """
        return (self.output_spin < 0.0) != (self.shaft_spin < 0.0)

    @computed_field  # type: ignore[prop-decorator]
    @property
    def K1(self) -> float:
        """Trochoid shortening coefficient.  Must stay below 1."""
        return self.eccentricity * self.pin_count / self.pin_circle_radius

    @computed_field  # type: ignore[prop-decorator]
    @property
    def output_hole_diameter(self) -> float:
        """Verified exactly: the disc translates on a circle of radius E in the
        output carrier frame, so the hole must be the pin plus 2E."""
        return self.output_pin_diameter + 2 * self.eccentricity + self.hole_clearance

    @computed_field  # type: ignore[prop-decorator]
    @property
    def disc_outer_radius(self) -> float:
        return self.pin_circle_radius - self.pin_radius + self.eccentricity

    @computed_field  # type: ignore[prop-decorator]
    @property
    def housing_outer_radius(self) -> float:
        return self.pin_circle_radius + self.pin_radius + self.housing_wall

    @computed_field  # type: ignore[prop-decorator]
    @property
    def stack_height(self) -> float:
        n = self.disc_count
        return n * self.disc_thickness + (n - 1) * self.disc_gap

    @property
    def barrel_bottom(self) -> float:
        """Where the ring housing starts, with the first disc face at zero.

        Negative, and it has to be.  The output carrier hangs below the disc
        stack - a drop, then its own thickness - and the output end plate bolts
        to the barrel underneath *that*.  A barrel that stops at the discs
        leaves the carrier standing in the open with a gap between the housing
        and the plate it is supposed to be bolted to, which is a gearbox with a
        slot cut round it.

        The barrel was sized to the disc stack when the disc stack was all there
        was to enclose.  The end plates gave it something to reach.
        """
        return -(CARRIER_DROP + self.output_flange_thickness)

    @property
    def barrel_top(self) -> float:
        """Where the ring housing stops.

        The top of the disc stack, unless there is a frame - and then the same
        allowance the carrier gets at the other end, because the end cap is the
        carrier's mirror image and has to fit inside the barrel exactly as the
        carrier does.  A barrel that stopped at the discs would leave the cap
        standing proud of the housing it is meant to be inside, which is the
        same fault the carrier had before the barrel was lengthened for it.
        """
        if not self.ground_frame_fitted:
            return self.stack_height
        return self.stack_height + CARRIER_DROP + self.output_flange_thickness

    @property
    def barrel_height(self) -> float:
        """Length of the ring housing: the discs, and whatever is inside it at
        each end."""
        return self.barrel_top - self.barrel_bottom

    @computed_field  # type: ignore[prop-decorator]
    @property
    def output_rpm(self) -> float:
        return self.input_rpm / self.ratio

    # ---- effective (clearance-corrected) cutting geometry ---------------------
    @property
    def effective_R(self) -> float:
        """Pin circle radius the profile is *generated* with.

        Shrinking it moves the whole profile inward, which is what opens the
        radial clearance.  The pins themselves stay at
        :attr:`pin_circle_radius`; only the cutting geometry moves.
        """
        c = self.profile_clearance
        if self.offset_mode is OffsetMode.PIN_CIRCLE:
            return self.pin_circle_radius - c
        if self.offset_mode is OffsetMode.BOTH:
            return self.pin_circle_radius - c / 2
        return self.pin_circle_radius

    @property
    def effective_Rr(self) -> float:
        c = self.profile_clearance
        if self.offset_mode is OffsetMode.EQUIDISTANT:
            return self.pin_radius + c
        if self.offset_mode is OffsetMode.BOTH:
            return self.pin_radius + c / 2
        return self.pin_radius

    @property
    def cam_diameter(self) -> float:
        """Eccentric cam OD.  Leaves room for a 4 mm walled needle bearing by default.

        With no bearing fitted there is no wall to leave: the disc bore runs on
        the cam itself, so the cam is the nominal bore and ``hole_clearance`` -
        which the bore is opened by - becomes the running fit.  Keeping it 8 mm
        under would be a disc flopping about on a shaft.
        """
        if self.eccentric_cam_diameter is not None:
            return self.eccentric_cam_diameter
        if not self.cam_bearing_fitted:
            return max(self.center_bore_diameter, self.input_shaft_diameter + 2.0)
        return max(self.center_bore_diameter - 8.0, self.input_shaft_diameter + 2.0)

    @property
    def plate_thickness(self) -> float:
        """The two end plates that close the housing.

        The housing wall plus a couple of millimetres, because these are bearing
        housings and not covers: whatever goes in one has to fit *inside* it, and
        the thin-section balls that suit this size run 5 to 7 mm wide.  A plate
        at the bare wall could not hold the bearing the output seat asks for.
        """
        if self.end_plate_thickness is not None:
            return self.end_plate_thickness
        return self.housing_wall + 2.0

    @property
    def housing_plate_thickness(self) -> float:
        """The plates that close the housing, mm.

        The end plates off the carrier.  Off the ring, the plate under the
        turning housing and the housing's own closed top, each of which carries
        a main bearing and nothing else - while :attr:`plate_thickness` is then
        the base the motor stands on, which is what a 12 mm plate on a motor
        preset was sized for.  So these take :attr:`turning_plate_thickness`,
        or a housing wall and 2 mm: a 12 mm plate under a 7 mm bearing was a
        drive 8 mm longer than it needed to be.
        """
        if not self.ground_frame_fitted:
            return self.plate_thickness
        if self.turning_plate_thickness is not None:
            return self.turning_plate_thickness
        return self.housing_wall + 2.0

    @property
    def shaft_overhang(self) -> float:
        """How far the input shaft stands past the disc stack at each end.

        It has to reach through the carrier's boss, because a shaft support sits
        in there and a bearing on a shaft that stops short of it is not a
        bearing.  The fixed 12 mm this used to be predates the boss existing.
        """
        return max(SHAFT_OVERHANG,
                   CARRIER_DROP + self.output_flange_thickness
                   + self.housing_plate_thickness + 3.0)

    @property
    def carrier_radius(self) -> float:
        """The carrier plate's radius, and the end cap's: a pin's diameter of
        metal past the output pin circle, but never out among the ring pins.

        The plate sits inside the barrel, a drop below the first disc, and the
        ring pins run the barrel's whole length past it - so a plate wider than
        their inner edge turns into them.  A large output pin on a small pin
        circle did exactly that: the 29:1 NEMA 23's 12 mm pins put its carrier
        half a millimetre into the ring pins.  Held to their inner edge, less
        :data:`CARRIER_RIM_GAP`; ``THIN_CARRIER_RIM`` says when what that
        leaves round an output pin is too little.
        """
        return min(self.output_bolt_circle_radius + self.output_pin_diameter,
                   self.pin_circle_radius - self.pin_radius - CARRIER_RIM_GAP)

    @property
    def hub_diameter(self) -> float:
        """Output carrier boss: what the main output bearing sits *on*.

        Twenty over the shaft by default, which is a bearing wall each side of
        both bearings it separates - a support inside it on the shaft, the output
        bearing outside it in the end plate.  On the 10 mm default shaft that is
        30 mm, and 30 mm is a catalogue bore, so the two land on each other
        exactly rather than nearly.
        """
        if self.output_hub_diameter is not None:
            return self.output_hub_diameter
        return max(self.input_shaft_diameter + 20.0,
                   self.input_shaft_diameter + 2.0)

    @property
    def hub_bore(self) -> float:
        """Inside of the boss: the seat for the outboard shaft support.

        The hub less a 4 mm wall each side, which is the same allowance the cam
        makes for the bearing inside the disc bore.
        """
        return max(self.hub_diameter - 8.0, self.input_shaft_diameter + 1.0)

    @property
    def boss_bottom(self) -> float:
        """The far end of the carrier's boss.

        The output end plate's outer face, then however far the boss was asked
        to stand past it.  What is *at* that end differs: a coupling grips it
        when the carrier is the output, and the base the drive is bolted down
        by is on it when the carrier is the ground.
        """
        return (self.barrel_bottom - self.housing_plate_thickness
                - self.boss_protrusion)

    @property
    def boss_protrusion(self) -> float:
        """How far the boss stands past the output end plate, mm.

        What is stated, if anything is.  Otherwise off the carrier
        :data:`COUPLING_GRIP` of it for a coupling to grip, and off the ring
        only :data:`BASE_RUNNING_GAP`, the air the turning plate needs over the
        base it turns against.  That gap used to be the coupling's grip as well
        - eight millimetres of open neck under a drive that had nothing there
        to grip - and then the tie screws' heads and a millimetre, turning in
        it; the screws that hold that plate on are let into it now.
        """
        if self.output_boss_protrusion is not None:
            return self.output_boss_protrusion
        if not self.ground_frame_fitted:
            return COUPLING_GRIP
        return BASE_RUNNING_GAP

    @property
    def ground_frame_fitted(self) -> bool:
        """Whether the drive carries a frame the housing turns *inside*.

        Only when the ring is the output member, and then it is not an option
        but the shape of the machine.  A motor cannot bolt to a plate that
        turns, and with the ring as the output every plate on the housing does -
        so the grounded member has to become something you can bolt a motor to
        and hang a gearbox off, at both ends.

        What that frame is: the carrier plate, the output pins standing on it,
        an **end cap** the far end of those pins lands in, a boss on each of the
        two plates, and a base on the outside of the lower one.  It is one rigid
        body and the pins are what holds it together - which is the arrangement
        every printed micro drive of this kind uses, and it buys two things that
        are not cosmetic.  The housing is carried at *both* ends instead of
        hanging off one bearing, so it can take a moment.  And the output pins
        are beams rather than cantilevers, which is a factor of four off their
        bending stress.
        """
        return self.output_member is OutputMember.RING

    @property
    def end_cap_bottom(self) -> float:
        """Inner face of the end cap: a drop above the last disc.

        The same drop the carrier keeps below the first one, and for the same
        reason - two surfaces at one height is a fight the renderer cannot win -
        so the two plates are mirror images about the stack.
        """
        return self.stack_height + CARRIER_DROP

    @property
    def end_cap_top(self) -> float:
        """Outer face of the end cap, which is where the barrel ends."""
        return self.end_cap_bottom + self.output_flange_thickness

    @property
    def cap_boss_top(self) -> float:
        """The far end of the end cap's boss.

        Inside the turning housing's top, :data:`CAP_CLEARANCE` short of the
        ceiling that closes it.  It does not stand proud the way the carrier's
        does: nothing grips this one, it is inside the frame, and its whole job
        is to hold the output bearing on its outside and the shaft support in
        its bore.  It used to come up flush with the face the load bolts to -
        a stationary disc in the middle of the one face that turns.
        """
        return self.barrel_top + self.housing_plate_thickness - CAP_CLEARANCE

    @property
    def cup_top(self) -> float:
        """The housing's closed face on a ring drive, which the driven machine
        bolts to: the top that holds the second main bearing, then
        :data:`CUP_CEILING` over it.  The barrel's top on a carrier drive,
        where an end plate closes the housing instead.

        One part with the barrel, which is the point: the plate that used to
        bolt on there turned with the barrel anyway, and took a joint, six
        screws and two dowels to do it.
        """
        if not self.ground_frame_fitted:
            return self.barrel_top
        return self.barrel_top + self.housing_plate_thickness + CUP_CEILING

    @property
    def base_plate_bottom(self) -> float:
        """Outer face of the base: the face the motor bolts to."""
        return self.boss_bottom - self.plate_thickness

    # ---- the base: a flange to bolt down by, and a pad the motor stands on ---
    #
    # A ring-output drive stands on its carrier's base.  It was a plate the
    # housing's size and an end plate's thickness, which is what the motor's
    # register, its screws' seats and the shaft support's lip need - under the
    # motor.  Out past the motor it needs only to be bolted down, and had
    # nothing to bolt it down by.  So it is two things now: a pad the motor's
    # size and the full thickness, and a flange round it the housing's size
    # and only as thick as the screws that hold it down need, tapped for them
    # from the machine's side.  Through a coupling the motor stands on a mount
    # instead, and the base stays whole.
    @property
    def base_pad(self) -> tuple[str, float]:
        """The pad under the motor: ``("square", width)`` for a stepper's body,
        its corners cut back :data:`MOTOR_CORNER` of it, or ``("round",
        diameter)`` for a drum - and the boss's own, with no motor on it."""
        if self.has_motor_face and not self.coupled:
            frame = self.motor_face
            return ("round" if frame.round_body else "square"), frame.body_width
        return "round", self.hub_diameter

    def base_pad_outline(self, segments: int = 72) -> list[tuple[float, float]]:
        """The pad as a loop: the cut-cornered square, or a circle of
        ``segments``."""
        kind, size = self.base_pad
        if kind == "square":
            h, c = size / 2.0, MOTOR_CORNER * size
            return [(h, -h + c), (h, h - c), (h - c, h), (-h + c, h),
                    (-h, h - c), (-h, -h + c), (-h + c, -h), (h - c, -h)]
        r = size / 2.0
        return [(r * math.cos(2.0 * math.pi * k / segments),
                 r * math.sin(2.0 * math.pi * k / segments)) for k in range(segments)]

    @property
    def base_pad_area(self) -> float:
        """The pad's area, mm^2: the square less its four corners, or the
        circle."""
        kind, size = self.base_pad
        if kind == "square":
            return size * size - 2.0 * (MOTOR_CORNER * size) ** 2
        return math.pi * size * size / 4.0

    @property
    def base_flange_thickness(self) -> float:
        """The flange round the pad, mm: a housing wall, or what the screws it
        is held down by need to thread into from below with
        :data:`TAPPED_COVER` over their tips, whichever is more - and never
        more than the pad.  The whole plate where there is no flange."""
        if not self.base_has_flange:
            return self.plate_thickness
        insert = self.base_insert
        hole = (insert.length + INSERT_SUMP if insert is not None
                else self._thread_engagement(None, self.bolt_size_asked))
        return min(self.plate_thickness, max(self.housing_wall, hole + TAPPED_COVER))

    @property
    def base_has_flange(self) -> bool:
        """Whether the base is a pad and a flange rather than a plate: on a ring
        drive, unless the motor is on a mount or wider than the base."""
        if not self.ground_frame_fitted or self.coupled:
            return False
        loop = self.base_pad_outline()
        if self.square_housing:
            return max(max(abs(x), abs(y)) for x, y in loop) < self.housing_outer_radius
        return max(math.hypot(x, y) for x, y in loop) < self.housing_outer_radius

    @property
    def base_mount_centres(self) -> list[tuple[float, float]]:
        """Where the base is bolted down from: four tapped holes in the flange,
        each on an axis - the middle of a stepper's flat, out of its corners'
        way - halfway out from the pad to the edge.  None where that leaves no
        wall either side of the thread's drill."""
        if not self.base_has_flange or self.bolt_size_asked is None:
            return []
        pad = self.base_pad[1] / 2.0                   # the pad's reach along x
        edge = self.housing_outer_radius
        r = 0.5 * (pad + edge)
        if (edge - pad) / 2.0 - self.base_mount_hole / 2.0 < 1.5:
            return []
        return [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)]

    @property
    def base_mount_depth(self) -> float:
        """How deep the mount holes go up into the flange from below: an
        insert's length and a little for the plastic it displaces, or the
        thread a tapped one needs."""
        insert = self.base_insert
        need = (insert.length + INSERT_SUMP if insert is not None
                else self._thread_engagement(None, self.bolt_size_asked))
        return min(need, self.base_flange_thickness - TAPPED_COVER)

    @property
    def output_pins_are_supported_at_both_ends(self) -> bool:
        """Whether an output pin is a beam or a cantilever.

        A beam once there is an end cap for the far end of it to land in, which
        is the single biggest thing the frame buys: a pin loaded at mid-span
        carries ``F*L/4`` rather than the ``F*L`` of a cantilever off one plate.
        Everything that bends a pin reads this rather than assuming, because
        the assumption used to be wired in.
        """
        return self.ground_frame_fitted

    @property
    def output_face_bolt_radius(self) -> float:
        """Where a driven machine bolts to the turning housing.

        The tie bolts' own circle, because that is where the metal is: it is
        the one radius on that plate with the barrel wall behind it rather than
        the disc chamber, so a bolt lands in something.  The two patterns share
        a circle and are kept apart by angle instead - see
        :attr:`output_bolt_phase` - and when they cannot be, the
        ``OUTPUT_BOLT_CLASH`` check says so rather than the model quietly
        drawing one hole through another.  A square housing keeps the circle
        - the middle of its wall at the flats - and its tie bolts go out into
        the corners, off it.

        Since the face became the housing's closed top it is solid from the
        second main bearing's seat out to the rim, and the holes go in the
        middle of that, where they have the most round them - an insert's
        would not fit down the middle of a 6 mm wall.  Unless they are deeper
        than the top, and would go on down into the barrel: only the middle of
        its wall clears the pin pockets, so there they stay.
        """
        if not self.ground_frame_fitted:
            return self.housing_bolt_radius
        wanted = (insert_for(self.output_bolt_size) if self.heat_set_inserts
                  and self.housing_mat.E_GPa < 10.0 else None)
        if (self._face_hole_depth(wanted)
                > self.housing_plate_thickness + CUP_CEILING - CAP_CLEARANCE):
            return self.housing_bolt_radius
        return 0.5 * (self.output_bearing_seat_diameter / 2.0
                      + self.housing_outer_radius)

    @property
    def output_bolt_phase(self) -> float:
        """Half a tie-bolt pitch, so the two circles interleave.

        Zero when there are no tie bolts to miss.  With the usual six of each
        this puts an output hole exactly between every pair of tie bolts, which
        is as far from them as the circle allows.  Zero on a square housing
        too: its tie bolts are on the diagonals, so the first output bolt goes
        on the middle of a flat, as far from two corners as it can be.
        """
        import math
        if not self.housing_bolt_count or self.square_housing:
            return 0.0
        return math.pi / self.housing_bolt_count

    # ---- the face a driven machine bolts to -----------------------------------
    #
    # On a ring drive it is the housing's own closed top, and the machine's
    # screws thread into it - blind holes down into solid wall, where the plate
    # that used to be there had clearance holes with nothing under them but the
    # barrel's end.  Tapped, or with heat-set inserts in a printed one.
    @property
    def output_bolt_size(self) -> MetricSize | None:
        """The screw the output bolt hole is the clearance for: what the
        driven machine is bolted to a ring drive's face with."""
        if not self.output_bolt_count:
            return None
        return size_for_hole(self.output_bolt_diameter)

    @property
    def output_face_bolt_centres(self) -> list[tuple[float, float]]:
        """Where the driven machine's screws go into a ring drive's face."""
        if not self.ground_frame_fitted or not self.output_bolt_count:
            return []
        r, n = self.output_face_bolt_radius, self.output_bolt_count
        return [(r * math.cos(self.output_bolt_phase + 2.0 * math.pi * k / n),
                 r * math.sin(self.output_bolt_phase + 2.0 * math.pi * k / n))
                for k in range(n)]

    @property
    def output_insert(self) -> HeatSetInsert | None:
        """The insert the driven machine's screws thread into in a printed
        ring drive's face: asked for, and a wall of :data:`INSERT_WALL` round
        its hole."""
        if (not self.heat_set_inserts or self.housing_mat.E_GPa >= 10.0
                or not self.output_face_bolt_centres):
            return None
        wanted = insert_for(self.output_bolt_size)
        if wanted is None or self.face_insert_wall(wanted.hole) < INSERT_WALL:
            return None
        return wanted

    def face_insert_wall(self, hole: float) -> float:
        """The least wall round holes of ``hole`` in a ring drive's face, mm:
        to the outline, and in towards the second main bearing's seat - or,
        where they are down the middle of the wall because they go on into the
        barrel, to the pin pockets as well."""
        centres = self.output_face_bolt_centres
        if not centres:
            return math.inf
        if self.output_face_bolt_radius == self.housing_bolt_radius:
            return self._wall_round(centres, hole)
        seat = self.output_bearing_seat_diameter / 2.0
        return min(min(self._inside_outline(x, y), math.hypot(x, y) - seat)
                   for x, y in centres) - hole / 2.0

    @property
    def output_face_hole(self) -> float:
        """The hole in a ring drive's face for each of the driven machine's
        screws: the insert's, the tap drill, or with no screw named for the
        output bolt hole that hole as it is given."""
        insert = self.output_insert
        if insert is not None:
            return insert.hole
        size = self.output_bolt_size
        return size.d - size.pitch if size else self.output_bolt_diameter

    @property
    def output_face_hole_depth(self) -> float:
        """How deep those holes go down into the face, mm: the insert and its
        sump, or the thread a screw needs - one and a half diameters in metal,
        two in a polymer - and two pitches past it for the tap to run out in."""
        return self._face_hole_depth(self.output_insert)

    def _face_hole_depth(self, insert: HeatSetInsert | None) -> float:
        if insert is not None:
            return insert.length + INSERT_SUMP
        size = self.output_bolt_size
        if size is None:
            return self.housing_plate_thickness + CUP_CEILING
        polymer = self.housing_mat.E_GPa < 10.0
        return (2.0 if polymer else 1.5) * size.d + 2.0 * size.pitch

    @property
    def motor_face(self) -> MotorFrame:
        """The face the motor bolts to, as the frame standard defines it.

        Named for the face rather than for the motor, because a motor is two
        facts and this is only the first of them.  The other one is
        :attr:`motor_curve`, and an analysis result carries the answer under
        ``motor`` - so leaving this called ``motor`` would have put two
        different objects behind one word on the two most-read classes here.
        """
        return MOTOR_FRAMES[self.motor_frame]

    @property
    def has_motor_face(self) -> bool:
        return self.motor_frame != NO_MOTOR

    @property
    def motor_curve(self) -> MotorCurve:
        """What the motor can deliver, against speed.

        Assembled on every read rather than cached: it is eight attribute reads
        and a dataclass, and a cache here would be a copy of the design that can
        disagree with the design.

        Independent of :attr:`has_motor_face` on purpose.  A face is what the
        drive bolts to and a curve is what turns the crank, and the two are
        genuinely separable - a motor driving through a coupling from somewhere
        else has a curve and no face, and a plate cut for a NEMA 17 says nothing
        about which NEMA 17 goes on it.
        """
        return curve_from(self)

    @property
    def coupled(self) -> bool:
        """Whether a coupling joins the motor's shaft to the drive's: a motor
        face, and the motor not driving the shaft directly."""
        return self.has_motor_face and not self.motor_drives_the_shaft

    @property
    def plate_face_z(self) -> float:
        """The outside of the plate the motor's end is: the input end plate on
        a carrier drive, the carrier's base on a ring one."""
        if self.ground_frame_fitted:
            return self.base_plate_bottom
        return self.barrel_top + self.housing_plate_thickness

    @property
    def coupling(self) -> Coupling | None:
        """The coupling between the two shafts, or None with no coupling."""
        if not self.coupled:
            return None
        return coupling_for(self.motor_face.shaft_diameter, self.input_shaft_diameter)

    @property
    def coupling_span(self) -> tuple[float, float] | None:
        """Where the coupling is along the axis, ``(lo, hi)``: just clear of the
        plate, towards the motor."""
        c = self.coupling
        if c is None:
            return None
        start = self.plate_face_z + self.motor_side * COUPLING_GAP
        end = start + self.motor_side * c.length
        return (start, end) if start < end else (end, start)

    @property
    def motor_mount(self) -> tuple[float, float, float] | None:
        """The tube the motor stands on, through a coupling: ``(bore radius,
        outer radius, height)``, or None.

        Round the coupling with room to turn, a wall thick, and as tall as the
        motor stands off the plate less the flange it bolts to - which is the
        motor's own shaft less what of it the coupling takes, or the coupling
        and the spigot, whichever is more.
        """
        c = self.coupling
        if c is None:
            return None
        bore = c.outer / 2.0 + 1.5
        return (bore, bore + MOTOR_MOUNT_WALL,
                abs(self.motor_face_z - self.plate_face_z) - self.motor_mount_flange)

    @property
    def motor_mount_flange(self) -> float:
        """How thick the mount's flange is, mm; zero with no mount."""
        if not self.coupled:
            return 0.0
        return mount_flange(self.motor_face)

    @property
    def motor_mount_hole(self) -> float:
        """The hole through the mount's flange, diameter: the motor's spigot,
        which is what centres it, or the tube's bore where that is smaller -
        then the spigot sits in a recess on top instead.  Zero with no mount."""
        mount = self.motor_mount
        if mount is None:
            return 0.0
        return min(self.motor_face.pilot_diameter, 2.0 * mount[0])

    @property
    def window_span(self) -> tuple[float, float] | None:
        """The windows through the mount's tube, along the axis: the coupling,
        two millimetres in from each end, which is where its screws are and
        where a key has to get to them."""
        span = self.coupling_span
        if span is None:
            return None
        return span[0] + 2.0, span[1] - 2.0

    @property
    def window_half_width(self) -> float:
        """Half the width of a window across the tube, mm; zero with no mount."""
        mount = self.motor_mount
        if mount is None:
            return 0.0
        return min(0.7 * mount[0], 8.0)

    @property
    def motor_mount_volume(self) -> float:
        """The mount's own metal, mm^3: tube less its windows, flange less its
        holes and the spigot's recess.  What the mass model adds to the plate
        it stands on, off the same numbers the solid is built from."""
        if not self.coupled:
            return 0.0
        frame = self.motor_face
        bore, outer, height = self.motor_mount
        half = self.window_half_width

        def under(r: float) -> float:          # area under a circle, |x| < half
            return half * math.sqrt(r * r - half * half) + r * r * math.asin(half / r)

        w0, w1 = self.window_span
        tube = (math.pi * (outer ** 2 - bore ** 2) * height
                - 2.0 * (under(outer) - under(bore)) * (w1 - w0))
        hole = self.motor_mount_hole
        flange = (frame.body_section_area - math.pi * hole * hole / 4.0
                  - len(self.motor_bolt_centres) * math.pi
                  * frame.bolt_diameter ** 2 / 4.0) * self.motor_mount_flange
        if frame.pilot_diameter > hole + 1e-9:
            flange -= math.pi * (frame.pilot_diameter ** 2 - hole ** 2) / 4.0 \
                * frame.pilot_depth
        return tube + flange

    @property
    def direct_drive(self) -> bool:
        """Whether the motor's shaft goes straight into the cam: a motor face,
        and no coupling between them."""
        return self.has_motor_face and self.motor_drives_the_shaft

    @property
    def shaft_span(self) -> tuple[float, float]:
        """Where the input shaft runs along the axis, as built: ``(lo, hi)``.

        Past the disc stack at both ends by :attr:`shaft_overhang` - except,
        driven directly, at the motor's end.  There it stops just short of the
        motor's spigot, because the motor is flush on its face: the shaft that
        stood out of the plate for a coupling to grip was standing where the
        motor is.  On a ring drive that end is the far side of the base, so it
        reaches further rather than less.
        """
        lo, hi = -self.shaft_overhang, self.stack_height + self.shaft_overhang
        end = None
        if self.rotor_hub is not None:
            # Through the motor's hollow centre to the hub on its can.
            z0, z1 = self.rotor_hub[:2]
            end = z1 if self.motor_side > 0 else z0
        elif self.direct_drive:
            end = self.motor_face_z - self.motor_side * (
                self.motor_face.pilot_depth + MOTOR_HUB_CLEARANCE)
        elif self.coupled:
            # Into the near half of the coupling, short of its middle.
            c0, c1 = self.coupling_span
            end = (c0 + c1) / 2.0 - self.motor_side * COUPLING_GAP / 2.0
        if end is not None:
            if self.motor_side > 0:
                hi = end
            else:
                lo = end
        if self.ground_frame_fitted:
            # Its far end is in the end cap's boss, under the housing's closed
            # top: flush with the boss, not out through the face that turns.
            hi = min(hi, self.cap_boss_top)
        return lo, hi

    @property
    def motor_bore(self) -> tuple[float, float, float, float] | None:
        """The blind bore the motor's shaft goes into: ``(z from, z to,
        diameter, flat)``, or None when the motor does not drive the shaft.

        From the shaft's motor end in as far as the motor's shaft reaches, and
        half a millimetre past its tip.  A D where the motor's shaft has a D, so
        the torque goes through the flat rather than through a set screw nobody
        can reach once the motor is on; and bored by the process's own hole
        clearance, which is the fit this process makes.

        None too where the bore would be as wide as the shaft or wider: there
        is no shaft round it then, and a bore through all of a shaft's end is
        a cut no solid modeller can make.  The shaft is drawn whole, and
        ``MOTOR_SHAFT_TOO_BIG`` says why it cannot be built.
        """
        if (not self.direct_drive or self.motor_face.rotor_driven
                or self.motor_hub_wall <= 0.0):
            return None
        frame = self.motor_face
        tip = self.motor_face_z - self.motor_side * frame.shaft_length
        lo, hi = self.shaft_span
        if self.motor_side > 0:
            z0, z1 = max(lo, tip - MOTOR_HUB_CLEARANCE), hi
        else:
            z0, z1 = lo, min(hi, tip + MOTOR_HUB_CLEARANCE)
        if z1 <= z0:
            return None
        return (z0, z1, self.motor_bore_diameter, frame.shaft_flat)

    @property
    def motor_bore_diameter(self) -> float:
        """What the input shaft is bored to for the motor's, driven directly:
        the motor's shaft at the process's hole clearance.  Zero otherwise."""
        if not self.direct_drive or self.motor_face.rotor_driven:
            return 0.0
        return self.motor_face.shaft_diameter + self.hole_clearance

    @property
    def motor_engagement(self) -> float:
        """How much of the motor's shaft is inside what it drives - the input
        shaft directly, or its half of the coupling - mm."""
        if self.coupled:
            frame, side = self.motor_face, self.motor_side
            tip = self.motor_face_z - side * frame.shaft_length
            c0, c1 = self.coupling_span
            near = c1 if side > 0 else c0
            return max(min(side * (near - tip), self.coupling.length / 2.0), 0.0)
        bore = self.motor_bore
        if bore is None:
            return 0.0
        return max(bore[1] - bore[0] - MOTOR_HUB_CLEARANCE, 0.0)

    @property
    def motor_hub_wall(self) -> float:
        """What is left round the motor's shaft, driven directly, mm - below
        zero where the motor's shaft is the thicker, and then nothing is bored."""
        if not self.direct_drive or self.motor_face.rotor_driven:
            return float("inf")
        return (self.input_shaft_diameter - self.motor_bore_diameter) / 2.0

    @property
    def rotor_hub(self) -> tuple[float, float, float, float] | None:
        """The hub an outrunner's can drives the input shaft through:
        ``(z from, z to, outside diameter, bore)``, or None.

        On the can's face, which is the motor's far end from its base; the
        input shaft runs through the motor's hollow centre to it.  As wide as
        the can's bolt circle and the screws' holes need with a rim outside
        them, bored for the shaft with the process's hole clearance and held
        on it by a set screw on the shaft's flat.
        """
        if not self.direct_drive or not self.motor_face.rotor_driven:
            return None
        frame, side = self.motor_face, self.motor_side
        near = self.motor_face_z + side * frame.body_length
        far = near + side * ROTOR_HUB_THICKNESS
        size = size_named(frame.rotor_screw)
        hole = size.clearance if size else 3.4
        outer = frame.rotor_bolt_span + hole + 2.0 * ROTOR_HUB_RIM
        z0, z1 = sorted((near, far))
        return z0, z1, outer, self.input_shaft_diameter + self.hole_clearance

    @property
    def rotor_hub_screw(self) -> tuple[MetricSize, float] | None:
        """The can's screws through the hub: ``(size, length)``, or None.

        ISO 4762 at the shortest preferred length past the hub and three
        millimetres of the can's thread - which is how deep an outrunner's
        tapped face is as a rule, not a figure its catalogue gives.
        """
        if self.rotor_hub is None:
            return None
        size = size_named(self.motor_face.rotor_screw)
        if size is None:
            return None
        need = ROTOR_HUB_THICKNESS + ROTOR_CAN_THREAD
        length = next((n for n in SCREW_LENGTHS if n >= need - 1e-9), None)
        return (size, length) if length else None

    @property
    def rotor_hub_bolt_centres(self) -> list[tuple[float, float]]:
        """Where the can's screws go through the hub, in the plane."""
        if self.rotor_hub is None:
            return []
        frame = self.motor_face
        r, n = frame.rotor_bolt_span / 2.0, frame.rotor_bolt_count
        return [(r * math.cos(2.0 * math.pi * k / n), r * math.sin(2.0 * math.pi * k / n))
                for k in range(n)]

    @property
    def motor_fixing(self) -> MotorFixing | None:
        """The screws the motor is held on by, and the recess they need."""
        if not self.has_motor_face:
            return None
        if self.coupled:
            return motor_fixing(self.motor_face, self.motor_mount_flange,
                                recessed=False)
        return motor_fixing(self.motor_face, self.plate_thickness)

    @property
    def motor_bolt_centres(self) -> list[tuple[float, float]]:
        """Where the motor's screws go through its face, in the plane.

        NEMA patterns are a *square*.  Drawing them on a circle of the same size
        puts all four holes somewhere the motor has nothing.
        """
        if not self.has_motor_face:
            return []
        frame = self.motor_face
        if frame.square:
            half = frame.bolt_span / 2.0
            return [(sx * half, sy * half) for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)]
        r = frame.bolt_span / 2.0
        n = max(frame.bolt_count, 1)
        return [(r * math.cos(2.0 * math.pi * k / n), r * math.sin(2.0 * math.pi * k / n))
                for k in range(n)]

    @property
    def motor_face_z(self) -> float:
        """Where the motor's mounting face sits along the axis, assembled.

        The outside of whichever plate carries the motor's pattern: the input
        end plate on a carrier drive, the carrier's base on a ring drive.
        """
        face = self.plate_face_z
        if not self.coupled:
            return face
        # Through a coupling the motor stands off on a mount: far enough that
        # its shaft reaches the middle of the coupling and no further, and at
        # least far enough that the flange it lands on clears the coupling -
        # whose hole is the spigot's, narrower than the coupling turning in it.
        frame, c, side = self.motor_face, self.coupling, self.motor_side
        start = face + side * COUPLING_GAP
        middle = start + side * c.length / 2.0
        end = start + side * c.length
        reach = middle + side * (COUPLING_GAP / 2.0 + frame.shaft_length)
        clear = end + side * (COUPLING_GAP + self.motor_mount_flange)
        return max(reach, clear) if side > 0 else min(reach, clear)

    @property
    def motor_side(self) -> float:
        """Which way the motor stands off its face: +1 up the axis, -1 down."""
        return -1.0 if self.motor_mounts_on_carrier else 1.0

    @property
    def motor_mounts_on_carrier(self) -> bool:
        """Whether the motor face is cut into the carrier's base instead of the
        input end plate.  It goes on whichever member does not turn."""
        return self.has_motor_face and self.ground_frame_fitted

    @property
    def grounded_part(self) -> str:
        """The part that is bolted to the world, named as the assembly names it."""
        return ("housing" if self.output_member is OutputMember.CARRIER
                else "output_flange")

    @property
    def output_part(self) -> str:
        """The part the load comes off, named as the assembly names it."""
        return ("output_flange" if self.output_member is OutputMember.CARRIER
                else "housing")

    @property
    def housing_bolt_radius(self) -> float:
        """Where the tie bolts run: through the wall, clear of the pin pockets.

        Halfway between the deepest a pocket cuts and the outside, so a bolt
        misses the pins whose plate it is holding on and still has metal round
        it on both sides.
        """
        return self.housing_outer_radius - self.housing_wall / 2.0

    @property
    def rim_land(self) -> float:
        """Metal between the tie-bolt holes and the outside of the housing,
        which is what an edge break on the rim has to stay within.  On a ring
        drive the widest of those holes is the counterbore in the plate under
        the housing, and the land is what is left outside it."""
        if not self.housing_bolt_count:
            return self.housing_wall
        counterbore = self.tie_counterbore
        if counterbore is not None:
            return self._counterbore_room / 2.0 + COUNTERBORE_LAND - counterbore[0] / 2.0
        if self.ground_frame_fitted:
            return 0.0                  # no screw's counterbore fits there
        if self.square_housing:
            # A corner bolt sits on its corner's centre: the arc is as far
            # from it as the two flat sides are.
            return self.housing_corner_radius - self.housing_bolt_diameter / 2.0
        return (self.housing_outer_radius - self.housing_bolt_radius
                - self.housing_bolt_diameter / 2.0)

    # ---- the outline ------------------------------------------------------------
    #
    # Everything that draws, weighs or cools the housing and its end plates reads
    # these rather than a radius, so the square and the round are one question.
    # ``housing_outer_radius`` stays what it was: the round housing's radius, and
    # the square's half-width across the flats - the same metal past the pin
    # pockets either way, at the thinnest place.
    @property
    def square_housing(self) -> bool:
        return self.housing_shape is HousingShape.SQUARE

    @property
    def housing_corner_radius(self) -> float:
        """A square housing's corners are rounded to a wall; a round one's
        "corner" is its radius."""
        if self.square_housing:
            return self.housing_wall
        return self.housing_outer_radius

    def housing_outline(self, segments: int = 96) -> list[tuple[float, float]]:
        """The outline, counter-clockwise from the +x axis, as points: a circle
        of ``segments``, or a square with its corners rounded, a quarter of
        ``segments`` to each corner."""
        r = self.housing_outer_radius
        if not self.square_housing:
            return [(r * math.cos(2.0 * math.pi * k / segments),
                     r * math.sin(2.0 * math.pi * k / segments))
                    for k in range(segments)]
        rc = self.housing_corner_radius
        c = r - rc
        n = max(segments // 4, 6)
        points = []
        for cx, cy, start in ((c, c, 0.0), (-c, c, 0.5 * math.pi),
                              (-c, -c, math.pi), (c, -c, 1.5 * math.pi)):
            for k in range(n + 1):
                a = start + 0.5 * math.pi * k / n
                points.append((cx + rc * math.cos(a), cy + rc * math.sin(a)))
        return points

    @property
    def housing_section_area(self) -> float:
        """The area inside the outline, mm^2."""
        r = self.housing_outer_radius
        if not self.square_housing:
            return math.pi * r * r
        rc = self.housing_corner_radius
        return 4.0 * r * r - (4.0 - math.pi) * rc * rc

    @property
    def housing_polar_area_moment(self) -> float:
        """``integral r^2 dA`` over the outline, about the axis, mm^4.

        The square's is a square's, ``(2r)^4 / 6``, less what rounding takes
        off each corner: the corner square of side ``rc`` less the quarter disc
        that replaces it, both about the axis rather than about themselves.
        """
        r = self.housing_outer_radius
        if not self.square_housing:
            return math.pi * r ** 4 / 2.0
        rc = self.housing_corner_radius
        a = r - rc
        corner_square = 2.0 * rc * (r ** 3 - a ** 3) / 3.0
        quarter = (math.pi * rc ** 4 / 8.0 + 4.0 * a * rc ** 3 / 3.0
                   + math.pi * a * a * rc * rc / 2.0)
        return (2.0 * r) ** 4 / 6.0 - 4.0 * (corner_square - quarter)

    @property
    def housing_perimeter(self) -> float:
        """Round the outline, mm."""
        r = self.housing_outer_radius
        if not self.square_housing:
            return 2.0 * math.pi * r
        rc = self.housing_corner_radius
        return 8.0 * r - (8.0 - 2.0 * math.pi) * rc

    @property
    def housing_size_text(self) -> str:
        """What the outline is called on a bill or a title block."""
        size = f"{2 * self.housing_outer_radius:.1f}"
        return f"{size} square" if self.square_housing else f"{size} dia"

    @property
    def tie_bolt_centres(self) -> list[tuple[float, float]]:
        """Where the tie bolts go through the plates and the barrel.

        Round: ``housing_bolt_count`` of them on the wall's middle circle from
        the +x axis.  Square: four, one on each rounded corner's centre, where
        the corner's arc and both flat sides are a wall away.
        """
        n = self.housing_bolt_count
        if not n:
            return []
        if self.square_housing:
            c = self.housing_outer_radius - self.housing_corner_radius
            return [(c, c), (-c, c), (-c, -c), (c, -c)]
        if self.ground_frame_fitted:
            r = self.cup_screw_radius
            return [(r * math.cos(a), r * math.sin(a)) for a in self._cup_screw_angles]
        r = self.housing_bolt_radius
        return [(r * math.cos(2.0 * math.pi * k / n), r * math.sin(2.0 * math.pi * k / n))
                for k in range(n)]

    # ---- the screws under a ring drive's housing ----------------------------------
    #
    # The housing is one part with its closed top there, so the only joint left
    # is the plate under it, which carries the lower main bearing.  Its screws
    # come up from below into the barrel and their heads are let into the plate
    # - nothing turns in the gap over the base.  A counterbore is wider than
    # the screw, and down the middle of a 6 mm wall it breaks out of the rim;
    # midway between the ring pins and the outside, and between two pins, is
    # where both it and the thread past it have metal round them.  The screw is
    # then the largest, no bigger than the tie-bolt hole asks for, whose
    # counterbore leaves :data:`COUNTERBORE_LAND` to the outside.
    @property
    def cup_screw_radius(self) -> float:
        """The circle a round ring drive's plate screws are on: midway between
        the pin circle and the outside."""
        return 0.5 * (self.pin_circle_radius + self.housing_outer_radius)

    @property
    def _cup_screw_angles(self) -> list[float]:
        """``housing_bolt_count`` angles as nearly equally spaced as the gaps
        between the ring pins allow, each in the middle of one - the pins sit
        at whole pitches from the +x axis."""
        pitch = 2.0 * math.pi / self.pin_count
        out: list[float] = []
        for j in range(self.housing_bolt_count):
            target = 2.0 * math.pi * j / self.housing_bolt_count
            a = (math.floor(target / pitch) + 0.5) * pitch
            if not any(abs(math.remainder(a - b, 2.0 * math.pi)) < 1e-9 for b in out):
                out.append(a)
        return out

    @property
    def _counterbore_room(self) -> float:
        """The widest counterbore the plate under a ring drive's housing takes
        round each screw, mm: to :data:`COUNTERBORE_LAND` of the outside - a
        round housing's rim, or a square one's corner arc."""
        if self.square_housing:
            reach = self.housing_corner_radius
        else:
            reach = self.housing_outer_radius - self.cup_screw_radius
        return 2.0 * (reach - COUNTERBORE_LAND)

    @property
    def tie_counterbore(self) -> tuple[float, float] | None:
        """``(diameter, depth)`` of the counterbore a ring drive's plate screw
        is let into from below: the head and 0.6 mm, 0.3 mm deeper than the
        head, as the motor's screws are let into a plate.  None off the ring,
        where the heads stand on the output plate."""
        size = self.tie_bolt_size
        if not self.ground_frame_fitted or size is None or not self.tie_bolt_centres:
            return None
        return size.head_d + 0.6, size.head_h + 0.3

    # ---- the dowels that locate the end plates on the barrel -------------------
    #
    # The plates carry the shaft supports and the output bearing, so they set
    # where the crank's axis is; the ring pins are in the barrel.  Nothing but
    # the tie bolts joined the two, and a bolt in a clearance hole locates
    # nothing - an M4 in 4.5 mm lets a plate sit half a millimetre off, against
    # a machined drive's three hundredths of profile clearance.  So two dowels
    # per joint, pressed into the barrel and a tight slide in the plate.
    @property
    def plate_dowel_diameter(self) -> float:
        """ISO 8734 dowel, mm: the largest that leaves a quarter of the wall
        either side of it on the tie-bolt circle; zero with none."""
        if not self.plate_dowels:
            return 0.0
        return dowel_for_wall(self.housing_wall) or 0.0

    @property
    def input_plate_holes(self) -> list[tuple[float, float, float, str]]:
        """Every hole through the input end plate but its bore and the dowels':
        ``(x, y, diameter, what)``, at its widest - a counterbore or a nut's
        pocket where it has one, since that is the metal it takes."""
        holes = [(x, y, self.housing_bolt_diameter, "a tie bolt")
                 for x, y in self.tie_bolt_centres]
        if self.ground_frame_fitted and self.output_bolt_count:
            r, m = self.output_face_bolt_radius, self.output_bolt_count
            for k in range(m):
                a = self.output_bolt_phase + 2.0 * math.pi * k / m
                holes.append((r * math.cos(a), r * math.sin(a),
                              self.output_bolt_diameter, "an output bolt"))
        elif self.has_motor_face and not self.coupled:
            fix = self.motor_fixing
            hole = max(self.motor_face.bolt_diameter, fix.recess if fix else 0.0)
            holes += [(x, y, hole, "a motor screw") for x, y in self.motor_bolt_centres]
        return holes

    def plate_dowel_metal(self, angle: float) -> tuple[float, str]:
        """The least metal between a dowel at ``angle`` on the tie-bolt circle
        and any other hole through the input plate, mm, and which hole."""
        r, d = self.housing_bolt_radius, self.plate_dowel_diameter
        x, y = r * math.cos(angle), r * math.sin(angle)
        holes = self.input_plate_holes
        if self.ground_frame_fitted:
            # The only plate is the one under the housing, and what it has
            # round the dowels is its screws' counterbores.
            counterbore = self.tie_counterbore
            width = counterbore[0] if counterbore else self.housing_bolt_diameter
            holes = [(hx, hy, width, "a plate screw's counterbore")
                     for hx, hy in self.tie_bolt_centres]
        return min(((math.hypot(hx - x, hy - y) - (d + hd) / 2.0, what)
                    for hx, hy, hd, what in holes),
                   default=(math.inf, ""))

    @property
    def plate_dowel_angles(self) -> tuple[float, ...]:
        """Where the two dowels sit on the tie-bolt circle, radians.

        A quarter of a tie-bolt pitch round, which is as far as can be from both
        the tie bolts and the output bolts that sit half a pitch round on a
        ring drive.  The first at the first such place; the second at whichever
        other one keeps a millimetre clear of the motor's pattern and stands
        nearest opposite the first - but never exactly opposite, and never
        where a flip would bring the pair back onto itself, so that a plate
        goes on one way only and the seats it carries line up with the barrel
        they were bored to match.  (A second dowel three quarters of a pitch
        round would mirror the first across a bolt.)
        """
        if not self.plate_dowel_diameter:
            return ()
        if self.square_housing or self.ground_frame_fitted:
            # Off the quarter-pitch rule's circle: in the corners, or between
            # the ring pins.  Searched for instead.
            return self._square_dowel_angles()
        n = len(self.tie_bolt_centres) or 6
        pitch = 2.0 * math.pi / n
        first = 0.25 * pitch
        candidates = [(k + 0.25) * pitch for k in range(1, n) if 2 * k != n]
        if not candidates:                  # one or two bolts: anywhere off-axis
            return (first, 0.6 * pitch)

        def merit(angle: float) -> tuple[bool, float, float]:
            metal = self.plate_dowel_metal(angle)[0]
            apart = abs(math.remainder(angle - first, 2.0 * math.pi))
            return metal >= 1.0, -abs(apart - math.pi), metal

        return (first, max(candidates, key=merit))

    def _square_dowel_angles(self) -> tuple[float, float]:
        """The two dowels on a square housing's plates.

        Its tie bolts are off the dowels' circle, out in the corners, so the
        quarter-pitch rule has nothing to measure from: the circle is searched
        instead, every 15 degrees.  The first dowel goes where the most metal
        is round it; the second as the round rule puts it - a millimetre clear,
        then nearest opposite - and the pair may not come back onto itself
        under any of the eight ways a square plate can be turned or flipped
        onto its bolts.
        """
        grid = [math.radians(15.0 * k) for k in range(24)]
        first = max(grid, key=lambda a: round(self.plate_dowel_metal(a)[0], 9))

        def near(x: float, y: float) -> bool:
            return abs(math.remainder(x - y, 2.0 * math.pi)) < 1e-9

        def on_itself(a: float, b: float) -> bool:
            for k in range(4):
                turn = 0.5 * math.pi * k
                # Turned a quarter at a time, and flipped across each axis of
                # symmetry; leaving it as it is is not a way of getting it wrong.
                images = [(turn - a, turn - b)] + ([(a + turn, b + turn)] if k else [])
                for p, q in images:
                    if (near(p, a) and near(q, b)) or (near(p, b) and near(q, a)):
                        return True
            return False

        def merit(angle: float) -> tuple[bool, float, float]:
            metal = self.plate_dowel_metal(angle)[0]
            apart = abs(math.remainder(angle - first, 2.0 * math.pi))
            return metal >= 1.0, -abs(apart - math.pi), metal

        return (first, max((a for a in grid
                            if a != first and not on_itself(first, a)), key=merit))

    @property
    def plate_dowel_centres(self) -> list[tuple[float, float]]:
        """The dowels' centres in the plane, on the tie-bolt circle."""
        r = self.housing_bolt_radius
        return [(r * math.cos(a), r * math.sin(a)) for a in self.plate_dowel_angles]

    @property
    def printed(self) -> bool:
        """Whether the drive's parts come off a printer, which holds no fit."""
        return self.process in (Process.FDM, Process.SLA, Process.SLS)

    @property
    def plate_dowel_hole(self) -> float:
        """The dowel's hole through an end plate, mm.

        Its own diameter when machined: reamed to H7 it is a tight slide, and
        the fit is a note on the drawing, not a size.  Printed, a hole holds no
        fit, so it gets the design's own allowance for a hole that takes a part
        - the barrel's stays at the dowel, which is a press into plastic.
        """
        d = self.plate_dowel_diameter
        return d + self.hole_clearance if d and self.printed else d

    @property
    def plate_dowel_engagement(self) -> tuple[float, float]:
        """How far each dowel goes into ``(the plate, the barrel)``, mm.

        Into the plate to half a millimetre short of its outside face, so it
        locates over the plate's whole depth and never stands proud of it; into
        the barrel one and a half diameters, the usual hold for a press fit.
        """
        d = self.plate_dowel_diameter
        return (max(self.housing_plate_thickness - 0.5, 0.0), 1.5 * d)

    @property
    def plate_dowel_length(self) -> float | None:
        """The preferred ISO 8734 length that makes both engagements; None with
        no dowels or past the table."""
        if not self.plate_dowel_diameter:
            return None
        return dowel_length(sum(self.plate_dowel_engagement))

    @property
    def plate_dowel_spans(self) -> list[tuple[float, float]]:
        """Where the dowels are along the axis, ``(z0, z1)``: the input plate's
        pair, then the output plate's - each into its plate as far as
        :attr:`plate_dowel_engagement` says and the rest of its length into the
        barrel, whose hole runs right through."""
        length = self.plate_dowel_length
        if not length:
            return []
        into_plate = self.plate_dowel_engagement[0]
        top = self.barrel_top + into_plate
        bottom = self.barrel_bottom - into_plate
        if self.ground_frame_fitted:
            # One joint: the housing's top is the housing.
            return [(bottom, bottom + length)]
        return [(top - length, top), (bottom, bottom + length)]

    @property
    def housing_dowel_depth(self) -> float:
        """How far a ring drive's dowel holes go up into the housing's
        underside: as far as the dowel does, and :data:`DOWEL_SUMP`.  The
        housing is closed at the top, so they are blind."""
        spans = self.plate_dowel_spans
        if not self.ground_frame_fitted or not spans:
            return 0.0
        return spans[0][1] - self.barrel_bottom + DOWEL_SUMP

    @property
    def edge_break(self) -> float:
        """The 45 degree break the end plates actually get: the design's, held
        to a quarter of the plate and half the rim land so it cannot run into
        the tie-bolt holes or through a thin plate."""
        return min(self.edge_chamfer, self.housing_plate_thickness / 4.0,
                   self.rim_land / 2.0)

    @property
    def tie_bolt_bottom(self) -> float:
        """Where a tie bolt starts: the outer face of the output end plate, or
        on a ring drive the floor of the counterbore its head is let into."""
        bottom = self.barrel_bottom - self.housing_plate_thickness
        counterbore = self.tie_counterbore
        return bottom + counterbore[1] if counterbore is not None else bottom

    @property
    def tie_bolt_length(self) -> float:
        """Under the head: through one plate, the barrel, and the other plate.

        Off the barrel rather than off the disc stack, which is what the bill
        of materials used to say.  When the barrel was lengthened to reach the
        plates it bolts to, this did not follow it - so the drive was ordering
        bolts seven millimetres short of the thing they had to pass through.
        One property now, read by the bill of materials and by both renderers.

        On a ring drive, what the screw passes through before it threads into
        the housing: the plate under it, below its counterbore.
        """
        if self.ground_frame_fitted:
            return self.barrel_bottom - self.tie_bolt_bottom
        return self.barrel_height + 2.0 * self.plate_thickness

    @property
    def input_flat(self) -> tuple[float, float, float] | None:
        """The flat on the shaft's input end: ``(from z, to z, depth)``, or None.

        What a coupling's set screw bears on, so it drives through a face
        instead of biting a round shaft and walking.  A tenth of the diameter
        deep, which is what a stepper's own D-cut is - half a millimetre on a
        5 mm shaft - and enough to seat a screw point without thinning the
        shaft where it carries torque.  From a millimetre clear of the input
        plate to the end, which is the length a coupling grips.

        None on a ring-output drive.  The motor is on the carrier's base there,
        the shaft's driven end is inside the boss, and a set screw has no way
        in; the end that does stand out, past the end cap, drives nothing.
        """
        if self.rotor_hub is not None:
            z0, z1 = self.rotor_hub[:2]
            return z0, z1, 0.1 * self.input_shaft_diameter
        if self.direct_drive:
            return None
        lo, hi = self.shaft_span
        if self.coupled:
            # Along the shaft's half of the coupling, at whichever end that
            # is: down through the base's mount on a ring drive, where the
            # mount's windows are what a key gets in by.
            c0, c1 = self.coupling_span
            z0, z1 = (c0, hi) if self.motor_side > 0 else (lo, c1)
        elif self.ground_frame_fitted:
            return None
        else:
            z0, z1 = self.barrel_top + self.plate_thickness + 1.0, hi
        if z1 - z0 < 2.0:
            return None
        return z0, z1, 0.1 * self.input_shaft_diameter

    @property
    def tie_bolt_size(self) -> MetricSize | None:
        """The bolt the tie-bolt holes are drilled for, read off the hole.

        ``None`` with no tie bolts, or a hole too small for any size in the
        table - in which case the drive is drawn and ordered as it always was,
        by the hole and a length.  On a ring drive no bigger than that, and no
        bigger than a counterbore in the plate under the housing takes - and
        ``None`` where none does, which ``HOUSING_BOLT_CLASH`` stops.
        """
        if not self.housing_bolt_count:
            return None
        asked = self.bolt_size_asked
        if not self.ground_frame_fitted or asked is None:
            return asked
        room = self._counterbore_room
        fits = [s for s in SIZES if s.d <= asked.d + 1e-9 and s.head_d + 0.6 <= room + 1e-9]
        return fits[-1] if fits else None

    @property
    def bolt_size_asked(self) -> MetricSize | None:
        """The screw the tie-bolt hole is the clearance for - what the base's
        own screws are, whatever the housing's turned out to be."""
        if not self.housing_bolt_count:
            return None
        return size_for_hole(self.housing_bolt_diameter)

    @property
    def tie_screw_length(self) -> float | None:
        """The preferred screw length: into the input plate's thread where it
        is tapped, otherwise the one that clamps the stack with a nut on it."""
        tapped = self._tapped_tie_screw_length
        if tapped is not None:
            return tapped
        size = self.tie_bolt_size
        return screw_length(self.tie_bolt_length, size) if size else None

    # ---- threaded into the housing ---------------------------------------------
    #
    # On a ring-output drive the plate under the housing is screwed up into the
    # housing's underside: tapped there, or with an insert where one is asked
    # for and has room.  See the screws under a ring drive's housing, above.
    @property
    def tie_thread_engagement(self) -> float:
        """The least thread a tie screw has in a tapped plate, mm: one and a
        half diameters in metal or in a brass insert, two in a polymer, whose
        thread shears first."""
        return self._thread_engagement(self.tie_insert)

    def _thread_engagement(self, insert: HeatSetInsert | None,
                           size: MetricSize | None = None) -> float:
        size = size or self.tie_bolt_size
        if size is None:
            return 0.0
        polymer = self.housing_mat.E_GPa < 10.0 and insert is None
        return (2.0 if polymer else 1.5) * size.d

    # ---- heat-set inserts ------------------------------------------------------
    #
    # A printed part's own thread strips at a fraction of what the screw takes;
    # a knurled brass insert pressed in hot carries it on its whole length.  Asked
    # for with ``heat_set_inserts`` and only in a polymer.  On a ring drive they
    # go wherever a screw threads into the print: up into the turning housing's
    # underside for the screws that hold the plate under it, down into its face
    # for the driven machine's, and up into the base for the screws that hold
    # it down.  Each needs a wall round it the tapped thread did not, and where
    # there is not one that part is tapped as before and ``INSERTS_DO_NOT_FIT``
    # says what would take them.
    @property
    def tie_insert_wanted(self) -> HeatSetInsert | None:
        """The insert the tie screws' size takes, where inserts are asked for
        in a polymer part."""
        if not self.heat_set_inserts or self.housing_mat.E_GPa >= 10.0:
            return None
        return insert_for(self.tie_bolt_size)

    @property
    def tie_insert_wall(self) -> float:
        """Wall left round the insert hole a ring drive's plate screw threads
        into in the housing's underside, mm: to the outside, and on a round
        housing to the ring-pin pockets either side of it as well."""
        wanted = self.tie_insert_wanted
        if wanted is None or not self.tie_bolt_centres:
            return math.inf
        return self._wall_round(self.tie_bolt_centres, wanted.hole)

    def _wall_round(self, centres: list[tuple[float, float]], hole: float) -> float:
        """The least wall round holes of ``hole`` at ``centres`` in the
        barrel's wall: to the outline, and to the nearest ring-pin pocket."""
        pins = [(self.pin_circle_radius * math.cos(2.0 * math.pi * k / self.pin_count),
                 self.pin_circle_radius * math.sin(2.0 * math.pi * k / self.pin_count))
                for k in range(self.pin_count)]
        walls = []
        for x, y in centres:
            walls.append(self._inside_outline(x, y) - hole / 2.0)
            walls.append(min(math.hypot(x - px, y - py) for px, py in pins)
                         - self.pin_radius - hole / 2.0)
        return min(walls)

    def _inside_outline(self, x: float, y: float) -> float:
        """How far ``(x, y)`` is inside the housing's outline, mm: off the
        circle, or off the square's flats and rounded corners."""
        if not self.square_housing:
            return self.housing_outer_radius - math.hypot(x, y)
        rc = self.housing_corner_radius
        k = self.housing_outer_radius - rc
        qx, qy = abs(x) - k, abs(y) - k
        return rc - math.hypot(max(qx, 0.0), max(qy, 0.0)) - min(max(qx, qy), 0.0)

    @property
    def tie_insert(self) -> HeatSetInsert | None:
        """The insert a ring drive's housing takes the screws that hold the
        plate under it in: asked for, and a wall of :data:`INSERT_WALL` round
        its hole."""
        wanted = self.tie_insert_wanted
        if (wanted is None or not self.ground_frame_fitted
                or self.tie_insert_wall < INSERT_WALL):
            return None
        return wanted

    @property
    def tie_thread_hole(self) -> float:
        """The hole a ring drive's plate screws thread into in the housing:
        the insert's where there is one, otherwise the tap drill."""
        insert = self.tie_insert
        return insert.hole if insert is not None else self.tie_tap_drill

    @property
    def base_insert(self) -> HeatSetInsert | None:
        """The insert the base's own screws thread into, asked for in a
        polymer base - its flange has the room round it the housing wall may
        not, and takes its length and cover where the pad is that thick."""
        if not self.heat_set_inserts or self.housing_mat.E_GPa >= 10.0:
            return None
        wanted = insert_for(self.bolt_size_asked)
        if (wanted is None or not self.base_has_flange
                or wanted.length + INSERT_SUMP + TAPPED_COVER > self.plate_thickness):
            return None
        return wanted

    @property
    def base_mount_hole(self) -> float:
        """The hole the base is bolted down into: the insert's, or the tap
        drill of the screw the tie-bolt hole asks for."""
        insert = self.base_insert
        if insert is not None:
            return insert.hole
        size = self.bolt_size_asked
        return size.d - size.pitch if size else 0.0

    @property
    def tie_tap_drill(self) -> float:
        """The hole a tapped tie screw goes in: the diameter less a pitch, the
        usual tap drill for a coarse thread."""
        size = self.tie_bolt_size
        return size.d - size.pitch if size else 0.0

    @property
    def _tapped_tie_screw_length(self) -> float | None:
        """The shortest preferred length with the thread it needs in the
        housing past the plate - the hole is drilled to suit it."""
        if not self.ground_frame_fitted or self.tie_bolt_size is None:
            return None
        need = self.tie_bolt_length + self.tie_thread_engagement
        return next((n for n in SCREW_LENGTHS if n >= need - 1e-9), None)

    @property
    def tie_thread_depth(self) -> float:
        """How deep the hole a ring drive's plate screw threads into goes up
        into the housing from its underside: the screw's end and
        :data:`TAPPED_COVER` past it, or the insert and its sump."""
        length = self.tie_screw_length
        if not self.ground_frame_fitted or length is None:
            return 0.0
        depth = length - self.tie_bolt_length + TAPPED_COVER
        insert = self.tie_insert
        if insert is not None:
            depth = max(depth, insert.length + INSERT_SUMP)
        return depth

    @property
    def tie_screws_tapped(self) -> bool:
        """Whether the tie screws thread into a part rather than taking nuts:
        on a ring-output drive, where the plate under the housing is screwed
        up into the housing's underside."""
        return self._tapped_tie_screw_length is not None

    @property
    def ring_pin_length(self) -> float:
        """A ring pin: the whole barrel, trapped between the two end plates.

        The same oversight as the tie bolt above, one part along.  A pocket is
        broached down the bore in one pass, so it runs the barrel's whole length
        - and a pin cut to the disc stack instead leaves seven millimetres of
        empty groove under it with nothing at the bottom but the end plate.
        Nothing holds it up there.  It slides down, and what it slides out of is
        the mesh: a third of its engagement gone, and an open pocket left at the
        top for a lobe to drop into.

        Axial location is the whole reason for the length.  Only the disc stack
        loads a pin, so this is not carrying capacity - it is the pin still being
        where the load is after an hour of running.
        """
        return self.barrel_height

    @property
    def output_pin_length(self) -> float:
        """An output pin: the carrier drop it crosses, then the discs it drives.

        It starts on the carrier face, which sits a drop below the first disc, so
        a pin cut to the disc stack alone arrives one drop short of the top of
        it.  On a two-disc drive that is the last disc driven over seven of its
        eight millimetres - and the bearing stress this app reports for that hole
        is computed over all eight.

        With a frame it crosses a second drop at the far end and goes *through*
        the end cap, which is what turns it from a cantilever into a beam and
        what holds the frame together: these are the drive's own tie bolts in
        that configuration, threaded into the carrier and headed on the outside
        of the cap.  The extra few millimetres of pin is the cheapest structure
        in the drive.
        """
        span = self.stack_height + CARRIER_DROP
        if self.ground_frame_fitted:
            span += CARRIER_DROP + self.output_flange_thickness
        return span

    @property
    def input_plate_bore(self) -> float:
        """The hole in the input end plate.

        The shaft support's seat normally.  With a frame it is the *output
        bearing's* seat instead: the end cap's boss comes up through this plate,
        the second main bearing rides on it, and the shaft support moves inside
        that boss - which is exactly what happens at the other end, one plate
        further out.  Two ends of one symmetrical frame.
        """
        return (self.output_bearing_seat_diameter if self.ground_frame_fitted
                else self.hub_bore)

    @property
    def output_bearing_seat_diameter(self) -> float:
        """Bore in the output end plate: what the main output bearing sits *in*.

        A housing wall out from the hub on each side, so the ring between them is
        the space the bearing has.  With the default shaft that is 30 mm on 42,
        and a 6806 is 30 on 42.
        """
        return self.hub_diameter + 2.0 * self.housing_wall

    @property
    def disc_phases(self) -> list[float]:
        """Crank phase of each disc in the stack: 180 deg for two, 120 deg for three."""
        import math
        return [2.0 * math.pi * i / self.disc_count for i in range(self.disc_count)]

    @property
    def disc_hole_phases(self) -> list[float]:
        """How far each disc's output-hole pattern is rotated against its lobes.

        A disc sitting on crank phase ``p`` must rotate by ``p/lobes`` to stay
        meshed with the ring, but every disc is coupled to the *same* output
        carrier and so must share the carrier's rotation.  The only way both hold
        is to pre-rotate the hole pattern by ``-p/lobes`` so it lands back on the
        carrier pins.

        Consequence: in a multi-disc stack the discs are **different parts**,
        unless :attr:`discs_are_identical`.
        """
        return [-p / self.lobes for p in self.disc_phases]

    @property
    def discs_are_identical(self) -> bool:
        """True when the hole pre-rotation happens to be a whole hole pitch.

        That needs ``output_pin_count`` to be a multiple of ``2*lobes``, which is
        usually far too many pins to be practical - so normally this is False.
        """
        import math
        if self.disc_count == 1:
            return True
        pitch = 2.0 * math.pi / self.output_pin_count
        return all(min(abs(ph % pitch), abs(ph % pitch - pitch)) < 1e-9
                   for ph in self.disc_hole_phases)

    @property
    def disc_mat(self) -> Material:
        return MATERIALS[self.disc_material]

    @property
    def pin_mat(self) -> Material:
        return MATERIALS[self.pin_material]

    @property
    def housing_mat(self) -> Material:
        return MATERIALS[self.housing_material]

    @property
    def shaft_mat(self) -> Material:
        return MATERIALS[self.shaft_material]

    @property
    def lube(self) -> Lubricant:
        """The lubricant, falling back to dry rather than raising.

        Same reasoning as the bearing designations: a saved design naming a
        lubricant this build does not know should cost a conservative answer,
        not a file that will not open.
        """
        return LUBRICANTS.get(self.lubricant, LUBRICANTS[DRY])

    @property
    def roughness_um(self) -> float:
        """RMS roughness of one sliding face, stated or typical for the process."""
        if self.surface_roughness_um is not None:
            return self.surface_roughness_um
        return PROCESS_ROUGHNESS_UM[self.process]

    @property
    def envelope_length(self) -> float:
        """Axial length of the assembled gearbox, end plate face to end plate face.

        The plates are part of the gearbox: they close it, they carry the shaft
        supports and the output bearing, and they are what it bolts to the world
        by.  Leaving them out of the envelope understated the length of every
        drive this app has ever sized.

        Stated as the barrel plus its two plates rather than as the same sum
        written out again, because the two have to agree: this was already
        counting the carrier's share of the length while the barrel itself
        stopped at the discs, so the app reported an envelope the geometry did
        not fill.

        A ring-output drive is longer by its base and the standoff the base
        needs - and the barrel itself is longer, because the end cap lives
        inside it.  The boss protrusion is not a face of the gearbox when the
        carrier is the output: it is a shaft end, and nobody counts a shaft
        end in a gearbox's length.  When the carrier is the *ground* that same
        protrusion is structure between two faces of the machine, with the
        plate you bolt it down by on the end of it.
        """
        if self.ground_frame_fitted:
            # The base's underside to the housing's closed face.
            return self.cup_top - self.base_plate_bottom
        return self.barrel_height + 2.0 * self.plate_thickness

    @property
    def cooling_area_mm2(self) -> float:
        """Outside surface the housing can shed heat through.

        Barrel plus the two end faces.  Whatever the gearbox is bolted to will
        conduct some heat away as well, which this deliberately ignores - it is
        the pessimistic, free-standing case.
        """
        return (self.housing_perimeter * self.envelope_length
                + 2.0 * self.housing_section_area)

    # ---------------------------------------------------------------- helpers --
    @property
    def ring_pins_roll(self) -> bool:
        """Whether the ring contact actually rolls.

        Not the same as ``ring_pins_are_rollers``, which is what was *asked
        for*: a pin formed with the housing is not free to turn in it, however
        the two flags were set.  Derived rather than enforced by a validator,
        because a validator would have to fire on every route a spec arrives
        by - construction, assignment, ``model_copy``, a design file, a sweep -
        and ``model_copy`` does not run them at all.  A design that names both
        keeps the roller preference for when the pins stop being integral, and
        every consumer reads this instead.
        """
        return self.ring_pins_are_rollers and not self.ring_pins_integral

    @model_validator(mode="after")
    def _check_materials(self) -> GearSpec:
        for field in ("disc_material", "pin_material", "housing_material",
                      "shaft_material"):
            if getattr(self, field) not in MATERIALS:
                raise ValueError(f"unknown material {getattr(self, field)!r}")
        return self

    @property
    def first_layer_chamfer(self) -> float:
        """The break round a printed part's bed face, from the process alone.

        Not a setting: it follows the process the way the elephant's foot
        does, so switching process cannot leave one behind.
        """
        return FIRST_LAYER_CHAMFER.get(self.process, 0.0)

    @property
    def contact_length(self) -> float:
        """How long the line contacts on a disc are, along its thickness.

        The thickness, less the first-layer break where the disc is printed -
        that break is on the flank and in the output holes, which are where the
        pins bear, so the bottom of each line is gone.
        """
        if self.disc_material in PRINT_MATERIALS:
            return self.disc_thickness - self.first_layer_chamfer
        return self.disc_thickness

    def apply_process_defaults(self) -> GearSpec:
        """Reset the two clearances to the design guide for the selected process."""
        prof, hole = PROCESS_CLEARANCE[self.process]
        self.profile_clearance = prof
        self.hole_clearance = hole
        return self


def preset(ratio: int) -> GearSpec:
    """Sensible starting points for prototype gearboxes, ratios 10:1 to 59:1."""
    table = {
        10: {"pin_circle_radius": 45.0, "pin_radius": 4.5, "eccentricity": 2.0, "output_bolt_circle_radius": 27.0, "center_bore_diameter": 22.0},
        15: {"pin_circle_radius": 50.0, "pin_radius": 4.0, "eccentricity": 1.6, "output_bolt_circle_radius": 30.0, "center_bore_diameter": 24.0},
        21: {"pin_circle_radius": 55.0, "pin_radius": 3.8, "eccentricity": 1.4, "output_bolt_circle_radius": 33.0, "center_bore_diameter": 26.0},
        29: {"pin_circle_radius": 60.0, "pin_radius": 3.5, "eccentricity": 1.1, "output_bolt_circle_radius": 36.0, "center_bore_diameter": 28.0},
        39: {"pin_circle_radius": 65.0, "pin_radius": 3.2, "eccentricity": 0.9, "output_bolt_circle_radius": 39.0, "center_bore_diameter": 30.0},
        59: {"pin_circle_radius": 70.0, "pin_radius": 3.0, "eccentricity": 0.7, "output_bolt_circle_radius": 42.0, "center_bore_diameter": 32.0},
    }
    closest = min(table, key=lambda k: abs(k - ratio))
    return GearSpec(lobes=ratio, **table[closest]).apply_process_defaults()
