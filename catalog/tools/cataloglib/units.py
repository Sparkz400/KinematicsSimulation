"""Unit conversions from the units vendor documents print to SI, and back, for the column maps of `legacy.py`.

A conversion by a power of ten is done in decimal, so that 0.093 kg cm2 becomes 0.0000093 kg m2 and not
9.300000000000001e-06. A conversion through pi (r/min, arcmin) is done in binary and written with 17 significant
digits, which reads back to the same double.
"""

from decimal import Decimal
import math

# name: (factor to SI, exact in decimal)
CONVERSIONS = {
    "same": (1, True),
    "milli": (Decimal("1e-3"), True),      # mm -> m, mH -> H, ms -> s, g/m -> kg/m, mm/m -> 1
    "kilo": (Decimal("1e3"), True),        # kW -> W, kJ -> J, kHz -> Hz
    "micro": (Decimal("1e-6"), True),      # uF -> F
    "centi_inverse": (Decimal("1e2"), True),  # N/cm -> N/m
    "percent": (Decimal("1e-2"), True),    # % -> 1
    "kgcm2": (Decimal("1e-4"), True),      # kg cm2 -> kg m2
    "minute": (Decimal("60"), True),       # min -> s
    "per_100_mm": (Decimal("10"), True),   # kg per 100 mm -> kg/m
    "ten": (Decimal("10"), True),          # Festo's motor factor per kg cm2, times 1e-3 -> per kg m2
    "rpm": (math.pi / 30, False),          # r/min -> rad/s
    "arcmin": (math.pi / 10800, False),    # arcmin -> rad
    "per_arcmin": (10800 / math.pi, False),  # N m/arcmin -> N m/rad
    "v_per_krpm": (30 / (1000 * math.pi), False),  # V per 1000 r/min -> V s/rad
}


def to_si(text, conversion):
    """The SI value of a number printed as `text`, as text; "" stays ""."""
    if text == "" or conversion == "same":
        return text
    factor, exact = CONVERSIONS[conversion]
    if exact:
        value = (Decimal(text) * factor).normalize()
        return format(value, "f")
    return repr(float(text) * factor)


def from_si(text, conversion):
    """The inverse of `to_si`, as a float, for the round trip check."""
    if text == "":
        return None
    factor, exact = CONVERSIONS[conversion]
    return float(text) / float(factor)
