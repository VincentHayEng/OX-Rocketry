#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#! python3.11

"""
LORK — Lykos Physics-True Runner (finalized)

What this version does:
- Supports four configs: ballistic, main_at_apogee, nominal, drogue_only.
- Robust parachute add (pre-Flight), tolerant across RocketPy API variants.
- Mean-wind from NetCDF if available; turbulence-only fallback otherwise.
- Turbulence wrapper with altitude correlation; TI sweep support.
- Inclination convention auto-detect (user inputs degrees FROM VERTICAL).
- Landing coordinates rotated by rail heading -> ENU -> lat/lon (pad-anchored).
- CSV with QA columns; KMZ export appends a Folder with Placemarks.
- CLI: choose single/grid, config, TI, tilt, heading, export toggles.

Notes:
- ASCII-only source to avoid paste/encoding issues.
"""

from __future__ import annotations

import argparse
import logging
import math
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

# ------------------------------
# Keep them. If you need to add imports, add them below this block.
# ------------------------------
try:
    from netCDF4 import Dataset  # optional; script runs without
except Exception:
    Dataset = None

try:
    from scipy.interpolate import RegularGridInterpolator, interp1d
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False

# ------------------------------
# Let these stay.
# ------------------------------
try:
    from rocketpy import Rocket as RocketPy, SolidMotor, Environment, Flight
    try:
        from rocketpy import Parachute  # not required in this script
    except Exception:
        Parachute = None
except Exception as e:
    print("ERROR: RocketPy import failed:", e)
    raise

# ------------------------------
# please do not touch this.
# ------------------------------
import sys

def _setup_logging(level=logging.INFO):
    """
    Configure the LORK logger with a dedicated stdout handler.
    propagate=False ensures LORK messages never double-log through root,
    and root-level changes by RocketPy or other libraries cannot silence us.
    """
    logger = logging.getLogger("LORK")
    logger.propagate = False  # isolate from root entirely

    # Remove any stale handlers (e.g. from a previous run in the same process)
    for h in list(logger.handlers):
        try:
            logger.removeHandler(h)
            h.close()
        except Exception:
            pass

    sh = logging.StreamHandler(stream=sys.stdout)
    sh.setLevel(level)
    sh.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
    logger.addHandler(sh)
    logger.setLevel(level)
    return logger

logger = _setup_logging(logging.INFO)

# ------------------------------
# ze paths
# ------------------------------
BASE_DIR = Path(r"C:\Users\jakwm\OneDrive - Algonquin College\Lykos Sims\Python Runs\WindGrabberForLykos")
TEMPLATE_KMZ = BASE_DIR / "LC2025 Range Layout.kmz"
ENG_FILE = Path(r"C:\Users\jakwm\OneDrive - Algonquin College\Lykos Sims\Python Runs\L820CTI.eng")
NC_FILE = Path(r"C:\Users\jakwm\OneDrive - Algonquin College\Lykos Sims\Python Runs\LykosCDF.nc")
CD_MACH_CSV = BASE_DIR / "Sim1V2.csv"

TS = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_PREFIX = f"LORK_{TS}"

# ------------------------------
# Site / launch — URRG Potter Launch Field
# 4272 State Route 364, Penn Yan, NY 14527
# 42 deg 41'53" N, 77 deg 09'25" W
# ------------------------------
LAT_LAUNCH  = 42.698056    # 42 deg 41'53" N
LON_LAUNCH  = -77.157222   # 77 deg 09'25" W
ELEV_LAUNCH = 240.0        # meters MSL (Finger Lakes basin near Penn Yan)
RAIL_LENGTH = 4.877        # meters

# Altitudes (m AGL) sampled and logged for every single simulation run.
# Brackets: rail exit (~30m), max-Q (~200m), burnout (~500m),
#           mid-ascent (~1000m), near-apogee (~2000m), full apogee coverage.
WIND_LOG_ALTS_M = [10, 50, 100, 200, 500, 800, 1000, 1500, 2000, 2500]

# ------------------------------
# Rocket geometry & mass
# ------------------------------
ROCKET_NAME = "Lykos"
LYKOS_LENGTH = 2.40
LYKOS_RADIUS = 0.103 / 2.0
LYKOS_DIAM = 2 * LYKOS_RADIUS

LYKOS_DRY_MASS = 6.818
LYKOS_CG_FROM_NOSE = 1.54

# OpenRocket reference (info only)
OPENROCKET_CP_FROM_NOSE = 1.83
OPENROCKET_SM_CAL = (OPENROCKET_CP_FROM_NOSE - LYKOS_CG_FROM_NOSE) / LYKOS_DIAM

# ------------------------------
# Parachute config (user inputs)
# ------------------------------
PARACHUTES = {
    "main":   {"diameter_m": 1.62,  "cd": 2.2, "mass_kg": 0.274, "deploy_altitude_m": 274.0},
    "drogue": {"diameter_m": 0.914, "cd": 2.2, "mass_kg": 0.126},  # apogee trigger
}

# ------------------------------
# Grids / sweeps
# ------------------------------
RAIL_INCLINATIONS = [0, 1, 2, 3, 4, 5]       # degrees FROM VERTICAL (user-facing)
RAIL_HEADINGS = [0, 90, 180, 270]             # degrees (0=N, 90=E, 180=S, 270=W)
N_WIND_PROFILES = 4
TURBULENCE_INTENSITIES = [0.01, 0.20, 0.25, 0.30]  # fraction of mean wind

# Turbulence params
TI_CORR_LEN_M = 150.0     # correlation length along altitude (m)
TI_MIN_REF_WIND = 1.5     # m/s minimum ref speed when mean wind is calm

# ------------------------------
# Helper: inertia via parallel-axis (rigid cylinder) (we love calcuating inertia in code)
# ------------------------------
def compute_inertia_parallel_axis(mass_kg: float, length_m: float, radius_m: float, cg_from_nose_m: float) -> Tuple[float, float, float]:
    m = float(mass_kg)
    L = float(length_m)
    R = float(radius_m)
    d = abs(float(cg_from_nose_m) - (L / 2.0))
    Ixx = m * (R ** 2)
    Iyy_cm = (1.0 / 12.0) * m * (3.0 * R ** 2 + L ** 2)
    Iyy = Iyy_cm + m * d ** 2
    Izz = Iyy
    return (Ixx, Iyy, Izz)

LYKOS_INERTIA = compute_inertia_parallel_axis(LYKOS_DRY_MASS, LYKOS_LENGTH, LYKOS_RADIUS, LYKOS_CG_FROM_NOSE)
logger.info("Inertia (physics-based): Ixx=%.4f, Iyy=%.4f, Izz=%.4f kg*m^2", *LYKOS_INERTIA)
logger.info("OpenRocket reference: CP=%.3f m, SM=%.3f cal", OPENROCKET_CP_FROM_NOSE, OPENROCKET_SM_CAL)

# ------------------------------
# Cd vs Mach table (.csv is called Sim1V2.csv)
# ------------------------------
def load_cd_mach_curve_from_sim_csv(csv_path: Path, stop_at_event="RECOVERY_DEVICE_DEPLOYMENT", max_points: int = 12) -> List[List[float]]:
    import re
    if not csv_path.exists():
        return [
            [0.00, 0.65], [0.10, 0.55], [0.20, 0.48],
            [0.40, 0.44], [0.60, 0.43], [0.80, 0.42],
        ]
    pairs = []
    with open(str(csv_path), "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if line.startswith("# Event") and stop_at_event in line:
                break
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            nums = []
            for p in parts:
                if re.match(r"^[\-+]?\d*\.?\d+(?:[eE][\-+]?\d+)?$", p):
                    nums.append(float(p))
            if len(nums) >= 2:
                cd, mach = nums[0], nums[1]
                if 0.0 <= mach <= 0.8 and 0.05 <= cd <= 2.0:
                    pairs.append((mach, cd))
    if not pairs:
        logger.warning("No usable Cd vs Mach pairs in %s; using defaults.", csv_path)
        return [
            [0.00, 0.65], [0.10, 0.55], [0.20, 0.48],
            [0.40, 0.44], [0.60, 0.43], [0.80, 0.42],
        ]
    pairs = sorted(pairs, key=lambda x: x[0])
    m_vals = np.array([p[0] for p in pairs], dtype=float)
    c_vals = np.array([p[1] for p in pairs], dtype=float)
    m_min, m_max = float(m_vals.min()), float(m_vals.max())
    nbins = min(max_points, max(3, int(np.ceil((m_max - m_min) / 0.08))))
    bins = np.linspace(m_min, m_max, nbins + 1)
    cd_curve = []
    inds = np.digitize(m_vals, bins)
    for i in range(1, len(bins) + 1):
        sel = (inds == i)
        if np.any(sel):
            m_med = float(np.median(m_vals[sel]))
            c_med = float(np.median(c_vals[sel]))
            cd_curve.append([round(m_med, 3), round(c_med, 3)])
    seen, cleaned = set(), []
    for m, c in sorted(cd_curve, key=lambda x: x[0]):
        if m in seen:
            continue
        seen.add(m)
        cleaned.append([m, c])
    if len(cleaned) < 3:
        uniq_m = sorted(set(m_vals.tolist()))
        picks = [uniq_m[0], uniq_m[len(uniq_m)//2], uniq_m[-1]]
        cleaned = []
        for target in picks:
            idx = int(np.argmin(np.abs(m_vals - target)))
            cleaned.append([round(float(m_vals[idx]), 3), round(float(c_vals[idx]), 3)])
    logger.info("Cd(M) points: %d, Mach range [%.3f..%.3f]", len(cleaned), cleaned[0][0], cleaned[-1][0])
    return cleaned

_CD_CACHE: Optional[List[List[float]]] = None
_CP_SM_LOGGED = False

# ------------------------------
# Wind profiles — Open-Meteo live fetch
# ------------------------------
# Altitude AGL mapping for each variable (adjusted for ELEV_LAUNCH).
# Open-Meteo surface heights are above ground; pressure-level altitudes
# are standard-atmosphere MSL values converted to AGL below.
_SURFACE_HEIGHTS_M = [10, 80, 120, 180]  # m AGL from Open-Meteo
_PRESSURE_LEVEL_TAGS = [                  # (hPa_label, MSL_alt_m approx)
    ("1000hPa",  111), ("975hPa",  320), ("950hPa",  540),
    ("925hPa",   770), ("900hPa", 1000), ("850hPa", 1457),
    ("800hPa",  1949), ("700hPa", 3012),
]

def _dir_to_uv(speed_ms: float, from_deg: float) -> Tuple[float, float]:
    """Convert meteorological wind (FROM direction, degrees) to (u_east, v_north) m/s."""
    r = math.radians(from_deg)
    return (-speed_ms * math.sin(r), -speed_ms * math.cos(r))

def fetch_wind_profiles_openmeteo(lat: float, lon: float,
                                   n_profiles: int = 4,
                                   elev_launch_m: float = ELEV_LAUNCH
                                   ) -> Optional[List[Dict]]:
    """
    Fetch N distinct hourly wind profiles from Open-Meteo (no API key needed).
    Returns a list of dicts, each with 'alts_agl', 'u_mps', 'v_mps', 'label'.
    Returns None on failure (caller falls back to synthetic profiles).
    """
    import urllib.request, urllib.error, json

    surface_vars = ",".join(
        f"wind_speed_{h}m,wind_direction_{h}m" for h in _SURFACE_HEIGHTS_M
    )
    pressure_vars = ",".join(
        f"wind_speed_{tag},wind_direction_{tag}" for tag, _ in _PRESSURE_LEVEL_TAGS
    )
    url = (
        f"https://api.open-meteo.com/v1/forecast"
        f"?latitude={lat}&longitude={lon}"
        f"&hourly={surface_vars},{pressure_vars}"
        f"&wind_speed_unit=ms&forecast_days=2&timezone=auto"
    )

    try:
        with urllib.request.urlopen(url, timeout=15) as resp:
            data = json.loads(resp.read().decode())
    except Exception as e:
        logger.warning("Open-Meteo fetch failed: %s", e)
        return None

    hourly = data.get("hourly", {})
    n_hours = len(hourly.get("time", []))
    if n_hours == 0:
        logger.warning("Open-Meteo returned empty hourly block.")
        return None

    # Pick n_profiles hours spread evenly across the 48h window
    if n_profiles <= 1:
        hour_indices = [0]
    else:
        hour_indices = [int(round(i * (n_hours - 1) / (n_profiles - 1))) for i in range(n_profiles)]

    profiles = []
    for hi in hour_indices:
        alts_agl, u_vals, v_vals = [], [], []

        # Surface heights (already AGL)
        for h in _SURFACE_HEIGHTS_M:
            sk = f"wind_speed_{h}m"
            dk = f"wind_direction_{h}m"
            if sk in hourly and dk in hourly:
                spd = hourly[sk][hi]
                drn = hourly[dk][hi]
                if spd is not None and drn is not None:
                    u, v = _dir_to_uv(float(spd), float(drn))
                    alts_agl.append(float(h))
                    u_vals.append(u); v_vals.append(v)

        # Pressure-level heights (MSL -> AGL)
        for tag, msl_m in _PRESSURE_LEVEL_TAGS:
            sk = f"wind_speed_{tag}"
            dk = f"wind_direction_{tag}"
            if sk in hourly and dk in hourly:
                spd = hourly[sk][hi]
                drn = hourly[dk][hi]
                if spd is not None and drn is not None:
                    agl = float(msl_m) - elev_launch_m
                    if agl > 0:
                        u, v = _dir_to_uv(float(spd), float(drn))
                        alts_agl.append(agl)
                        u_vals.append(u); v_vals.append(v)

        if len(alts_agl) < 2:
            logger.warning("Profile hour=%d has too few altitude points, skipping.", hi)
            continue

        # Sort by altitude
        order = sorted(range(len(alts_agl)), key=lambda i: alts_agl[i])
        alts_agl = [alts_agl[i] for i in order]
        u_vals   = [u_vals[i]   for i in order]
        v_vals   = [v_vals[i]   for i in order]

        spd10 = math.hypot(u_vals[0], v_vals[0])
        dir10 = (math.degrees(math.atan2(-u_vals[0], -v_vals[0])) + 360) % 360
        label = hourly["time"][hi]

        # Log full multi-level table for this profile
        logger.info("Open-Meteo profile %d @ %s — %d altitude levels:", len(profiles), label, len(alts_agl))
        for a, u, v in zip(alts_agl, u_vals, v_vals):
            spd = math.hypot(u, v)
            frm = (math.degrees(math.atan2(-u, -v)) + 360) % 360
            logger.info("  AGL %6.0fm  %5.2f m/s  FROM %5.1f deg  (u=%.3f  v=%.3f)", a, spd, frm, u, v)
        profiles.append({
            "label":    label,
            "alts_agl": alts_agl,
            "u_mps":    u_vals,
            "v_mps":    v_vals,
        })

    if len(profiles) == 0:
        return None
    logger.info("Fetched %d wind profile(s) from Open-Meteo.", len(profiles))
    return profiles


def _make_synthetic_profiles(n: int = 4) -> List[Dict]:
    """
    Fallback wind profiles for upstate NY (Finger Lakes basin, ~240m MSL).
    Each profile has physically motivated multi-level shear AND directional veer
    so that wind at 10m, 500m, 1500m, and 2500m all differ in speed AND angle.

    Upstate NY typical patterns:
      - Low levels: often NW or SW surface flow
      - Mid-levels (500-1000m): tends to back toward W/SW (Ekman layer)
      - Upper levels (1500-2500m): backs further, often W or WSW with the jet
    Veer and shear values are tuned so the NET drift on a ~45s descent
    under main parachute is roughly 100-400m from the pad — realistic for
    an L-motor flight to ~2400m apogee.
    """
    # Each tuple: (label, nodes)
    # Each node: (alt_m_AGL, speed_m_s, FROM_direction_deg)
    # Altitudes must span from surface to above apogee (2500m+)
    scenarios = [
        (
            "Light WNW — calm morning inversion",
            [
                (10,    2.5,  290),   # WNW at surface — often trapped below inversion
                (50,    3.2,  285),
                (100,   4.1,  280),
                (200,   5.3,  275),   # W at inversion top
                (500,   7.0,  268),   # slips to slightly S of W
                (800,   8.5,  263),
                (1000,  9.2,  258),   # WSW, jet shear beginning
                (1500, 11.0,  252),
                (2000, 12.5,  248),   # SSW of W at altitude
                (2500, 13.0,  245),
            ],
        ),
        (
            "Moderate SW — warm-sector afternoon",
            [
                (10,    5.5,  220),   # SSW at surface (warm sector)
                (50,    6.8,  228),
                (100,   8.0,  235),
                (200,   9.5,  242),   # veering SW with height
                (500,  12.0,  255),   # W-SW, heating mixes BL
                (800,  13.5,  260),
                (1000, 14.5,  263),
                (1500, 16.0,  268),   # nearly due W
                (2000, 17.5,  272),
                (2500, 18.0,  275),   # slight WNW at jet level
            ],
        ),
        (
            "Strong W — frontal passage / fast upper winds",
            [
                (10,    7.0,  260),   # W at surface
                (50,    9.0,  258),
                (100,  11.0,  255),
                (200,  13.5,  253),
                (500,  17.0,  250),   # steepening shear
                (800,  19.5,  247),
                (1000, 21.0,  245),
                (1500, 24.0,  242),   # WSW with fast jet
                (2000, 26.5,  240),
                (2500, 28.0,  238),
            ],
        ),
        (
            "Gusty NNW — post-frontal cold air",
            [
                (10,    6.0,  340),   # NNW at surface (cold air behind front)
                (50,    7.5,  335),
                (100,   9.0,  328),
                (200,  11.0,  320),   # backing toward NW with height
                (500,  13.5,  310),   # NW
                (800,  15.0,  300),
                (1000, 16.5,  292),   # WNW — cold trough aloft
                (1500, 18.0,  282),
                (2000, 19.5,  275),   # nearly due W at altitude
                (2500, 20.5,  270),
            ],
        ),
    ]

    alts_base = [node[0] for node in scenarios[0][1]]
    profiles = []
    for i in range(n):
        label, nodes = scenarios[i % len(scenarios)]
        alts_agl, u_list, v_list = [], [], []
        for alt, spd, from_deg in nodes:
            u, v = _dir_to_uv(spd, from_deg)
            alts_agl.append(float(alt))
            u_list.append(u); v_list.append(v)

        # Log the profile level-by-level so it's clear this is multi-level
        logger.info("Synthetic profile %d: '%s'", i, label)
        for alt, spd, from_deg in nodes:
            logger.info("  AGL %5.0fm  ->  %5.1f m/s  FROM %3.0f deg", alt, spd, from_deg)

        profiles.append({
            "label":    label,
            "alts_agl": alts_agl,
            "u_mps":    u_list,
            "v_mps":    v_list,
        })
    return profiles


def make_profile_fns(profile: Dict) -> Tuple[Any, Any]:
    """Convert a profile dict to separate (u_fn, v_fn) callables: altitude_m -> m/s."""
    alts = np.array(profile["alts_agl"], dtype=float)
    u_arr = np.array(profile["u_mps"],    dtype=float)
    v_arr = np.array(profile["v_mps"],    dtype=float)

    def u_fn(alt_m):
        return float(np.interp(float(alt_m), alts, u_arr,
                               left=float(u_arr[0]), right=float(u_arr[-1])))

    def v_fn(alt_m):
        return float(np.interp(float(alt_m), alts, v_arr,
                               left=float(v_arr[0]), right=float(v_arr[-1])))

    return u_fn, v_fn


def _wind_table_str(u_fn, v_fn,
                    alts: List[float] = WIND_LOG_ALTS_M,
                    label: str = "") -> str:
    """
    Build a compact multi-level wind table string for per-run logging.
    Each row: AGL altitude, speed m/s, FROM direction deg, u-component, v-component.
    This is logged at INFO level for every simulation run so the CSV and console
    both carry a full multi-level snapshot of what wind each sim actually saw.
    """
    header = f"  {'AGL(m)':>7}  {'Spd(m/s)':>9}  {'From(deg)':>10}  {'U_east':>8}  {'V_north':>8}"
    lines = [f"  Wind profile{' — ' + label if label else ''}:", header]
    for alt in alts:
        u = u_fn(float(alt))
        v = v_fn(float(alt))
        spd = math.hypot(u, v)
        frm = (math.degrees(math.atan2(-u, -v)) + 360) % 360
        lines.append(f"  {alt:>7.0f}  {spd:>9.2f}  {frm:>10.1f}  {u:>8.3f}  {v:>8.3f}")
    return "\n".join(lines)


def _wind_csv_cols(u_fn, v_fn,
                   sample_alts: Tuple[float, ...] = (10, 200, 500, 1000, 2000)) -> Dict[str, float]:
    """
    Return flat dict of wind speed/dir columns at sample altitudes for the CSV.
    Keeps the CSV wide but reviewable without reading log files.
    Column names: Wind_Spd_10m, Wind_Dir_10m, Wind_Spd_200m, ... etc.
    """
    cols: Dict[str, float] = {}
    for alt in sample_alts:
        u = u_fn(float(alt)); v = v_fn(float(alt))
        spd = math.hypot(u, v)
        frm = (math.degrees(math.atan2(-u, -v)) + 360) % 360
        cols[f"Wind_Spd_{int(alt)}m"] = round(spd, 3)
        cols[f"Wind_Dir_{int(alt)}m"] = round(frm, 1)
    return cols


def make_wind_fn_pair(base_u_fn, base_v_fn, ti_frac: float, seed: int,
                      corr_len_m: float = TI_CORR_LEN_M,
                      min_ref_wind: float = TI_MIN_REF_WIND) -> Tuple[Any, Any]:
    """
    Returns (u_fn, v_fn) each callable: altitude_m -> m/s.
    Adds altitude-correlated turbulent perturbations on top of the mean wind.
    These are the functions passed directly to RocketPy's set_atmospheric_model.

    FIX vs previous version: RocketPy requires SEPARATE u and v callables.
    The old code returned a single wrapper returning a tuple, which RocketPy
    silently ignored — hence zero wind effect in all simulations.
    """
    rng = np.random.RandomState(int(seed) & 0x7FFFFFFF)
    z_max = 5000.0
    n_bins = max(2, int(math.ceil(z_max / corr_len_m)) + 1)
    noise_u = rng.normal(0.0, 1.0, size=n_bins)
    noise_v = rng.normal(0.0, 1.0, size=n_bins)

    def _lerp_noise(alt_m: float, noise: np.ndarray) -> float:
        z = float(max(0.0, min(alt_m, z_max)))
        idx = z / corr_len_m
        i0 = int(math.floor(idx))
        i1 = min(i0 + 1, n_bins - 1)
        frac = idx - i0
        return (1.0 - frac) * noise[i0] + frac * noise[i1]

    def _sigma(alt_m: float) -> float:
        if ti_frac is None or ti_frac <= 0.0:
            return 0.0
        u0 = base_u_fn(alt_m) if base_u_fn is not None else 0.0
        v0 = base_v_fn(alt_m) if base_v_fn is not None else 0.0
        vmag = math.hypot(u0, v0)
        return ti_frac * max(vmag, float(min_ref_wind))

    def u_fn(alt_m: float) -> float:
        mean = base_u_fn(alt_m) if base_u_fn is not None else 0.0
        return float(mean + _sigma(alt_m) * _lerp_noise(alt_m, noise_u))

    def v_fn(alt_m: float) -> float:
        mean = base_v_fn(alt_m) if base_v_fn is not None else 0.0
        return float(mean + _sigma(alt_m) * _lerp_noise(alt_m, noise_v))

    return u_fn, v_fn

# ------------------------------
# Rocket builder
# FIX: Removed the `quiet` parameter and its temporary setLevel() mutation.
# _CP_SM_LOGGED and _CD_CACHE already ensure verbose messages only fire once,
# so suppressing the logger level mid-run was both unnecessary and risky
# (it could silence legitimate warnings during grid runs).
# ------------------------------
def build_lykos() -> RocketPy:
    global _CD_CACHE, _CP_SM_LOGGED

    # One-time Cd(M) cache — logged only on first call
    if _CD_CACHE is None:
        _CD_CACHE = load_cd_mach_curve_from_sim_csv(CD_MACH_CSV, max_points=12)
        logger.info("Using Cd(M) from %s (or defaults).", CD_MACH_CSV)

    power_off_drag = _CD_CACHE
    power_on_drag = _CD_CACHE

    rocket = None
    last_err = None
    try:
        rocket = RocketPy(
            radius=LYKOS_RADIUS, mass=LYKOS_DRY_MASS, inertia=LYKOS_INERTIA,
            center_of_mass_position=LYKOS_CG_FROM_NOSE,
            power_off_drag=power_off_drag, power_on_drag=power_on_drag,
            coordinate_system_orientation="nose_to_tail",
        )
    except TypeError as e:
        last_err = e
    if rocket is None:
        try:
            rocket = RocketPy(
                radius=LYKOS_RADIUS, mass=LYKOS_DRY_MASS, inertia=LYKOS_INERTIA,
                center_of_mass_without_motor=LYKOS_CG_FROM_NOSE,
                power_off_drag=power_off_drag, power_on_drag=power_on_drag,
                coordinate_system_orientation="nose_to_tail",
            )
        except TypeError as e:
            last_err = e
    if rocket is None:
        try:
            rocket = RocketPy(
                LYKOS_RADIUS, LYKOS_DRY_MASS, LYKOS_INERTIA,
                power_off_drag, power_on_drag, LYKOS_CG_FROM_NOSE,
                coordinate_system_orientation="nose_to_tail",
            )
        except TypeError as e:
            last_err = e
    if rocket is None:
        raise TypeError("RocketPy constructor mismatch. Last error: %s" % (last_err,))

    rocket.add_nose(length=0.5, kind="ogive", position=0.0)
    rocket.add_trapezoidal_fins(n=3, root_chord=0.2, tip_chord=0.1, span=0.12, position=LYKOS_LENGTH - 0.3)

    MOTOR_POSITION_FROM_NOSE = 1.90
    if not (0.0 <= MOTOR_POSITION_FROM_NOSE <= LYKOS_LENGTH):
        raise ValueError("Motor position outside rocket body.")
    motor = SolidMotor(
        thrust_source=str(ENG_FILE),
        dry_mass=1.66,
        dry_inertia=(0.08, 0.08, 0.0011),
        center_of_dry_mass_position=0.24,
        grains_center_of_mass_position=0.29,
        burn_time=3.6,
        grain_number=3,
        grain_separation=0.005,
        grain_density=1815,
        grain_outer_radius=0.033,
        grain_initial_inner_radius=0.015,
        grain_initial_height=0.12,
        nozzle_radius=0.033,
        throat_radius=0.011,
        interpolation_method="linear",
        nozzle_position=0.0,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )
    rocket.add_motor(motor, MOTOR_POSITION_FROM_NOSE)

    # CP/SM diagnostic logged once only
    if not _CP_SM_LOGGED:
        try:
            cp_rp = rocket.cp_position(0)
            sm_rp = (cp_rp - LYKOS_CG_FROM_NOSE) / LYKOS_DIAM
            logger.info("RocketPy CP=%.3f m; SM=%.3f cal (OpenRocket CP=%.3f; SM=%.3f)",
                        cp_rp, sm_rp, OPENROCKET_CP_FROM_NOSE, OPENROCKET_SM_CAL)
        except Exception as e:
            logger.warning("CP/SM diagnostic failed: %s", e)
        _CP_SM_LOGGED = True

    return rocket

# ------------------------------
# Environment builder
# FIX: RocketPy requires wind to be set via set_atmospheric_model(type="custom_atmosphere",
# wind_u=callable, wind_v=callable) where each callable takes altitude (m) and returns a scalar.
# The previous approach of passing a single tuple-returning wrapper to set_wind() / set_wind_velocity()
# was silently ignored by RocketPy — causing zero wind effect in every simulation.
# ------------------------------
def build_environment(wind_u_fn=None, wind_v_fn=None) -> Environment:
    env = Environment(latitude=LAT_LAUNCH, longitude=LON_LAUNCH, elevation=ELEV_LAUNCH)
    try:
        env.set_rail_length(RAIL_LENGTH)
    except Exception:
        try:
            env.set_rail(RAIL_LENGTH)
        except Exception:
            env.rail_length = RAIL_LENGTH

    if wind_u_fn is not None or wind_v_fn is not None:
        u = wind_u_fn if wind_u_fn is not None else (lambda alt: 0.0)
        v = wind_v_fn if wind_v_fn is not None else (lambda alt: 0.0)
        hooked = False
        # Primary: correct RocketPy API
        for kwargs in (
            {"type": "custom_atmosphere", "wind_u": u, "wind_v": v},
            {"type": "CustomAtmosphere",  "wind_u": u, "wind_v": v},
        ):
            try:
                env.set_atmospheric_model(**kwargs)
                hooked = True
                break
            except Exception:
                pass
        # Fallback: older RocketPy API names
        if not hooked:
            for setter in ("set_wind_velocity_x_y", "set_wind"):
                try:
                    getattr(env, setter)(u, v)
                    hooked = True
                    break
                except Exception:
                    pass
        if not hooked:
            logger.warning("Could not hook wind into Environment — all sims will have no wind.")
    return env

# ------------------------------
# Inclination convention
# ------------------------------
def create_flight(rocket_obj, env, incl_deg, heading_deg):
    RL = getattr(env, "rail_length", None)
    if RL is None or RL <= 0:
        RL = RAIL_LENGTH
    try:
        setattr(env, "rail_length", RL)
    except Exception:
        pass
    try:
        return Flight(rocket=rocket_obj, environment=env, rail_length=RL, inclination=incl_deg, heading=heading_deg)
    except Exception:
        pass
    try:
        return Flight(rocket_obj, env, RL, incl_deg, heading_deg)
    except Exception:
        pass
    return Flight(rocket=rocket_obj, environment=env, inclination=incl_deg, heading=heading_deg)

def detect_inclination_convention(rocket_obj) -> str:
    try:
        env_probe = build_environment()
        f0 = create_flight(rocket_obj, env_probe, incl_deg=0, heading_deg=0)
        f90 = create_flight(rocket_obj, env_probe, incl_deg=90, heading_deg=0)
        a0 = float(getattr(f0, "apogee", 0) or 0)
        a90 = float(getattr(f90, "apogee", 0) or 0)
        if a90 > a0 + 50.0:
            logger.info("Inclination API is 'from_horizontal' (0=horizontal, 90=vertical).")
            return "from_horizontal"
        else:
            logger.info("Inclination API is 'from_vertical' (0=vertical, 90=horizontal).")
            return "from_vertical"
    except Exception as e:
        logger.warning("Inclination detection failed (%s); defaulting to 'from_vertical'.", e)
        return "from_vertical"

def convert_incl_for_api(user_deg_from_vertical: float, api_convention: str) -> float:
    tilt = float(user_deg_from_vertical)
    if api_convention == "from_horizontal":
        return max(0.0, min(90.0, 90.0 - tilt))
    return max(0.0, min(90.0, tilt))

# ------------------------------
# Landing lat/lon with heading rotation
# ------------------------------
def flight_landing_latlon(flight, heading_deg: Optional[float] = None) -> Tuple[Optional[float], Optional[float], str]:
    for lat_name, lon_name in (("lat_impact", "lon_impact"),
                               ("latitude_impact", "longitude_impact"),
                               ("impact_latitude", "impact_longitude")):
        if hasattr(flight, lat_name) and hasattr(flight, lon_name):
            try:
                return float(getattr(flight, lat_name)), float(getattr(flight, lon_name)), "direct"
            except Exception:
                pass

    if not hasattr(flight, "x_impact") or not hasattr(flight, "y_impact"):
        return None, None, "unavailable"
    try:
        x = float(getattr(flight, "x_impact"))  # downrange (launch frame)
        y = float(getattr(flight, "y_impact"))  # cross-range (right positive)
    except Exception:
        return None, None, "unavailable"

    east_m, north_m = x, y
    if heading_deg is not None:
        th = math.radians(float(heading_deg) % 360.0)
        north_m = x * math.cos(th) - y * math.sin(th)
        east_m  = x * math.sin(th) + y * math.cos(th)

    lat_offset = north_m / 111_111.0
    lon_scale = 111_111.0 * math.cos(math.radians(LAT_LAUNCH))
    lon_offset = (east_m / lon_scale) if lon_scale != 0 else 0.0
    return LAT_LAUNCH + lat_offset, LON_LAUNCH + lon_offset, "converted"

# ------------------------------
# Parachute helpers
# ------------------------------
def get_parachute_params(side: str) -> Dict[str, Optional[float]]:
    data = PARACHUTES.get(side, {}) if isinstance(PARACHUTES, dict) else {}
    def pick(keys):
        for k in keys:
            if k in data:
                try:
                    return float(data[k])
                except Exception:
                    pass
        return None
    return {
        "diameter_m":        pick(("diameter_m", "diameter")),
        "cd":                pick(("cd", "Cd", "CD")),
        "mass_kg":           pick(("mass_kg", "mass")),
        "deploy_altitude_m": pick(("deploy_altitude_m", "altitude")),
    }

def cds_from(diameter_m: Optional[float], Cd: Optional[float]) -> Optional[float]:
    if diameter_m and (Cd or 0.0) > 0.0:
        area = math.pi * (diameter_m / 2.0) ** 2
        return (Cd or 2.2) * area
    return None

def add_parachute_robust(rocket, *, name, cd_s, kind, main_alt=None) -> bool:
    """
    kind in: 'drogue_apogee', 'main_apogee', 'main_altitude'
    Tries multiple RocketPy API spellings.
    """
    if cd_s is None:
        return False
    attempts: List[Dict[str, Any]] = []
    if kind in ("drogue_apogee", "main_apogee"):
        attempts += [
            dict(name=name, cd_s=cd_s, trigger="apogee", sampling_rate=100,
                 lag=1.0 if "drogue" in kind else 0.0, noise=(0, 8.3, 0.5)),
            dict(name=name, cd_s=cd_s, trigger="apogee", sampling_rate=100),
        ]
    elif kind == "main_altitude":
        attempts += [
            dict(name=name, cd_s=cd_s, trigger=main_alt, sampling_rate=100),
            dict(name=name, cd_s=cd_s, trigger="altitude", altitude=main_alt, sampling_rate=100),
        ]
    for kw in attempts:
        try:
            rocket.add_parachute(**kw)
            return True
        except Exception:
            continue
    return False

# ------------------------------
# Runners
# ------------------------------
def run_single_sim(config_key: str, ti_frac: float, user_tilt_deg: float, heading_deg: float,
                   wind_profiles: List[Dict], incl_convention: str,
                   profile_index: int = 0) -> Dict[str, Any]:
    profile = wind_profiles[profile_index % len(wind_profiles)]
    base_u, base_v = make_profile_fns(profile)
    seed = 12345
    u_fn, v_fn = make_wind_fn_pair(base_u, base_v, ti_frac=ti_frac, seed=seed)
    env = build_environment(u_fn, v_fn)

    # --- Per-sim wind table (full multi-level) ---
    logger.info("=" * 65)
    logger.info("SINGLE SIM — config=%s  tilt=%.1fdeg  heading=%.0fdeg  TI=%.0f%%",
                config_key, user_tilt_deg, heading_deg, 100 * ti_frac)
    logger.info(
        _wind_table_str(u_fn, v_fn, WIND_LOG_ALTS_M, label=profile.get("label", ""))
    )
    logger.info("=" * 65)

    rocket = build_lykos()

    drogue_created = False
    main_created = False
    if config_key != "ballistic":
        drogue_p = get_parachute_params("drogue")
        main_p   = get_parachute_params("main")
        main_alt = main_p["deploy_altitude_m"] if main_p["deploy_altitude_m"] is not None else 274.0
        d_cds = cds_from(drogue_p["diameter_m"], drogue_p["cd"])
        m_cds = cds_from(main_p["diameter_m"],   main_p["cd"])

        if config_key in ("nominal", "drogue_only") and d_cds is not None:
            drogue_created = add_parachute_robust(rocket, name="Drogue", cd_s=d_cds, kind="drogue_apogee")
        if config_key in ("nominal", "main_at_apogee") and m_cds is not None:
            kind = "main_apogee" if config_key == "main_at_apogee" else "main_altitude"
            main_created = add_parachute_robust(rocket, name="Main", cd_s=m_cds, kind=kind, main_alt=main_alt)
        if config_key == "nominal" and not (drogue_created and main_created):
            logger.warning("Single: 'nominal' but one/both parachutes missing (ballistic).")

    incl = convert_incl_for_api(user_tilt_deg, incl_convention)
    flight = create_flight(rocket, env, incl, heading_deg)

    landing_lat, landing_lon, _ = flight_landing_latlon(flight, heading_deg=heading_deg)

    v_rail = None
    for cand in ("out_of_rail_velocity", "v_rail"):
        if hasattr(flight, cand):
            v_rail = getattr(flight, cand, None)
            break
    if v_rail is not None and v_rail < 18.0:
        logger.warning("LOW off-rail velocity (%.1f m/s). Expect higher wind sensitivity.", v_rail)

    v_imp = None
    for cand in ("impact_speed", "v_impact", "impact_velocity"):
        if hasattr(flight, cand):
            try:
                v_imp = float(getattr(flight, cand))
            except Exception:
                pass
            break

    dN_m = dE_m = Range_m = Bearing_deg = None
    if landing_lat is not None and landing_lon is not None:
        dN_m = (landing_lat - LAT_LAUNCH) * 111_111.0
        dE_m = (landing_lon - LON_LAUNCH) * (111_111.0 * math.cos(math.radians(LAT_LAUNCH)))
        Range_m = (dN_m ** 2 + dE_m ** 2) ** 0.5
        Bearing_deg = (math.degrees(math.atan2(dE_m, dN_m)) + 360.0) % 360.0

    # Surface wind diagnostic (10m AGL)
    surf_u = u_fn(10.0); surf_v = v_fn(10.0)
    surf_spd = math.hypot(surf_u, surf_v)
    surf_dir = (math.degrees(math.atan2(-surf_u, -surf_v)) + 360) % 360

    # Multi-altitude wind columns for CSV
    wind_cols = _wind_csv_cols(u_fn, v_fn)

    result = {
        "Config": config_key,
        "Wind_Profile": profile_index,
        "Wind_Label": profile.get("label", ""),
        "Wind_Speed_10m_mps": round(surf_spd, 2),
        "Wind_Dir_10m_deg": round(surf_dir, 1),
        **wind_cols,
        "TI": ti_frac,
        "Rail_Incl_deg": user_tilt_deg,
        "Rail_Heading_deg": heading_deg,
        "Apogee_m": getattr(flight, "apogee", None),
        "Max_Speed_mps": getattr(flight, "max_speed", None) or getattr(flight, "max_velocity", None),
        "Rail_Exit_mps": v_rail,
        "Flight_Time_s": getattr(flight, "t_final", None),
        "Landing_Lat": landing_lat,
        "Landing_Lon": landing_lon,
        "Impact_Speed_mps": v_imp,
        "dN_m": dN_m,
        "dE_m": dE_m,
        "Range_m": Range_m,
        "Bearing_deg": Bearing_deg,
        "Drogue_Created": drogue_created,
        "Main_Created": main_created,
    }
    logger.info("Single sim result: %s", result)
    return result

def run_grid_monte_carlo(wind_profiles: List[Dict], incl_convention: str) -> pd.DataFrame:
    rows: List[Dict[str, Any]] = []
    n_wp = len(wind_profiles)
    total = len(RAIL_INCLINATIONS) * len(RAIL_HEADINGS) * n_wp * len(TURBULENCE_INTENSITIES) * 4
    sim_number = 0

    for cfg_key in ("ballistic", "main_at_apogee", "nominal", "drogue_only"):
        for user_tilt in RAIL_INCLINATIONS:
            for heading in RAIL_HEADINGS:
                for wp in range(n_wp):
                    for ti_frac in TURBULENCE_INTENSITIES:
                        sim_number += 1
                        try:
                            rocket = build_lykos()

                            drogue_created = main_created = False
                            if cfg_key != "ballistic":
                                drogue_p = get_parachute_params("drogue")
                                main_p   = get_parachute_params("main")
                                main_alt = main_p["deploy_altitude_m"] if main_p["deploy_altitude_m"] is not None else 274.0
                                d_cds = cds_from(drogue_p["diameter_m"], drogue_p["cd"])
                                m_cds = cds_from(main_p["diameter_m"],   main_p["cd"])

                                if cfg_key in ("nominal", "drogue_only") and d_cds is not None:
                                    drogue_created = add_parachute_robust(rocket, name="Drogue", cd_s=d_cds, kind="drogue_apogee")
                                if cfg_key in ("nominal", "main_at_apogee") and m_cds is not None:
                                    kind = "main_apogee" if cfg_key == "main_at_apogee" else "main_altitude"
                                    main_created = add_parachute_robust(rocket, name="Main", cd_s=m_cds, kind=kind, main_alt=main_alt)
                                if cfg_key == "nominal" and not (drogue_created and main_created):
                                    logger.warning("Sim %d: 'nominal' but parachute(s) not created — ballistic.", sim_number)

                            # Build wind: select profile, add turbulence with unique seed per sim
                            profile = wind_profiles[wp % n_wp]
                            base_u, base_v = make_profile_fns(profile)
                            # Seed is deterministic but unique per (config, tilt, heading, wp, ti)
                            seed = abs(hash((cfg_key, user_tilt, heading, wp, round(ti_frac, 4)))) % (2**31)
                            u_fn, v_fn = make_wind_fn_pair(base_u, base_v, ti_frac, seed)
                            env = build_environment(u_fn, v_fn)

                            # --- Per-run extensive wind logging (every single run) ---
                            logger.info("-" * 65)
                            logger.info(
                                "Run %d/%d | cfg=%-15s | tilt=%ddeg | head=%3ddeg | "
                                "wp=%d | TI=%.0f%%",
                                sim_number, total, cfg_key, user_tilt, heading, wp, 100*ti_frac,
                            )
                            logger.info(
                                _wind_table_str(u_fn, v_fn, WIND_LOG_ALTS_M,
                                                label=profile.get("label", ""))
                            )

                            incl = convert_incl_for_api(user_tilt, incl_convention)
                            flight = create_flight(rocket, env, incl, heading)

                            landing_lat, landing_lon, _ = flight_landing_latlon(flight, heading_deg=heading)

                            v_rail = None
                            for cand in ("out_of_rail_velocity", "v_rail"):
                                if hasattr(flight, cand):
                                    v_rail = getattr(flight, cand, None)
                                    break
                            if v_rail is not None and v_rail < 18.0:
                                logger.warning("Sim %d: low off-rail velocity (%.1f m/s).", sim_number, v_rail)

                            v_imp = None
                            for cand in ("impact_speed", "v_impact", "impact_velocity"):
                                if hasattr(flight, cand):
                                    try:
                                        v_imp = float(getattr(flight, cand))
                                    except Exception:
                                        pass
                                    break

                            dN_m = dE_m = Range_m = Bearing_deg = None
                            if landing_lat is not None and landing_lon is not None:
                                dN_m = (landing_lat - LAT_LAUNCH) * 111_111.0
                                dE_m = (landing_lon - LON_LAUNCH) * (111_111.0 * math.cos(math.radians(LAT_LAUNCH)))
                                Range_m = (dN_m ** 2 + dE_m ** 2) ** 0.5
                                Bearing_deg = (math.degrees(math.atan2(dE_m, dN_m)) + 360.0) % 360.0

                            # Surface wind and multi-altitude CSV columns
                            surf_u = u_fn(10.0); surf_v = v_fn(10.0)
                            surf_spd = math.hypot(surf_u, surf_v)
                            surf_dir = (math.degrees(math.atan2(-surf_u, -surf_v)) + 360) % 360
                            wind_cols = _wind_csv_cols(u_fn, v_fn)

                            # Post-flight result logging
                            logger.info(
                                "  RESULT: apogee=%.0fm  range=%.0fm  bearing=%.0fdeg  "
                                "v_impact=%.1fm/s  drogue=%s  main=%s",
                                getattr(flight, "apogee", 0) or 0,
                                Range_m or 0, Bearing_deg or 0,
                                v_imp or 0,
                                "OK" if drogue_created else "--",
                                "OK" if main_created else "--",
                            )

                            rows.append({
                                "Run": sim_number,
                                "Config": cfg_key,
                                "Wind_Profile": wp,
                                "Wind_Label": profile.get("label", ""),
                                "Wind_Speed_10m_mps": round(surf_spd, 2),
                                "Wind_Dir_10m_deg": round(surf_dir, 1),
                                **wind_cols,
                                "TI": ti_frac,
                                "Rail_Incl_deg": user_tilt,
                                "Rail_Heading_deg": heading,
                                "Apogee_m": getattr(flight, "apogee", None),
                                "Max_Speed_mps": getattr(flight, "max_speed", None) or getattr(flight, "max_velocity", None),
                                "Rail_Exit_mps": v_rail,
                                "Flight_Time_s": getattr(flight, "t_final", None),
                                "Landing_Lat": landing_lat,
                                "Landing_Lon": landing_lon,
                                "Drogue_Created": drogue_created,
                                "Main_Created": main_created,
                                "Impact_Speed_mps": v_imp,
                                "dN_m": dN_m,
                                "dE_m": dE_m,
                                "Range_m": Range_m,
                                "Bearing_deg": Bearing_deg,
                            })

                            if sim_number % 50 == 0 or sim_number in (1, total):
                                logger.info(
                                    ">>> Progress: %d/%d complete (%.0f%%)",
                                    sim_number, total, 100.0 * sim_number / total,
                                )
                        except Exception as e:
                            logger.error("Sim %d failed: %s", sim_number, e)

    return pd.DataFrame(rows)

# ------------------------------
# KMZ helpers
# ------------------------------
def read_template_kml_from_kmz(kmz_path: Path) -> Tuple[str, str]:
    with zipfile.ZipFile(str(kmz_path), "r") as zf:
        kml_names = [n for n in zf.namelist() if n.lower().endswith(".kml")]
        if not kml_names:
            raise FileNotFoundError("No .kml found inside template KMZ.")
        doc_name = kml_names[0]
        kml_text = zf.read(doc_name).decode("utf-8", "ignore")
        return (kml_text, doc_name)

def inject_landing_folder(kml_text: str, df: pd.DataFrame, folder_name: str) -> str:
    placemarks = []
    df2 = df.dropna(subset=["Landing_Lat", "Landing_Lon"])
    for _, r in df2.iterrows():
        run_id = int(r.get("Run", 0))
        cfg = str(r.get("Config", "unknown"))
        lon = float(r["Landing_Lon"]); lat = float(r["Landing_Lat"])
        placemarks.append(
            "  <Placemark>\n"
            f"    <name>Run {run_id} - {cfg}</name>\n"
            f"    <Point><coordinates>{lon},{lat},0</coordinates></Point>\n"
            "  </Placemark>"
        )
    folder_block = (
        "  <Folder>\n"
        f"    <name>{folder_name}</name>\n"
        f"{''.join(p + chr(10) for p in placemarks)}"
        "  </Folder>\n"
    )
    out = kml_text
    out = out.replace('<Style id="lorkPoint">', '<Style id="lorkPoint-removed">')
    out = out.replace("</Document>", f"{folder_block}</Document>")
    return out

def write_kmz_from_kml(kml_text: str, out_kmz_path: Path, doc_name: str):
    with zipfile.ZipFile(str(out_kmz_path), "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr(doc_name, kml_text)

# ------------------------------
# CLI + main
# ------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="LORK - Lykos Physics-True Runner")
    mode = p.add_mutually_exclusive_group(required=False)
    mode.add_argument("--single", action="store_true", help="Run a single simulation instead of the full grid")
    mode.add_argument("--grid",   action="store_true", help="Run full grid (all configs x sweeps) [default]")

    p.add_argument("--config", choices=("ballistic", "main_at_apogee", "nominal", "drogue_only"),
                   default="nominal", help="Flight configuration for --single (default: nominal)")
    p.add_argument("--incl",    type=float, default=3.0,  help="Rail tilt deg FROM VERTICAL for --single (default: 3)")
    p.add_argument("--heading", type=float, default=90.0, help="Rail heading deg for --single (default: 90=E)")
    p.add_argument("--ti",      type=float, default=0.20, help="Turbulence intensity fraction for --single (default: 0.20)")

    # Export flags — both on by default so a plain run produces files
    p.add_argument("--no-csv",  action="store_true", help="Suppress CSV export")
    p.add_argument("--no-kmz",  action="store_true", help="Suppress KMZ export")
    p.add_argument("--disable-wind", action="store_true", help="Disable mean wind (turbulence-only)")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()
    print("BOOTSTRAP: LORK.py starting")

    # Derive effective export flags (on by default; --no-csv / --no-kmz turn them off)
    export_csv = not args.no_csv
    export_kmz = not args.no_kmz

    # Load wind profiles — try Open-Meteo live fetch first, fall back to synthetic
    wind_profiles: List[Dict] = []
    if not args.disable_wind:
        logger.info("Fetching live wind profiles from Open-Meteo (lat=%.4f lon=%.4f)...",
                    LAT_LAUNCH, LON_LAUNCH)
        wind_profiles = fetch_wind_profiles_openmeteo(
            LAT_LAUNCH, LON_LAUNCH, n_profiles=N_WIND_PROFILES
        ) or []

    if len(wind_profiles) < N_WIND_PROFILES:
        if len(wind_profiles) == 0:
            logger.warning("=" * 60)
            logger.warning("Open-Meteo unavailable — using synthetic wind profiles.")
            logger.warning("Results will still have variation but not today's actual wind.")
            logger.warning("=" * 60)
        else:
            logger.warning("Only got %d profiles from Open-Meteo; padding with synthetic.",
                           len(wind_profiles))
        synthetic = _make_synthetic_profiles(N_WIND_PROFILES - len(wind_profiles))
        wind_profiles.extend(synthetic)

    logger.info("Wind profiles ready: %d", len(wind_profiles))
    for i, p in enumerate(wind_profiles):
        spd = math.hypot(p["u_mps"][0], p["v_mps"][0])
        drn = (math.degrees(math.atan2(-p["u_mps"][0], -p["v_mps"][0])) + 360) % 360
        logger.info("  Profile %d — %s — surface: %.1f m/s FROM %.0f deg", i, p["label"], spd, drn)

    # Inclination convention: RocketPy uses "from_horizontal" (0=horizontal, 90=vertical).
    # The probe-based auto-detection is unreliable because the probe rocket can hit
    # the ground before reaching meaningful apogee when launched at incl=0.
    # Hardcoded here after empirical confirmation (apogee==ELEV_LAUNCH at incl=0
    # confirmed that 0 means horizontal in this RocketPy installation).
    INCL_CONVENTION = "from_horizontal"
    logger.info("Inclination convention: from_horizontal (0=horizontal, 90=vertical). "
                "User inputs are degrees FROM VERTICAL and are converted automatically.")

    results_df = None
    # Default mode is grid; --single overrides
    if args.single:
        one = run_single_sim(
            config_key=args.config,
            ti_frac=args.ti,
            user_tilt_deg=args.incl,
            heading_deg=args.heading,
            wind_profiles=wind_profiles,
            incl_convention=INCL_CONVENTION,
        )
        results_df = pd.DataFrame([one])
        if export_csv:
            out_csv = BASE_DIR / f"{OUTPUT_PREFIX}_single_{args.config}.csv"
            results_df.to_csv(out_csv, index=False)
            logger.info("Wrote %s", out_csv)
    else:
        # Full grid (default when no mode flag given, or explicit --grid)
        df = run_grid_monte_carlo(wind_profiles, INCL_CONVENTION)
        results_df = df
        if export_csv:
            out_csv = BASE_DIR / f"{OUTPUT_PREFIX}_grid.csv"
            results_df.to_csv(out_csv, index=False)
            logger.info("Wrote %s (%d rows)", out_csv, len(df))

    if export_kmz and results_df is not None and results_df[["Landing_Lat", "Landing_Lon"]].notna().any().any():
        try:
            kml_text, doc_inside = read_template_kml_from_kmz(TEMPLATE_KMZ)
            updated_kml = inject_landing_folder(kml_text, results_df, folder_name=f"Lykos Landings {TS}")
            out_kmz = BASE_DIR / f"{OUTPUT_PREFIX}_landings.kmz"
            write_kmz_from_kml(updated_kml, out_kmz, doc_name=doc_inside)
            logger.info("KMZ written: %s (template: %s)", out_kmz, TEMPLATE_KMZ)
        except Exception as e:
            logger.error("KMZ export failed: %s", e)

    logger.info("Done.")