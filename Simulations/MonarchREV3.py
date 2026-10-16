#!/usr/bin/env python3
# Monarch 3" Monte Carlo DOE — RocketPy + ERA5


import csv
import math
import os
import queue
import threading
import time
import zipfile
import xml.etree.ElementTree as ET
import json
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, FIRST_COMPLETED, wait, TimeoutError as FutureTimeoutError

import numpy as np # type: ignore
import xarray as xr # type: ignore
from rocketpy import Environment, SolidMotor, Rocket, Flight # type: ignore
import rocketpy.rocket.aero_surface.aero_surface # pyright: ignore[reportMissingImports]


import sys
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    # Older Python or non-reconfigurable streams will just keep their defaults
    pass


def _sim_worker(geom, nominal_thrust, perturbations, config, wind_u, wind_v,
                 rail_length, inclination, heading, site_elev):
  
    env = Environment(latitude=SITE_LAT, longitude=SITE_LON, elevation=site_elev)
    env.set_atmospheric_model(
        type="custom_atmosphere", pressure=None, temperature=None,
        wind_u=list(wind_u), wind_v=list(wind_v),
    )

    rocket, motor, sim_params = build_rocket_and_motor(
        geom, nominal_thrust, perturbations, config=config)

    flight = Flight(
        rocket=rocket, environment=env, rail_length=rail_length,
        inclination=inclination, heading=heading,
        max_time=500, time_overshoot=True,
        max_time_step=0.02, min_time_step=1e-4,
        rtol=1e-6, atol=[1e-3] * 6 + [1e-6] * 4 + [1e-3] * 3,
        ode_solver="LSODA", verbose=False,
    )

    sol = np.array(flight.solution)
    t_arr, z_arr = sol[:, 0], sol[:, 3]
    if not np.all(np.isfinite(z_arr)):
        raise ValueError("NaN/inf in altitude — ODE blew up")
    if np.any(np.abs(sol[:, 1:4]) > 1e6):
        raise ValueError("position > 1000 km — deeply wrong")

    apogee_idx = int(np.argmax(z_arr))
    t_apogee = float(t_arr[apogee_idx])
    t_end = float(t_arr[-1])

    t_main_deploy = t_drogue_deploy = None
    for ev in flight.parachute_events:
        name = getattr(ev[1], "name", "") if len(ev) > 1 else ""
        if name == "Main" and t_main_deploy is None:
            t_main_deploy = float(ev[0])
        if name == "Drogue" and t_drogue_deploy is None:
            t_drogue_deploy = float(ev[0])

    def _descent_stats(t_start, t_stop):
        if t_start is None or t_stop is None or t_stop - t_start <= 5.0:
            return 0.0, 0.0
        mask = (t_arr >= t_start) & (t_arr <= t_stop)
        t_w = t_arr[mask]
        if len(t_w) <= 10:
            return 0.0, 0.0
        idx = np.linspace(0, len(t_w) - 1, min(200, len(t_w))).astype(int)
        vz = np.array([abs(float(flight.vz(t))) for t in t_w[idx]])
        vz = vz[np.isfinite(vz)]
        return (float(np.mean(vz)), float(np.max(vz))) if len(vz) else (0.0, 0.0)

    drogue_mean = drogue_max = 0.0
    if config in ("Nominal", "Drogue_Only"):
        drogue_mean, drogue_max = _descent_stats(
            t_drogue_deploy if t_drogue_deploy is not None else t_apogee,
            t_main_deploy if t_main_deploy is not None else t_end)

    main_mean = main_max = 0.0
    if config in ("Nominal", "Main_at_Apogee") and t_main_deploy is not None:
        main_mean, main_max = _descent_stats(t_main_deploy, t_end)

    apogee_x, apogee_y = float(flight.x(t_apogee)), float(flight.y(t_apogee))
    x_end, y_end = float(flight.x(t_end)), float(flight.y(t_end))
    try:
        v_impact = math.sqrt(float(flight.vx(t_end)) ** 2 +
                              float(flight.vy(t_end)) ** 2 +
                              float(flight.vz(t_end)) ** 2)
    except Exception:
        v_impact = abs(float(flight.vz(t_end)))

    track_step = max(1, len(t_arr) // 150)
    track = [(float(flight.y(t)), float(flight.x(t)), float(flight.z(t)))
             for t in t_arr[::track_step]]

    return {
        "t_apogee": t_apogee,
        "apogee_agl": float(z_arr[apogee_idx]) - site_elev,
        "t_end": t_end,
        "apogee_x": apogee_x, "apogee_y": apogee_y,
        "x_end": x_end, "y_end": y_end,
        "v_impact": v_impact, "track": track,
        "max_mach": float(flight.max_mach_number),
        "out_rail_v": float(flight.out_of_rail_velocity),
        "static_margin": _coerce_scalar(rocket.static_margin(0)),
        "drogue_mean_ms": drogue_mean, "drogue_max_ms": drogue_max,
        "main_mean_ms": main_mean, "main_max_ms": main_max,
        "sim_params": sim_params,
    }


_EXECUTOR = None


def _noop_worker():
    """Module-level no-op used to force worker import-spawn before the DOE loop."""
    return True


def _get_executor(n_workers):
    global _EXECUTOR
    if _EXECUTOR is None or _EXECUTOR._max_workers != n_workers:
        if _EXECUTOR is not None:
            _EXECUTOR.shutdown(wait=False, cancel_futures=True)
        _EXECUTOR = ProcessPoolExecutor(
            max_workers=n_workers,
            mp_context=mp.get_context("spawn"),
        )
    return _EXECUTOR


def _warm_up_executor(executor, n_workers):
    """Force each worker to finish its cold import path before real jobs start."""
    if executor is None:
        return

    print(f"  Spawning + importing in {n_workers} worker(s) — this can take a while on first run, especially from a OneDrive-synced path...", flush=True)
    t0 = time.perf_counter()
    futures = [executor.submit(_noop_worker) for _ in range(n_workers)]
    for i, fut in enumerate(futures, 1):
        fut.result()
        print(f"    worker {i}/{n_workers} ready ({time.perf_counter() - t0:.1f}s elapsed)", flush=True)
    print(f"  All {n_workers} workers warm ({time.perf_counter() - t0:.1f}s total)\n", flush=True)


def _restart_executor(n_workers):
    """Terminate the whole executor on a real hang and rebuild it with a fresh
    worker set rather than leaving a stuck pool behind."""
    global _EXECUTOR
    if _EXECUTOR is not None:
        _EXECUTOR.shutdown(wait=False, cancel_futures=True)
        _EXECUTOR = None
    return _get_executor(n_workers)


def _run_sim_with_timeout(timeout_s, executor, geom, nominal_thrust, perturbations,
                           config, wind_u, wind_v, rail_length, inclination, heading, site_elev):
    future = executor.submit(
        _sim_worker,
        geom, nominal_thrust, perturbations, config, wind_u, wind_v,
        rail_length, inclination, heading, site_elev,
    )
    try:
        return future.result(timeout=timeout_s)
    except FutureTimeoutError as exc:
        future.cancel()
        raise TimeoutError(f"sim exceeded {timeout_s}s — solver hang, skipping") from exc

def _patched_beta(mach):
    """Use a continuous, real-valued beta surrogate around the transonic region."""
    try:
        mach_arr = np.asarray(mach)
    except Exception:
        mach_arr = np.asarray([mach])

    if np.iscomplexobj(mach_arr):
        if np.any(np.abs(np.imag(mach_arr)) > 1e-12):
            raise TypeError("Complex Mach input is not supported by patched beta")
        mach_arr = np.real(mach_arr)

    try:
        mach_val = float(mach_arr.reshape(-1)[0])
    except Exception:
        mach_val = float(mach)

    mach_val = max(abs(mach_val), 1e-6)
    plateau_value = float(np.sqrt(max(1.0 - 0.8**2, 1e-12)))
    plateau_end = float(np.sqrt(1.0 + plateau_value**2))
    if mach_val < 0.8:
        return float(np.sqrt(max(1.0 - mach_val**2, 1e-12)))
    if mach_val < plateau_end:
        return plateau_value
    return float(np.sqrt(max(mach_val**2 - 1.0, 1e-12)))


try:
    rocketpy.rocket.aero_surface.aero_surface.AeroSurface._beta = staticmethod(_patched_beta)
except Exception:
    pass


def _coerce_scalar(value):
    """Best-effort conversion of RocketPy Function-like values to a float."""
    if callable(value):
        for candidate in (0.0, None):
            try:
                return float(value(candidate))
            except TypeError:
                continue
            except Exception:
                break
        try:
            return float(value())
        except Exception:
            pass

    if isinstance(value, np.ndarray):
        if value.size == 1:
            return float(value.reshape(-1)[0])
        return float(np.nanmean(value))

    if isinstance(value, (list, tuple)):
        if len(value) == 1:
            return _coerce_scalar(value[0])
        return float(np.nanmean([_coerce_scalar(v) for v in value]))

    try:
        return float(value)
    except Exception:
        return float("nan")


def _main_trigger_value(h, state, target_alt):
    """Return a boolean trigger result for the main parachute without blowing up on empty/array inputs."""
    try:
        h_val = float(np.asarray(h).reshape(-1)[-1])
    except Exception:
        try:
            h_val = float(h)
        except Exception:
            return False

    try:
        state_arr = np.asarray(state, dtype=float).reshape(-1)
    except Exception:
        return False

    if state_arr.size < 6:
        return False

    try:
        vz = float(state_arr[5])
    except Exception:
        return False

    try:
        target_alt_val = float(np.asarray(target_alt).reshape(-1)[-1])
    except Exception:
        try:
            target_alt_val = float(target_alt)
        except Exception:
            return False

    return (h_val <= target_alt_val) and (vz < 0.0)

# =============================================================================
# PATHS (Device specific)
# =============================================================================

BASE   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ORK_PATH = os.path.join(BASE, "Monarch REV5.ork")
ENG_PATH = os.path.join(BASE, "thrustcurve.eng")
NC1_PATH = os.path.join(BASE, "MonarchCDF1.nc")   # u, v wind
NC2_PATH = os.path.join(BASE, "MonarchCDF2.nc")   # geopotential z [m²/s²]
CD_PATH = os.path.join(BASE, "CD_Test.CSV") # Mach/Cd table

# =============================================================================
# SITE (Launch Canada 2026)
# =============================================================================

SITE_LAT  = 47.987 
SITE_LON  = -81.8485 
SITE_ELEV = 364.0 # M AGL
RAIL_LEN  = 5.6388 # 18.5ft

# =============================================================================
# DOE GRID
# =============================================================================

RAIL_ANGLES = [0, 2, 4, 6, 8] # from vert
RAIL_DIRS = [0, 90, 180, 270]  # 0=North, 90=East, etc
DAYS        = list(range(6)) # August 16-21
HOURS       = [8, 9, 10, 11, 12] # EST (AM)
MC_SAMPLES  = 1   # per condition — increase for more resolution at cost of runtime

CONFIGS = ["Ballistic", "Main_at_Apogee", "Nominal", "Drogue_Only"]
CONFIG_LABELS = {
    "Ballistic":      "LB",
    "Main_at_Apogee": "LMA",
    "Nominal":        "LN",
    "Drogue_Only":    "LDO",
}

TOTAL_SIMS = (
    len(DAYS) * len(HOURS) * len(RAIL_ANGLES) * len(RAIL_DIRS) * MC_SAMPLES * len(CONFIGS)
)

RNG_SEED = 42
rng = np.random.default_rng(RNG_SEED)

LOG_INTERVAL = 200   # print a progress line every N sims

# =============================================================================
# MC DISPERSION SIGMAS
# =============================================================================

MC_SIGMA = {
    "dry_mass_kg":      0.3, 
    "dry_cg_m":         0.008, 
    "prop_mass_kg":     0.030,
    "thrust_scale":     0.025,
    "cd_scale":         0.085,
    "main_cd_scale":    0.10,
    "rail_angle_deg":   0.400,
    "rail_heading_deg": 4.000,
    "main_alt_m":       12.00,
    "rail_length_m":    0.2, 
}
CD_SCALE_CLIP = (0.75, 1.25)
MAIN_CD_SCALE_CLIP = (0.70, 1.30)
RAIL_LENGTH_RANGE_M = (4.0, 8.0)

# =============================================================================
# PHYSICS CONSTANTS
# =============================================================================

DROGUE_DIAM_M = 24.0 * 0.0254
DROGUE_CD     = 0.97
DROGUE_AREA   = math.pi * (DROGUE_DIAM_M / 2) ** 2
MAIN_DIAM_M   = 48.0 * 0.0254
MAIN_CD       = 1.9  # hemispherical main parachute (48" tested 7/8/2026: Cd = 1.92)
G0            = 9.80665
R_AIR         = 287.058
RE            = 6_356_766.0   # Earth polar radius for geopotential conversion

# =============================================================================
# ORK PARSER
# =============================================================================

_RASAERO_CD = [
    (0.00, 0.40),
    (0.30, 0.40),
    (0.70, 0.45),
    (0.85, 0.60),
    (0.95, 0.75),
    (1.00, 0.85),
    (1.05, 0.80),
    (1.10, 0.75),
    (1.30, 0.60),
    (1.50, 0.52),
    (2.00, 0.48),
    (2.50, 0.45),
    (3.00, 0.43),
]

def load_CD_from_csv(path):
    """Load a Mach/Cd CSV exported from OpenRocket or a component-analysis tool."""
    if not path or not os.path.exists(path):
        return None
    points = []
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                parts = [p.strip() for p in line.replace(";", ",").split(",")]
                if len(parts) < 2:
                    continue
                try:
                    mach = float(parts[0])
                    cd = float(parts[1])
                except ValueError:
                    if len(parts) >= 3:
                        try:
                            mach = float(parts[1])
                            cd = float(parts[2])
                        except ValueError:
                            continue
                    else:
                        continue
                if math.isfinite(mach) and math.isfinite(cd):
                    points.append((mach, cd))
    except Exception:
        return None
    if not points:
        return None
    return sorted(points)


# Backward-compatible alias for the older loader name used elsewhere in the code.
load_cd_csv = load_CD_from_csv


def load_rasaero_cd_csv(path, alpha="0"):
    """Load a RASAero II Mach-sweep CSV export with separate power-off and power-on curves.
    Returns (cd_off_curve, cd_on_curve), both list[(mach, cd)]."""
    if not path or not os.path.exists(path):
        return None, None
    try:
        import csv as _csv
        rows = []
        with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
            reader = _csv.DictReader(f)
            if not reader.fieldnames or "CD Power-Off" not in reader.fieldnames:
                return None, None
            for row in reader:
                if str(row.get("Alpha", "")).strip() != str(alpha):
                    continue
                try:
                    mach = float(row["Mach"])
                    cd_off = float(row["CD Power-Off"])
                    cd_on = float(row["CD Power-On"])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(mach) and math.isfinite(cd_off) and math.isfinite(cd_on):
                    rows.append((mach, cd_off, cd_on))
    except Exception:
        return None, None

    if not rows:
        return None, None
    rows.sort()

    decimated = []
    for i, (mach, cd_off, cd_on) in enumerate(rows):
        if 0.6 <= mach <= 1.6:
            decimated.append((mach, cd_off, cd_on))
        elif i % 5 == 0:
            decimated.append((mach, cd_off, cd_on))
    if decimated and decimated[-1][0] != rows[-1][0]:
        decimated.append(rows[-1])

    cd_off_curve = [(m, c) for m, c, _ in decimated]
    cd_on_curve = [(m, c) for m, _, c in decimated]
    return cd_off_curve, cd_on_curve


def _parse_cdplot(xml_root):
    """Pull Mach/Cd tables from ORK cdplot elements if they exist."""
    cd_off, cd_on = [], []
    for el in xml_root.iter():
        if "cdplot" not in el.tag.lower():
            continue
        pts = []
        for p in el:
            m = p.get("mach") or p.get("Mach")
            c = p.get("cd")   or p.get("CD")
            if m and c:
                try: pts.append((float(m), float(c)))
                except ValueError: pass
        if not pts:
            continue
        kind = (el.get("type") or el.get("Type") or "poweroff").lower()
        (cd_on if "on" in kind else cd_off).extend(pts)

    if not cd_off:
        return None, None
    cd_off.sort()
    cd_on  = sorted(cd_on) if cd_on else [(m, cd * 0.90) for m, cd in cd_off]
    return cd_off, cd_on


_trapz = getattr(np, "trapezoid", getattr(np, "trapz", None))  # numpy 1.x/2.x compat

EXTERNAL_TAGS = {"nosecone", "bodytube", "transition"}
FINSET_TAGS   = {"trapezoidfinset", "freeformfinset", "ellipticalfinset", "tubefinset"}
INNER_TAGS    = {"innertube", "tubecoupler", "centeringring", "bulkhead",
                 "engineblock", "masscomponent", "parachute", "streamer",
                 "shockcord", "launchlug", "railbutton"}
ALL_COMPONENT_TAGS = EXTERNAL_TAGS | FINSET_TAGS | INNER_TAGS


def _text(el, tag):
    e = el.find(tag)
    return e.text if (e is not None and e.text is not None) else None


def _flt(el, tag, default=None):
    t = _text(el, tag)
    if t is None:
        return default
    t = t.strip()
    if t.startswith("auto"):
        rest = t[4:].strip()
        return float(rest) if rest else default
    try:
        return float(t)
    except ValueError:
        return default


def _bool(el, tag, default=False):
    t = _text(el, tag)
    return t.strip().lower() == "true" if t else default


def _mat_density(el, tag="material"):
    m = el.find(tag)
    if m is None:
        return None, None
    try:
        return float(m.get("density")), (m.get("type") or "bulk").lower()
    except (TypeError, ValueError):
        return None, None


def _nose_profile(shape, param, R, L, n=400):
    """Return x[], r[] for the given OpenRocket nose shape (tip at x=0)."""
    x = np.linspace(0.0, L, n)
    xi = x / L
    shape = (shape or "ogive").lower()
    if shape == "conical":
        r = R * xi
    elif shape == "ellipsoid":
        r = R * np.sqrt(np.clip(1.0 - (1.0 - xi) ** 2, 0.0, 1.0))
    elif shape == "power":
        p = param if param and param > 0 else 0.5
        r = R * xi ** p
    elif shape == "parabolic":
        K = param if param is not None else 1.0
        r = R * xi * (2.0 - K * xi) / max(2.0 - K, 1e-9)
    elif shape == "haack":
        C = param if param is not None else 0.0
        theta = np.arccos(np.clip(1.0 - 2.0 * xi, -1.0, 1.0))
        r = (R / math.sqrt(math.pi)) * np.sqrt(
            np.clip(theta - np.sin(2 * theta) / 2.0 + C * np.sin(theta) ** 3, 0.0, None))
    else:
        rho = (R**2 + L**2) / (2.0 * R)
        r = np.sqrt(np.clip(rho**2 - (L - x) ** 2, 0.0, None)) + R - rho
        r = np.clip(r, 0.0, R)
    return x, r


def _shell_mass_cg(x, r, thickness, density):
    """pi * integral(r^2 - max(r-t,0)^2) dx  -> (mass, cg_from_top)."""
    r_in = np.clip(r - thickness, 0.0, None)
    dA = math.pi * (r**2 - r_in**2)
    V = float(_trapz(dA, x))
    if V <= 0:
        return 0.0, float(x[-1]) / 2.0
    xbar = float(_trapz(x * dA, x) / V)
    return V * density, xbar


def _nose_kind_for_rocketpy(shape, param):
    shape = (shape or "").lower()
    if shape == "haack":
        p = param if param is not None else 0.0
        return "von karman" if p < 0.17 else "lvhaack"
    return {"conical": "conical", "ogive": "ogive", "ellipsoid": "elliptical",
            "parabolic": "parabolic", "power": "powerseries"}.get(shape, "ogive")


def _fin_geometry(fin_el):
    """Return (root, tip, span, sweep, bounding_len, area_per_fin, centroid_x, count)."""
    count = int(_flt(fin_el, "fincount", 4) or 4)
    tag = fin_el.tag.lower()

    if tag == "freeformfinset":
        pts_el = fin_el.find("finpoints")
        coords = [(float(p.get("x", 0)), float(p.get("y", 0)))
                  for p in pts_el.findall("point")] if pts_el is not None else []
        if len(coords) >= 3:
            root_pts = [p for p in coords if abs(p[1]) < 1e-9]
            tip_y    = max(p[1] for p in coords)
            tip_pts  = [p for p in coords if abs(p[1] - tip_y) < 1e-9 and tip_y > 0]
            root  = max(p[0] for p in root_pts) - min(p[0] for p in root_pts)
            tip   = (max(p[0] for p in tip_pts) - min(p[0] for p in tip_pts)) if tip_pts else 0.0
            span  = tip_y
            sweep = (min(p[0] for p in tip_pts) - min(p[0] for p in root_pts)) if tip_pts else 0.0
            blen  = max(p[0] for p in coords) - min(p[0] for p in coords)
            a = cx = 0.0
            for (x1, y1), (x2, y2) in zip(coords, coords[1:] + coords[:1]):
                cross = x1 * y2 - x2 * y1
                a += cross
                cx += (x1 + x2) * cross
            area = abs(a) / 2.0
            cxa  = (cx / (3.0 * a)) if abs(a) > 1e-12 else blen / 2.0
            return root, tip, span, sweep, blen, area, cxa, count

    if tag == "ellipticalfinset":
        root = _flt(fin_el, "rootchord", 0.2) or 0.2
        span = _flt(fin_el, "height", 0.08) or 0.08
        area = math.pi / 4.0 * root * span
        tip  = root * (math.pi / 2.0 - 1.0)
        sweep = (root - tip) / 2.0
        return root, tip, span, sweep, root, area, root / 2.0, count

    root  = _flt(fin_el, "rootchord",   0.2)  or 0.2
    tip   = _flt(fin_el, "tipchord",    0.05) or 0.05
    span  = _flt(fin_el, "height",      0.08) or 0.08
    sweep = _flt(fin_el, "sweeplength", 0.05) or 0.05
    blen  = max(root, sweep + tip)
    area  = 0.5 * (root + tip) * span
    denom = root + tip
    cxa = (root**2 + root * tip + tip**2 + 3.0 * sweep * tip) / (3.0 * denom) if denom > 0 else blen / 2
    return root, tip, span, sweep, blen, area, cxa, count


def _component_mass_cg(el, ctx):
    """
    Return (mass_kg, cg_local_from_top, length, source_str).
    ctx carries parent geometry for 'auto' radius resolution.
    """
    tag = el.tag.lower()
    override = _flt(el, "overridemass")
    dens, dtype = _mat_density(el)

    if tag == "nosecone":
        L = _flt(el, "length", 0.3)
        R = _flt(el, "aftradius", ctx.get("radius", 0.04))
        t = _flt(el, "thickness", 0.002)
        x, r = _nose_profile(_text(el, "shape"), _flt(el, "shapeparameter"), R, L)
        m, cg = _shell_mass_cg(x, r, t, dens or 1850.0)
        sr, sl, st = (_flt(el, "aftshoulderradius", 0.0), _flt(el, "aftshoulderlength", 0.0),
                      _flt(el, "aftshoulderthickness", 0.0))
        if sr and sl and st:
            sm = math.pi * (sr**2 - max(sr - st, 0)**2) * sl * (dens or 1850.0)
            cg = (m * cg + sm * (L + sl / 2)) / (m + sm)
            m += sm
        return (override if override is not None else m), cg, L, ("override" if override is not None else "material")

    if tag == "transition":
        L  = _flt(el, "length", 0.05)
        r1 = _flt(el, "foreradius", ctx.get("radius", 0.04))
        r2 = _flt(el, "aftradius",  ctx.get("radius", 0.04))
        t  = _flt(el, "thickness", 0.002)
        x  = np.linspace(0, L, 200)
        r  = r1 + (r2 - r1) * x / L
        m, cg = _shell_mass_cg(x, r, t, dens or 1780.0)
        return (override if override is not None else m), cg, L, ("override" if override is not None else "material")

    if tag == "bodytube":
        L  = _flt(el, "length", 0.1)
        ro = _flt(el, "radius", ctx.get("radius", 0.04))
        t  = _flt(el, "thickness", 0.002)
        m  = math.pi * (ro**2 - max(ro - t, 0)**2) * L * (dens or 1780.0)
        return (override if override is not None else m), L / 2, L, ("override" if override is not None else "material")

    if tag in ("innertube", "tubecoupler", "engineblock", "launchlug"):
        L  = _flt(el, "length", 0.05)
        ro = _flt(el, "outerradius", None)
        if ro is None:
            ro = max(ctx.get("radius", 0.04) - ctx.get("thickness", 0.002), 0.01)
        t  = _flt(el, "thickness", 0.001)
        m  = math.pi * (ro**2 - max(ro - t, 0)**2) * L * (dens or 1850.0)
        return (override if override is not None else m), L / 2, L, ("override" if override is not None else "material")

    if tag == "centeringring":
        L  = _flt(el, "length", 0.005)
        ro = _flt(el, "outerradius", None)
        if ro is None:
            ro = max(ctx.get("radius", 0.04) - ctx.get("thickness", 0.002), 0.01)
        ri = _flt(el, "innerradius", 0.0) or 0.0
        m  = math.pi * (ro**2 - ri**2) * L * (dens or 1850.0)
        return (override if override is not None else m), L / 2, L, ("override" if override is not None else "material")

    if tag == "bulkhead":
        L  = _flt(el, "length", 0.005)
        ro = _flt(el, "outerradius", None)
        if ro is None:
            ro = max(ctx.get("radius", 0.04) - ctx.get("thickness", 0.002), 0.01)
        m  = math.pi * ro**2 * L * (dens or 2700.0)
        return (override if override is not None else m), L / 2, L, ("override" if override is not None else "material")

    if tag == "masscomponent":
        L = _flt(el, "packedlength", _flt(el, "length", 0.05))
        m = _flt(el, "mass", 0.0) or 0.0
        return (override if override is not None else m), (L or 0.05) / 2, (L or 0.05), ("override" if override is not None else "explicit")

    if tag == "parachute":
        L = _flt(el, "packedlength", 0.1) or 0.1
        d = _flt(el, "diameter", 0.6) or 0.6
        canopy = (dens or 0.067) * math.pi * (d / 2)**2 if dtype == "surface" else 0.0
        ldens, _ = _mat_density(el, "linematerial")
        lines = (int(_flt(el, "linecount", 0) or 0) * (_flt(el, "linelength", 0.0) or 0.0)
                 * (ldens or 0.0))
        m = canopy + lines
        return (override if override is not None else m), L / 2, L, ("override" if override is not None else "material")

    if tag == "streamer":
        L  = _flt(el, "packedlength", 0.05) or 0.05
        sl = _flt(el, "striplength", 0.5) or 0.5
        sw = _flt(el, "stripwidth", 0.05) or 0.05
        m  = (dens or 0.067) * sl * sw if dtype == "surface" else 0.0
        return (override if override is not None else m), L / 2, L, ("override" if override is not None else "material")

    if tag == "shockcord":
        L  = _flt(el, "packedlength", 0.05) or 0.05
        cl = _flt(el, "cordlength", 1.0) or 1.0
        m  = (dens or 0.0018) * cl if dtype == "line" else (dens or 0.0018) * cl
        return (override if override is not None else m), L / 2, L, ("override" if override is not None else "material")

    if tag in FINSET_TAGS:
        root, tip, span, sweep, blen, area, cxa, count = _fin_geometry(el)
        t = _flt(el, "thickness", 0.003) or 0.003
        m = area * t * count * (dens or 1780.0)
        return (override if override is not None else m), cxa, blen, ("override" if override is not None else "material")

    if tag == "railbutton":
        return (override if override is not None else 0.010), 0.005, 0.01, ("override" if override is not None else "estimate")

    return (override if override is not None else 0.0), 0.0, 0.0, "unknown"


def _abs_top(el, parent_top, parent_len, comp_len, cursor):
    """Resolve absolute top coordinate of a component."""
    ax = el.find("axialoffset")
    if ax is None:
        return cursor
    try:
        off = float(ax.text)
    except (TypeError, ValueError):
        off = 0.0
    method = (ax.get("method") or "top").lower()
    if method == "top":
        return parent_top + off
    if method == "middle":
        return parent_top + (parent_len - comp_len) / 2.0 + off
    if method == "bottom":
        return parent_top + parent_len - comp_len + off
    if method == "absolute":
        return off
    if method == "after":
        return cursor + off
    return parent_top + off


def parse_ork(ork_path,
              motor_mass=6.293, prop_mass=4.903, casing_mass=1.390, motor_length=1.025,
              main_diam=48 * 0.0254, main_cd=1.9, main_alt_agl=457.2,
              drogue_diam=24 * 0.0254, drogue_cd=0.97,
              rasaero_cd=None, parse_cdplot=None, cd_csv_path=None,
              rasaero_sweep_csv_path=None):

    if zipfile.is_zipfile(ork_path):
        with zipfile.ZipFile(ork_path) as z:
            xml_root = ET.fromstring(z.read("rocket.ork"))
    else:
        xml_root = ET.parse(ork_path).getroot()

    rocket_el = next(e for e in xml_root.iter() if e.tag.lower() == "rocket")
    stages = [s for s in rocket_el.iter()
              if s.tag.lower() == "stage" and s.find("subcomponents") is not None
              and len(list(s.find("subcomponents")))]
    if not stages:
        raise RuntimeError("no populated stage found in ork — check the file")

    inventory = []
    finsets   = []
    externals = []
    nose_info = None
    warnings  = []

    def walk(el, parent_top, parent_len, ctx, skip_children_mass=False):
        nonlocal nose_info
        tag = el.tag.lower()
        name = _text(el, "name") or tag

        mass, cg_local, comp_len, source = _component_mass_cg(el, ctx)
        top = _abs_top(el, parent_top, parent_len, comp_len, ctx.get("cursor", parent_top))

        skip_own = skip_children_mass
        if not skip_own and mass > 0:
            inventory.append((name, tag, mass, source, top + cg_local, top, comp_len))
        elif skip_own and mass > 0:
            inventory.append((name, tag, 0.0, f"{source} (absorbed by parent override)",
                              top + cg_local, top, comp_len))

        child_ctx = dict(ctx)
        if tag == "bodytube":
            child_ctx["radius"]    = _flt(el, "radius", ctx.get("radius", 0.04))
            child_ctx["thickness"] = _flt(el, "thickness", 0.002)
        elif tag == "nosecone":
            child_ctx["radius"]    = _flt(el, "aftradius", ctx.get("radius", 0.04))
            child_ctx["thickness"] = _flt(el, "thickness", 0.002)
        elif tag in ("tubecoupler", "innertube"):
            ro = _flt(el, "outerradius", None)
            if ro is None:
                ro = max(ctx.get("radius", 0.04) - ctx.get("thickness", 0.002), 0.01)
            child_ctx["radius"]    = ro
            child_ctx["thickness"] = _flt(el, "thickness", 0.001)

        if tag == "nosecone" and nose_info is None:
            nose_info = {
                "length": comp_len,
                "aftradius": _flt(el, "aftradius", 0.04),
                "kind": _nose_kind_for_rocketpy(_text(el, "shape"), _flt(el, "shapeparameter")),
                "shape_raw": _text(el, "shape"),
                "param": _flt(el, "shapeparameter"),
            }

        if tag in FINSET_TAGS:
            root, tip, span, sweep, blen, area, cxa, count = _fin_geometry(el)
            finsets.append((area * count, {
                "fin_count": count, "fin_root": root, "fin_tip": tip,
                "fin_span": span, "fin_sweep": sweep,
                "fin_le_pos": top, "fin_type": tag, "name": name,
            }))

        if tag in EXTERNAL_TAGS:
            if tag == "transition":
                fore = _flt(el, "foreradius", ctx.get("radius", 0.04))
                aft  = _flt(el, "aftradius",  ctx.get("radius", 0.04))
            else:
                fore = aft = (_flt(el, "aftradius", None) if tag == "nosecone"
                              else _flt(el, "radius", ctx.get("radius", 0.04)))
            externals.append((tag, top, comp_len, fore, aft, name))

        subs = el.find("subcomponents")
        override_children = _bool(el, "overridesubcomponentsmass", False) and \
                            (_flt(el, "overridemass") is not None)
        if subs is not None:
            for c in subs:
                if c.tag.lower() in ALL_COMPONENT_TAGS:
                    walk(c, top, comp_len, child_ctx, skip_children_mass=override_children)

    cursor = 0.0
    stage_mass_override = None
    stage_cg_override = None
    for stage in stages:
        stage_mass_override = _flt(stage, "overridemass")
        stage_cg_override   = _flt(stage, "overridecg")
        stage_skip_children = (_bool(stage, "overridesubcomponentsmass", False)
                                and stage_mass_override is not None)

        for c in stage.find("subcomponents"):
            tag = c.tag.lower()
            if tag not in ALL_COMPONENT_TAGS:
                continue
            _, _, comp_len, _ = _component_mass_cg(c, {"radius": 0.04, "thickness": 0.002,
                                                     "cursor": cursor})
            ctx = {"radius": externals[-1][4] if externals else 0.04,
                   "thickness": 0.002, "cursor": cursor}
            walk(c, cursor, comp_len, ctx, skip_children_mass=stage_skip_children)
            if tag in EXTERNAL_TAGS:
                cursor += comp_len

    total_length = cursor
    if not finsets:
        raise RuntimeError("no fin set found anywhere in ork — check the file")

    finsets.sort(key=lambda f: -f[0])
    fins = finsets[0][1]
    if len(finsets) > 1:
        warnings.append(f"{len(finsets)} fin sets found; using largest "
                        f"('{fins['name']}'); others ignored by the single-finset model")

    if nose_info is None:
        raise RuntimeError("no nosecone found in ork — check the file")

    dry_mass = sum(m for _, _, m, _, _, _, _ in inventory)
    dry_cg   = (sum(m * cg for _, _, m, _, cg, _, _ in inventory) / dry_mass) if dry_mass > 0 else 0.0

    if stage_mass_override is not None:
        resolved_cg = stage_cg_override if stage_cg_override is not None else dry_cg
        inventory.append(("Stage override", "stage", stage_mass_override,
                          "stage-override", resolved_cg, 0.0, total_length))
        warnings.append(f"stage-level overridemass applied: {stage_mass_override:.4f} kg"
                        f"{f' @ cg {stage_cg_override:.4f} m' if stage_cg_override is not None else ''}"
                        f" — component-summed mass ({dry_mass:.4f} kg) discarded")
        dry_mass = stage_mass_override
        dry_cg = resolved_cg

    if not (0.05 * total_length < dry_cg < 0.95 * total_length):
        warnings.append(f"dry CG {dry_cg:.3f} m outside sanity band — check component positions")

    body_radius = nose_info["aftradius"]
    max_radius  = max((max(f or 0, a or 0) for _, _, _, f, a, _ in externals), default=body_radius)
    tail = None
    for tag, top, length, fore, aft, name in externals:
        if tag == "transition":
            tail = {"top_radius": fore, "bottom_radius": aft, "length": length, "position": top}

    if not any(t == "parachute" for _, t, *_ in inventory):
        warnings.append("no <parachute> components in ORK — recovery hardware mass is NOT "
                        "included in dry mass (add masscomponents in OpenRocket to capture it)")

    ork_cd_off = ork_cd_on = None
    if parse_cdplot is not None:
        ork_cd_off, ork_cd_on = parse_cdplot(xml_root)

    sweep_cd_off, sweep_cd_on = (
        load_rasaero_cd_csv(rasaero_sweep_csv_path) if rasaero_sweep_csv_path else (None, None)
    )
    csv_cd = load_CD_from_csv(cd_csv_path) if cd_csv_path else None
    fallback = rasaero_cd or [(0.0, 0.4), (3.0, 0.43)]

    if ork_cd_off is not None:
        cd_off = ork_cd_off
    elif sweep_cd_off is not None:
        cd_off = sweep_cd_off
    elif csv_cd is not None:
        cd_off = csv_cd
    else:
        cd_off = fallback

    if ork_cd_on is not None:
        cd_on = ork_cd_on
    elif sweep_cd_on is not None:
        cd_on = sweep_cd_on
    elif csv_cd is not None:
        cd_on = csv_cd
    else:
        cd_on = [(m, cd * 0.90) for m, cd in fallback]

    geom = {
        "total_length": total_length,
        "body_radius":  body_radius,
        "max_radius":   max_radius,
        "nose_length":  nose_info["length"],
        "nose_kind":    nose_info["kind"],
        "fin_count":    fins["fin_count"],
        "fin_root":     fins["fin_root"],
        "fin_tip":      fins["fin_tip"],
        "fin_span":     fins["fin_span"],
        "fin_sweep":    fins["fin_sweep"],
        "fin_le_pos":   fins["fin_le_pos"],
        "tail":         tail,
        "dry_mass":     dry_mass,
        "dry_cg":       dry_cg,
        "motor_mass":   motor_mass,
        "prop_mass":    prop_mass,
        "casing_mass":  casing_mass,
        "motor_length": motor_length,
        "main_diam":    main_diam,
        "main_cd":      main_cd,
        "main_alt_agl": main_alt_agl,
        "drogue_diam":  drogue_diam,
        "drogue_cd":    drogue_cd,
        "cd_off_nom":   cd_off,
        "cd_on_nom":    cd_on,
        "inventory":    inventory,
        "warnings":     warnings,
    }

    IN = 0.0254
    cd_source = (
        'ORK cdplot' if ork_cd_off else
        'RASAero Mach sweep CSV' if sweep_cd_off else
        'component-analysis CSV' if csv_cd else
        'fallback table'
    )
    print(f"    Cd source : {cd_source}")
    if cd_source == 'fallback table':
        print("    WARNING: using the hardcoded fallback drag curve — results are provisional until ORK cdplot, a RASAero Mach sweep CSV, or a REV5 component-analysis CSV is used.")
    print("  GEOMETRY + MASS FROM ORK (full-tree parse)")
    print(f"    Total length  : {total_length:.4f} m   |  ref radius {body_radius*1000:.2f} mm"
          f"  (max {max_radius*1000:.2f} mm)")
    print(f"    Nose          : {nose_info['length']:.4f} m  "
          f"{nose_info['shape_raw']}(C={nose_info['param']}) -> RocketPy '{nose_info['kind']}'")
    print(f"    Fins x{fins['fin_count']} [{fins['fin_type']}] : root={fins['fin_root']/IN:.2f}\""
          f"  tip={fins['fin_tip']/IN:.2f}\"  span={fins['fin_span']/IN:.2f}\""
          f"  sweep={fins['fin_sweep']/IN:.2f}\"")
    print(f"    Fin LE        : {fins['fin_le_pos']:.4f} m from nose "
          f"(TE at {fins['fin_le_pos']+max(fins['fin_root'], fins['fin_sweep']+fins['fin_tip']):.4f} m)")
    if tail:
        print(f"    Tail          : transition {tail['top_radius']*1000:.1f}->{tail['bottom_radius']*1000:.1f} mm over {tail['length']*1000:.0f} mm "
              f"@ {tail['position']:.4f} m")
    print(f"    ── component mass inventory ─────────────────────────────────")
    for name, tag, m, src, cg, top, ln in inventory:
        print(f"      {name[:26]:26s} {tag[:14]:14s} {m:7.4f} kg  ({src[:9]:9s})"
              f"  cg@{cg:6.3f} m")
    print(f"    ─────────────────────────────────────────────────────────────")
    print(f"    DRY MASS (no motor) : {dry_mass:.3f} kg   |  DRY CG: {dry_cg:.4f} m"
          f" ({dry_cg/IN:.2f}\")")
    wet_mass = dry_mass + motor_mass
    motor_cg = total_length - motor_length / 2.0
    wet_cg   = (dry_mass * dry_cg + motor_mass * motor_cg) / wet_mass
    print(f"    WET  MASS           : {wet_mass:.3f} kg   |  WET CG: {wet_cg:.4f} m"
          f" ({wet_cg/IN:.2f}\")")
    for w in warnings:
        print(f"    [!] {w}")
    return geom


# =============================================================================
# THRUST CURVE
# =============================================================================

def load_eng(eng_path):
    FALLBACK = [
        (0.000, 0.0), (0.002, 500.0), (0.010, 3000.0),
        (0.050, 3200.0), (0.150, 3100.0), (0.300, 2900.0),
        (0.500, 2700.0), (0.800, 2600.0), (1.200, 2400.0),
        (1.600, 2300.0), (2.000, 2200.0), (2.400, 2200.0),
        (2.800, 2250.0), (3.200, 2300.0), (3.600, 2350.0),
        (4.000, 2400.0), (4.200, 2300.0), (4.400, 1800.0),
        (4.440, 0.0),
    ]
    if not os.path.exists(eng_path):
        print(f"  [!] {eng_path} not found — using fallback curve")
        return FALLBACK

    points, header_seen = [], False
    with open(eng_path, "r", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith(";"): continue
            if not header_seen: header_seen = True; continue
            parts = s.split()
            if len(parts) >= 2:
                try: points.append((float(parts[0]), float(parts[1])))
                except ValueError: continue

    if len(points) < 2:
        return FALLBACK

    t  = np.array([p[0] for p in points])
    f  = np.array([p[1] for p in points])
    fs = np.convolve(f, np.ones(5) / 5, mode="same")
    fs[0] = f[0]; fs[-1] = f[-1]
    result = list(zip(t.tolist(), fs.tolist()))

    if len(result) >= 2:
        slope = abs(result[1][1] - result[0][1]) / max(result[1][0] - result[0][0], 1e-9)
        if slope > 10_000 and result[0][0] < 0.02:
            f1 = result[1][1]
            result = [(result[0][0], 0.0), (0.002, f1 * 0.15), (0.008, f1 * 0.85)] + result[1:]

    return result


# =============================================================================
# BUILD MOTOR + ROCKET
# =============================================================================

def build_rocket_and_motor(geom, nominal_thrust, perturbations, config="Nominal"):

    dry_mass = float(np.clip(
        geom["dry_mass"] + perturbations.get("dry_mass_kg", 0.0),
        geom["dry_mass"] * 0.90, geom["dry_mass"] * 1.10))
    dry_cg = float(np.clip(
        geom["dry_cg"] + perturbations.get("dry_cg_m", 0.0),
        0.20 * geom["total_length"], 0.80 * geom["total_length"]))
    thrust_scale = float(np.clip(1.0 + perturbations.get("thrust_scale", 0.0), 0.95, 1.05))
    cd_scale     = float(np.clip(1.0 + perturbations.get("cd_scale",     0.0), *CD_SCALE_CLIP))
    main_cd_scale = float(np.clip(1.0 + perturbations.get("main_cd_scale", 0.0), *MAIN_CD_SCALE_CLIP))
    rail_length  = float(np.clip(RAIL_LEN + perturbations.get("rail_length_m", 0.0), *RAIL_LENGTH_RANGE_M))
    main_alt_agl = float(np.clip(
        geom["main_alt_agl"] + perturbations.get("main_alt_m", 0.0), 200.0, 800.0))

    thrust_data = [(t, f * thrust_scale) for t, f in nominal_thrust]

    MOTOR_LEN = geom["motor_length"]
    motor = SolidMotor(
        thrust_source=thrust_data,
        dry_mass=geom["casing_mass"],
        dry_inertia=(0.10, 0.10, 0.002),
        nozzle_radius=0.0295,
        nozzle_position=0.0,
        grain_number=6,
        grain_density=1760,
        grain_outer_radius=0.0330,
        grain_initial_inner_radius=0.0145,
        grain_initial_height=0.1683,
        grain_separation=0.0030,
        grains_center_of_mass_position=MOTOR_LEN / 2,
        center_of_dry_mass_position=MOTOR_LEN / 2,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
        interpolation_method="linear",
    )

    L = geom["total_length"]
    r = geom["body_radius"]
    I_lat = (1/12) * dry_mass * L**2 + (1/4) * dry_mass * r**2
    I_ax  = 0.5 * dry_mass * r**2

    cd_off = [(m, cd * cd_scale) for m, cd in geom["cd_off_nom"]]
    cd_on  = [(m, cd * cd_scale) for m, cd in geom["cd_on_nom"]]

    rocket = Rocket(
        radius=geom["body_radius"],
        mass=dry_mass,
        inertia=(I_lat, I_lat, I_ax),
        power_off_drag=cd_off,
        power_on_drag=cd_on,
        center_of_mass_without_motor=dry_cg,
        coordinate_system_orientation="nose_to_tail",
    )

    rocket.add_motor(motor, position=geom["total_length"])
    rocket.add_nose(length=geom["nose_length"], kind=geom["nose_kind"], position=0.0)
    rocket.add_trapezoidal_fins(
        n=geom["fin_count"],
        root_chord=geom["fin_root"],
        tip_chord=geom["fin_tip"],
        span=geom["fin_span"],
        sweep_length=geom["fin_sweep"],
        position=geom["fin_le_pos"],
        cant_angle=0.0,
    )

    if geom.get("tail") and hasattr(rocket, "add_tail"):
        tail = geom["tail"]
        rocket.add_tail(
            top_radius=tail["top_radius"],
            bottom_radius=tail["bottom_radius"],
            length=tail["length"],
            position=tail["position"],
        )

    main_cd_value = geom["main_cd"] * main_cd_scale
    main_cds      = main_cd_value * math.pi * (geom["main_diam"]   / 2) ** 2
    drogue_cds    = geom["drogue_cd"] * math.pi * (geom["drogue_diam"] / 2) ** 2
    _main_alt  = main_alt_agl

    def main_trigger(p, h, state):
        return _main_trigger_value(h, state, _main_alt)

    if config == "Nominal":
        rocket.add_parachute("Drogue", cd_s=drogue_cds, trigger="apogee",
                             sampling_rate=50, lag=1.5, noise=(0, 8.3, 0.5))
        rocket.add_parachute("Main",   cd_s=main_cds,   trigger=main_trigger,
                             sampling_rate=25, lag=1.5, noise=(0, 8.3, 0.5))
    elif config == "Main_at_Apogee":
        rocket.add_parachute("Main", cd_s=main_cds, trigger="apogee",
                             sampling_rate=25, lag=1.5, noise=(0, 8.3, 0.5))
    elif config == "Drogue_Only":
        rocket.add_parachute("Drogue", cd_s=drogue_cds, trigger="apogee",
                             sampling_rate=50, lag=1.5, noise=(0, 8.3, 0.5))
    # Ballistic: no chutes

    sm_wet = _coerce_scalar(rocket.static_margin(0))
    sm_dry = _coerce_scalar(rocket.static_margin(motor.burn_out_time))

    sm = min(sm_wet, sm_dry)

    if not (np.isfinite(I_ax) and np.isfinite(I_lat) and I_ax > 1e-6 and I_lat > 1e-6):
        raise ValueError(f"Non-physical inertia: I_ax={I_ax:.3g} I_lat={I_lat:.3g}")
    ratio = I_lat / I_ax
    if not (1.0 < ratio < 1e4):
        raise ValueError(f"Inertia ratio I_lat/I_ax={ratio:.1f} outside sane bounds "
                         f"(dry_mass={dry_mass:.3f} dry_cg={dry_cg:.3f}) — bad MC draw")

    return rocket, motor, {
        "dry_mass":      dry_mass,
        "dry_cg":        dry_cg,
        "thrust_scale":  thrust_scale,
        "cd_scale":      cd_scale,
        "main_cd_scale": main_cd_scale,
        "main_cd":       main_cd_value,
        "main_alt_agl":  main_alt_agl,
        "rail_length_m": rail_length,
    }


# =============================================================================
# ERA5: wind (NC1) + geopotential density (NC2)
# =============================================================================

def _geopotential_to_geometric(z_m2s2):
    """ERA5 geopotential [m²/s²] → geometric altitude [m]."""
    h_geop = z_m2s2 / G0
    return RE * h_geop / (RE - h_geop)


def load_wind_profiles(nc1_path, nc2_path, days, hours):
    def _open(path):
        try:    return xr.open_dataset(path, engine="h5netcdf")
        except: return xr.open_dataset(path, engine="netcdf4")

    # ── NC1: wind ─────────────────────────────────────────────────────────────
    print(f"  Opening {os.path.basename(nc1_path)} ...", flush=True)
    with _open(nc1_path) as ds:
        n_times = len(ds.valid_time)
        valid_times = ds.valid_time.values
        site    = ds.sel(latitude=SITE_LAT, longitude=SITE_LON, method="nearest")
        p_levs  = site["pressure_level"].values.astype(float)
        u_all   = site["u"].values.astype(float)
        v_all   = site["v"].values.astype(float)
    if n_times < len(days) * len(hours):
        raise ValueError(f"NC1 has {n_times} time steps, but {len(days) * len(hours)} day/hour combos were requested")
    print(f"  NC1: {n_times} time steps, {len(p_levs)} pressure levels (u, v wind)")
    print(f"  NC1 time range: {valid_times[0]} -> {valid_times[-1]}")

    p_pa   = p_levs * 100.0
    z_hyps = 44330.0 * (1.0 - (p_pa / 101325.0) ** 0.1903)
    ord1   = np.argsort(z_hyps)
    z_wind = z_hyps[ord1]

    # ── NC2: geopotential → density ───────────────────────────────────────────
    geopot_all  = None
    geopot_levs = None
    if os.path.exists(nc2_path):
        print(f"  Opening {os.path.basename(nc2_path)} ...", flush=True)
        with _open(nc2_path) as ds2:
            site2       = ds2.sel(latitude=SITE_LAT, longitude=SITE_LON, method="nearest")
            geopot_levs = site2["pressure_level"].values.astype(float)
            geopot_all  = site2["z"].values.astype(float)
        print(f"  NC2: geopotential loaded — {len(geopot_levs)} pressure levels")
        print(f"       altitude range: "
              f"{_geopotential_to_geometric(geopot_all.min())/1000:.1f}"
              f"–{_geopotential_to_geometric(geopot_all.max())/1000:.1f} km geometric")
    else:
        print(f"  [!] NC2 not found at {nc2_path} — density falls back to ISA")

    cache       = {}
    density_lut = {}
    wind_raw    = {}

    for d in days:
        for h in hours:
            tidx = (d * len(hours) + hours.index(h)) % n_times
            try:
                u_raw = np.clip(
                    np.where(np.isfinite(u_all[tidx][ord1]), u_all[tidx][ord1], 0.0),
                    -150, 150)
                v_raw = np.clip(
                    np.where(np.isfinite(v_all[tidx][ord1]), v_all[tidx][ord1], 0.0),
                    -150, 150)
                u_d = u_raw[::2]; v_d = v_raw[::2]; z_d = z_wind[::2]
                _, uid = np.unique(z_d, return_index=True)
                z_u = z_d[uid]; uu = u_d[uid]; vv = v_d[uid]
                if not np.all(np.diff(z_u) > 0):
                    raise ValueError("wind altitudes not monotone after dedup")

                wind_u = [(float(z), float(u)) for z, u in zip(z_u, uu)]
                wind_v = [(float(z), float(v)) for z, v in zip(z_u, vv)]
                wind_raw[(d, h)] = (wind_u, wind_v)

                env = Environment(latitude=SITE_LAT, longitude=SITE_LON, elevation=SITE_ELEV)
                env.set_atmospheric_model(
                    type="custom_atmosphere", pressure=None, temperature=None,
                    wind_u=wind_u,
                    wind_v=wind_v,
                )
                cache[(d, h)] = env

                if geopot_all is not None:
                    tidx2  = tidx % geopot_all.shape[0]
                    z_vals = geopot_all[tidx2]
                    p2_pa  = geopot_levs * 100.0
                    h_geom = np.array([_geopotential_to_geometric(zv) for zv in z_vals])
                    T_est  = np.clip(288.15 - 0.0065 * h_geom, 216.65, 320.0)
                    rho    = p2_pa / (R_AIR * T_est)
                    ord2   = np.argsort(h_geom)
                    density_lut[(d, h)] = list(zip(h_geom[ord2].tolist(), rho[ord2].tolist()))
                else:
                    T_isa   = np.clip(288.15 - 0.0065 * z_wind, 216.65, 320.0)
                    p_isa   = p_pa * (T_isa / 288.15) ** (G0 / (0.0065 * R_AIR))
                    rho_isa = p_isa / (R_AIR * T_isa)
                    density_lut[(d, h)] = list(zip(z_wind.tolist(), rho_isa.tolist()))

            except Exception as e:
                print(f"  [!] ERA5 d={d} h={h}: {e}")

    print(f"  {len(cache)}/{len(days)*len(hours)} environments built")
    return cache, density_lut, wind_raw


# =============================================================================
# PHYSICS: drogue terminal velocity from ERA5 density
# =============================================================================

def parachute_vterm(altitude_msl, mass_kg, density_lut_entry, chute_cd, chute_diam_m, geom, body_cd=0.75):
    """v_t = sqrt(2·m·g / (rho·Cd·A)), rho from ERA5 or ISA fallback."""
    if density_lut_entry:
        alts = np.array([x[0] for x in density_lut_entry])
        rhos = np.array([x[1] for x in density_lut_entry])
        rho  = float(np.interp(altitude_msl, alts, rhos))
    else:
        T   = max(216.65, 288.15 - 0.0065 * altitude_msl)
        P   = 101325.0 * (T / 288.15) ** (G0 / (0.0065 * R_AIR))
        rho = P / (R_AIR * T)
    rho = max(rho, 0.001)
    A_body = math.pi * geom["body_radius"]**2
    A_chute = math.pi * (chute_diam_m / 2) ** 2

    CdA_total = (chute_cd * A_chute) + (body_cd * A_body)

    return math.sqrt(2.0 * mass_kg * G0 / (rho * CdA_total)), rho


def _parachute_physics_metrics(config, apogee_agl, main_alt_agl, mass_kg, density_lut_entry, geom, main_cd=None):
    """Return drogue/main terminal-velocity estimates only for configs that actually deploy those chutes."""
    if config in ("Nominal", "Drogue_Only"):
        v_phys_apo, rho_apo = parachute_vterm(
            apogee_agl + SITE_ELEV, mass_kg, density_lut_entry,
            DROGUE_CD, geom["drogue_diam"], geom)
    else:
        v_phys_apo, rho_apo = 0.0, 0.0

    if config in ("Nominal", "Main_at_Apogee"):
        main_cd_value = main_cd if main_cd is not None else geom["main_cd"]
        v_phys_main, rho_main = parachute_vterm(
            main_alt_agl + SITE_ELEV, mass_kg, density_lut_entry,
            main_cd_value, geom["main_diam"], geom)
    else:
        v_phys_main, rho_main = 0.0, 0.0

    return v_phys_apo, rho_apo, v_phys_main, rho_main


# =============================================================================
# KMZ: trajectories + 5 nmi yellow + 10 nmi red rings around pad
# =============================================================================

NMI_M = 1852.0

def _circle_coords(lat, lon, radius_m, n=128):
    out = []
    for i in range(n + 1):
        b    = 2.0 * math.pi * i / n
        dlat = math.degrees(radius_m / 6_371_000.0 * math.cos(b))
        dlon = math.degrees(radius_m / 6_371_000.0 * math.sin(b) / math.cos(math.radians(lat)))
        out.append((lon + dlon, lat + dlat))
    return out


def _ring_placemark(name, radius_m, color_kml, width=2):
    coords = _circle_coords(SITE_LAT, SITE_LON, radius_m)
    pts    = " ".join(f"{lo:.6f},{la:.6f},0" for lo, la in coords)
    return (f"<Placemark><n>{name}</n>"
            f"<Style><LineStyle><color>{color_kml}</color><width>{width}</width></LineStyle>"
            f"<PolyStyle><fill>0</fill></PolyStyle></Style>"
            f"<Polygon><outerBoundaryIs><LinearRing><altitudeMode>clampToGround</altitudeMode>"
            f"<coordinates>{pts}</coordinates>"
            f"</LinearRing></outerBoundaryIs></Polygon></Placemark>")


def export_kmz(flights_list, filename="monarch_trajectories.kmz"):
    # KML colors are AABBGGRR (NOT RGB)
    CONFIG_COLORS = {
        "Ballistic":      "ff0000ff",  # RED
        "Main_at_Apogee": "ff00ffff",  # YELLOW
        "Drogue_Only":    "ffff0000",  # BLUE
        "Nominal":        "ff00ff00",  # GREEN
    }

    lines = [
        "<?xml version='1.0' encoding='UTF-8'?>",
        "<kml xmlns='http://www.opengis.net/kml/2.2'><Document>",
        "<name>Monarch Flight Ensemble</name>",
        _ring_placemark("5 nmi radius",   5.0 * NMI_M, "ff00ffff", width=2),
        _ring_placemark("10 nmi radius", 10.0 * NMI_M, "ff0000ff", width=2),
    ]

    for i, fd in enumerate(flights_list):
        track = fd.get("track", [])
        if not track:
            continue

        config = fd.get("config", "Nominal")
        color  = CONFIG_COLORS.get(config, "ffffffff")

        # ---- SINGLE ARC PER SIM ----
        coords = " ".join(
            f"{lon:.6f},{lat:.6f},{alt:.1f}"
            for lat, lon, alt in track
        )

        lines.append(
            f"<Placemark>"
            f"<name>Sim{i:04d} {fd.get('config_label','')}</name>"
            f"<Style>"
            f"<LineStyle>"
            f"<color>{color}</color>"
            f"<width>2</width>"
            f"</LineStyle>"
            f"</Style>"
            f"<LineString>"
            f"<altitudeMode>absolute</altitudeMode>"
            f"<coordinates>{coords}</coordinates>"
            f"</LineString>"
            f"</Placemark>"
        )

        # ---- MARKERS (unchanged) ----
        for marker, lak, lok, altv in [
            ("Launch",  "launch_lat", "launch_lon", SITE_ELEV),
            ("Apogee",  "apogee_lat", "apogee_lon", fd.get("apogee_m_agl", 0) + SITE_ELEV),
            ("Landing", "impact_lat", "impact_lon", SITE_ELEV),
        ]:
            if lak in fd:
                lines.append(
                    f"<Placemark>"
                    f"<name>Sim{i:04d} {fd.get('config_label','')} {marker}</name>"
                    f"<Point><altitudeMode>absolute</altitudeMode>"
                    f"<coordinates>{fd[lok]:.6f},{fd[lak]:.6f},{altv:.1f}</coordinates>"
                    f"</Point>"
                    f"</Placemark>"
                )

    lines += ["</Document></kml>"]

    with zipfile.ZipFile(filename, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("doc.kml", "\n".join(lines))

    print(f"  -> {filename}  ({len(flights_list)} trajectories, SIMPLIFIED arcs)")

# =============================================================================
# HTML DASHBOARD EXPORT
# =============================================================================

def export_dashboard(results, html_path="monarch_dashboard.html"):
    """
    Write a self-contained HTML dashboard. No external dependencies except
    Chart.js from cdnjs. Open the output file in any browser.
    """
    if not results:
        print("  [!] No results to dashboard")
        return

    configs    = ["Ballistic", "Main_at_Apogee", "Nominal", "Drogue_Only"]
    clabels    = {"Ballistic":"LB","Main_at_Apogee":"LMA","Nominal":"LN","Drogue_Only":"LDO"}
    chex       = {"Ballistic":"#e24b4a","Main_at_Apogee":"#ef9f27",
                  "Nominal":"#1d9e75","Drogue_Only":"#378add"}
    bhex       = {"Ballistic":"#a32d2d","Main_at_Apogee":"#ba7517",
                  "Nominal":"#0f6e56","Drogue_Only":"#185fa5"}

    def _arr(rows, k): return [float(r[k]) for r in rows]
    def _mean(rows,k):
        arr = _arr(rows, k)
        return float(np.mean(arr)) if arr else 0.0
    def _std(rows,k):
        arr = _arr(rows, k)
        return float(np.std(arr)) if len(arr) > 1 else 0.0
    def _min(rows,k):
        arr = _arr(rows, k)
        return float(np.min(arr)) if arr else 0.0
    def _max(rows,k):
        arr = _arr(rows, k)
        return float(np.max(arr)) if arr else 0.0

    by_cfg = {c: [r for r in results if r["config"] == c] for c in configs}
    st = {}
    for c in configs:
        ro = by_cfg[c]
        st[c] = {
            "n":         len(ro),
            "apo_mean":  _mean(ro,"apogee_m_agl"),
            "apo_std":   _std(ro,"apogee_m_agl"),
            "apo_ft":    _mean(ro,"apogee_m_agl")*3.28084,
            "drift_mean":_mean(ro,"drift_m"),
            "drift_std": _std(ro,"drift_m"),
            "drift_min": _min(ro,"drift_m"),
            "drift_max": _max(ro,"drift_m"),
            "vimp_mean": _mean(ro,"impact_vel_ms"),
            "vimp_std":  _std(ro,"impact_vel_ms"),
            "mach_mean": _mean(ro,"max_mach"),
            "vode_mean": _mean(ro,"drogue_mean_ms"),
            "vphys_mean":_mean(ro,"drogue_physics_ms"),
            "sm_mean":   _mean(ro,"static_margin"),
        }

    all_apo    = [round(st[c]["apo_mean"],1)   for c in configs]
    all_drift  = [round(st[c]["drift_mean"],1)  for c in configs]
    all_vimp   = [round(st[c]["vimp_mean"],2)   for c in configs]
    all_vode   = [round(st[c]["vode_mean"],2)   for c in configs]
    all_vphys  = [round(st[c]["vphys_mean"],2)  for c in configs]
    bar_colors = [chex[c] for c in configs]
    bar_borders= [bhex[c] for c in configs]
    cfg_names  = [f"{clabels[c]} {c.replace(chr(95),' ')}" for c in configs]

    # drift by heading
    hdg_labels = ["N (0 deg)","E (90 deg)","S (180 deg)","W (270 deg)"]
    hdg_data   = []
    for lo, hi in [(340,20),(70,110),(160,200),(250,290)]:
        if lo < hi:
            sub = [r for r in results if lo <= float(r["heading"]) <= hi]
        else:
            sub = [r for r in results if float(r["heading"])>=lo or float(r["heading"])<=hi]
        hdg_data.append(round(_mean(sub,"drift_m"),1) if sub else 0)

    # drift by angle
    ang_labels = ["0 deg","2 deg","4 deg","6 deg","8 deg"]
    ang_data   = []
    for ang in [0,2,4,6,8]:
        sub = [r for r in results if abs(float(r["angle_off_v"])-ang) < 1.0]
        ang_data.append(round(_mean(sub,"drift_m"),1) if sub else 0)

    # apogee by day
    days_present = sorted(set(int(r["day"]) for r in results))
    day_labels   = [f"Day {d}" for d in days_present]
    day_apo      = []
    for d in days_present:
        sub = [r for r in results if int(r["day"])==d]
        day_apo.append(round(_mean(sub,"apogee_m_agl"),1))

    total_sims  = len(results)
    orv_all     = _arr(results,"out_rail_v")
    orv_mean    = round(float(np.mean(orv_all)),2)
    orv_min     = round(float(np.min(orv_all)),2)
    overall_sm  = round(float(np.mean(_arr(results,"static_margin"))),2)
    overall_mach= round(float(np.mean(_arr(results,"max_mach"))),3)
    mean_apo_ft = round(float(np.mean(_arr(results,"apogee_ft_agl"))),0)
    mean_apo_m  = round(float(np.mean(_arr(results,"apogee_m_agl"))),0)

    # config card HTML
    cfg_cards = ""
    for c in configs:
        s = st[c]
        cfg_cards += f"""
    <div class="cc" style="border-left-color:{chex[c]}">
      <p class="cn">{clabels[c]} &mdash; {c.replace("_"," ")}</p>
      <div class="cr"><span>Sims</span><b>{s["n"]}</b></div>
      <div class="cr"><span>Apogee</span><b>{s["apo_mean"]:.0f} &plusmn; {s["apo_std"]:.0f} m</b></div>
      <div class="cr"><span>Apogee</span><b>{s["apo_ft"]:.0f} ft AGL</b></div>
      <div class="cr"><span>Drift mean</span><b>{s["drift_mean"]:.0f} m</b></div>
      <div class="cr"><span>Drift range</span><b>{s["drift_min"]:.0f}&ndash;{s["drift_max"]:.0f} m</b></div>
      <div class="cr"><span>Impact vel</span><b>{s["vimp_mean"]:.1f} &plusmn; {s["vimp_std"]:.1f} m/s</b></div>
      <div class="cr"><span>Static margin</span><b>{s["sm_mean"]:.2f} cal</b></div>
    </div>"""

    legend_html = "".join(
        f'<span class="leg"><span class="ls" style="background:{chex[c]}"></span>{clabels[c]} {c.replace("_"," ")}</span>'
        for c in configs)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>Monarch DOE Dashboard</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js"></script>
<style>
*,*::before,*::after{{box-sizing:border-box}}
body{{margin:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
      background:#f5f4f0;color:#2c2c2a;font-size:14px;line-height:1.5}}
header{{background:#2c2c2a;color:#f1efe8;padding:1.25rem 2rem;display:flex;
        align-items:baseline;gap:1rem}}
header h1{{margin:0;font-size:18px;font-weight:500}}
header span{{font-size:12px;color:#b4b2a9}}
.wrap{{max-width:1100px;margin:0 auto;padding:1.5rem 1.5rem 3rem}}
.mg{{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-bottom:1.75rem}}
.mc{{background:#fff;border:0.5px solid #d3d1c7;border-radius:8px;padding:.9rem 1.1rem}}
.ml{{font-size:11px;color:#888780;margin:0 0 4px;text-transform:uppercase;letter-spacing:.05em}}
.mv{{font-size:22px;font-weight:500;margin:0}}
.ms{{font-size:11px;color:#888780;margin:2px 0 0}}
.sec{{margin:2rem 0 .6rem;font-size:11px;font-weight:500;color:#888780;
      text-transform:uppercase;letter-spacing:.06em}}
.cg{{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-bottom:1.5rem}}
.card{{background:#fff;border:0.5px solid #d3d1c7;border-radius:10px;padding:1rem 1.1rem .8rem}}
.ct{{font-size:12px;font-weight:500;color:#5f5e5a;margin:0 0 10px}}
.cw{{position:relative;width:100%}}
.legend{{display:flex;flex-wrap:wrap;gap:12px;margin-bottom:12px}}
.leg{{display:flex;align-items:center;gap:5px;font-size:11px;color:#5f5e5a}}
.ls{{width:9px;height:9px;border-radius:2px;flex-shrink:0}}
.ccg{{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;margin-bottom:1.5rem}}
.cc{{background:#fff;border:0.5px solid #d3d1c7;border-radius:10px;padding:1rem 1.1rem;border-left:3px solid}}
.cn{{font-size:13px;font-weight:500;margin:0 0 6px}}
.cr{{display:flex;justify-content:space-between;font-size:12px;color:#5f5e5a;
     padding:3px 0;border-bottom:0.5px solid #f1efe8}}
.cr:last-child{{border-bottom:none}}
.cr b{{font-weight:500;color:#2c2c2a}}
@media(max-width:650px){{.cg{{grid-template-columns:1fr}}}}
</style>
</head>
<body>
<header>
  <h1>Monarch 3&quot; Monte Carlo DOE</h1>
  <span>Algonquin College HPRS &mdash; {total_sims} simulations &mdash; RocketPy + ERA5</span>
</header>
<div class="wrap">
  <div class="mg">
    <div class="mc"><p class="ml">Total sims</p><p class="mv">{total_sims:,}</p><p class="ms">4 configs, {total_sims//4} each</p></div>
    <div class="mc"><p class="ml">Mean apogee</p><p class="mv">{mean_apo_ft:,.0f} ft</p><p class="ms">{mean_apo_m:,.0f} m AGL</p></div>
    <div class="mc"><p class="ml">Max Mach</p><p class="mv">{overall_mach}</p></div>
    <div class="mc"><p class="ml">Rail exit</p><p class="mv">{orv_mean} m/s</p><p class="ms">min {orv_min} &mdash; all stable</p></div>
    <div class="mc"><p class="ml">Static margin</p><p class="mv">{overall_sm} cal</p></div>
  </div>
  <div class="legend">{legend_html}</div>
  <div class="cg">
    <div class="card"><p class="ct">Apogee AGL by config (m)</p><div class="cw" style="height:200px"><canvas id="c1"></canvas></div></div>
    <div class="card"><p class="ct">Ground drift by config (m)</p><div class="cw" style="height:200px"><canvas id="c2"></canvas></div></div>
    <div class="card"><p class="ct">Impact velocity by config (m/s)</p><div class="cw" style="height:200px"><canvas id="c3"></canvas></div></div>
    <div class="card"><p class="ct">Drogue descent — ODE vs ERA5 physics (m/s)</p><div class="cw" style="height:200px"><canvas id="c4"></canvas></div></div>
    <div class="card"><p class="ct">Drift by rail heading (m)</p><div class="cw" style="height:200px"><canvas id="c5"></canvas></div></div>
    <div class="card"><p class="ct">Drift by rail angle off-vertical (m)</p><div class="cw" style="height:200px"><canvas id="c6"></canvas></div></div>
    <div class="card"><p class="ct">Mean apogee by launch day (m)</p><div class="cw" style="height:200px"><canvas id="c7"></canvas></div></div>
  </div>
  <p class="sec">Recovery config breakdown</p>
  <div class="ccg">{cfg_cards}</div>
</div>
<script>
const tk={{color:"#888780",font:{{size:11}}}};
const gr={{color:"rgba(136,135,128,0.15)"}};
function bo(lbl,mn){{return{{responsive:true,maintainAspectRatio:false,
  plugins:{{legend:{{display:false}},tooltip:{{callbacks:{{label:c=>" "+c.parsed.y.toLocaleString()+" "+lbl}}}}}},
  scales:{{x:{{ticks:tk,grid:{{display:false}}}},y:{{min:mn,ticks:tk,grid:gr}}}}}};}}
const cl={json.dumps(cfg_names)};
const bc={json.dumps(bar_colors)};
const bb={json.dumps(bar_borders)};
new Chart(document.getElementById("c1"),{{type:"bar",data:{{labels:cl,datasets:[{{data:{json.dumps(all_apo)},backgroundColor:bc,borderColor:bb,borderWidth:1,borderRadius:4}}]}},options:bo("m AGL",{round(min(all_apo)*0.99,0)})}});
new Chart(document.getElementById("c2"),{{type:"bar",data:{{labels:cl,datasets:[{{data:{json.dumps(all_drift)},backgroundColor:bc,borderColor:bb,borderWidth:1,borderRadius:4}}]}},options:bo("m",0)}});
new Chart(document.getElementById("c3"),{{type:"bar",data:{{labels:cl,datasets:[{{data:{json.dumps(all_vimp)},backgroundColor:bc,borderColor:bb,borderWidth:1,borderRadius:4}}]}},options:bo("m/s",0)}});
new Chart(document.getElementById("c4"),{{type:"bar",data:{{labels:cl,datasets:[
  {{label:"ODE sim",data:{json.dumps(all_vode)},backgroundColor:"rgba(55,138,221,0.75)",borderColor:"#185fa5",borderWidth:1,borderRadius:4}},
  {{label:"ERA5 physics",data:{json.dumps(all_vphys)},backgroundColor:"rgba(29,158,117,0.75)",borderColor:"#0f6e56",borderWidth:1,borderRadius:4}}
]}},options:{{responsive:true,maintainAspectRatio:false,plugins:{{legend:{{display:true,labels:{{font:{{size:11}},color:"#888780"}}}}}},scales:{{x:{{ticks:tk,grid:{{display:false}}}},y:{{ticks:tk,grid:gr}}}}}}}});
new Chart(document.getElementById("c5"),{{type:"bar",data:{{labels:{json.dumps(hdg_labels)},datasets:[{{data:{json.dumps(hdg_data)},backgroundColor:"#9FE1CB",borderColor:"#1D9E75",borderWidth:1,borderRadius:4}}]}},options:bo("m",0)}});
new Chart(document.getElementById("c6"),{{type:"line",data:{{labels:{json.dumps(ang_labels)},datasets:[{{data:{json.dumps(ang_data)},borderColor:"#378add",backgroundColor:"rgba(55,138,221,0.1)",tension:0.3,pointRadius:5,pointBackgroundColor:"#378add",fill:true}}]}},options:bo("m",0)}});
new Chart(document.getElementById("c7"),{{type:"bar",data:{{labels:{json.dumps(day_labels)},datasets:[{{data:{json.dumps(day_apo)},backgroundColor:"#AFA9EC",borderColor:"#534AB7",borderWidth:1,borderRadius:4}}]}},options:bo("m AGL",{round(min(day_apo)*0.99,0)})}});
</script>
</body>
</html>"""

    try:
        with open(html_path, "w", encoding="utf-8") as f:
            f.write(html)
        print(f"  -> {html_path}  (open in any browser)")
    except Exception as e:
        print(f"  [!] Dashboard export failed: {e}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("\n" + "=" * 70)
    print("  MONARCH DOE — RocketPy + ERA5")
    print("=" * 70 + "\n")
    print("[1/4] Parsing Monarch.ork ...")
    geom = parse_ork(
        ORK_PATH,
        motor_mass=6.293,
        prop_mass=4.903,
        casing_mass=1.390,
        motor_length=1.025,
        main_diam=MAIN_DIAM_M,
        main_cd=MAIN_CD,
        main_alt_agl=457.2,
        drogue_diam=DROGUE_DIAM_M,
        drogue_cd=DROGUE_CD,
        rasaero_cd=_RASAERO_CD,
        parse_cdplot=_parse_cdplot,
        cd_csv_path=None,
        rasaero_sweep_csv_path=CD_PATH,
    )

    print("\n[2/4] Loading nominal thrust curve ...")
    nominal_thrust = load_eng(ENG_PATH)

    _r, _m, _ = build_rocket_and_motor(geom, nominal_thrust, {})
    sm_wet = _coerce_scalar(_r.static_margin(0))
    sm_dry = _coerce_scalar(_r.static_margin(_m.burn_out_time + 1.0))
    cg_wet = _r.center_of_mass(0)
    cp     = _r.cp_position(0)
    print(f"  Motor : peak={_m.max_thrust:.0f} N  burn={_m.burn_out_time:.2f} s  "
          f"prop={_m.propellant_initial_mass:.3f} kg  (target {geom['prop_mass']:.3f})")
    print(f"  CG={cg_wet:.4f}m  CP={cp:.4f}m  SM_wet={sm_wet:.2f}  SM_dry={sm_dry:.2f}", end="")
    if sm_wet < 1.0 or sm_dry < 1.0:
        print("  <- UNSTABLE")
        raise SystemExit("aborting — fix the geometry before re-running")
    elif sm_wet > 8.0:
        print("  <- HIGH (worth a double check)")
    else:
        print("  OK")
    del _r, _m

    descs = {
        "Ballistic":      "no chutes — lawn dart",
        "Main_at_Apogee": "48\" main @ apogee",
        "Nominal":        "drogue @ apogee + 48\" main @ 1500 ft AGL",
        "Drogue_Only":    "drogue only",
    }
    print(f"\n  Recovery configs:")
    for cfg in CONFIGS:
        print(f"    {CONFIG_LABELS[cfg]:3s}  {cfg:20s}: {descs[cfg]}")

    print(f"\n[3/4] Pre-loading ERA5 wind + geopotential ...")
    wind_cache, density_lut, wind_raw = load_wind_profiles(NC1_PATH, NC2_PATH, DAYS, HOURS)
    if not wind_raw:
        raise SystemExit("no wind environments loaded — check NC file paths")

    N_WORKERS = max(1, (os.cpu_count() or 4) - 1)
    print(f"\n  Warming up {N_WORKERS} worker process(es) (importing rocketpy)...", flush=True)
    executor = _get_executor(N_WORKERS)
    _warm_up_executor(executor, N_WORKERS)

    print(f"  MC samples per condition: {MC_SAMPLES} (single perturbed draw per grid cell)")
    print(f"\n[4/4] Running {TOTAL_SIMS} simulations ...\n" + "-" * 70)

    results, errors, kmz_flights = [], [], []
    n_ok = 0
    n_fail = 0
    t_start = time.perf_counter()
    job_specs = []

    for config in CONFIGS:
        for d in DAYS:
            for h in HOURS:
                if (d, h) not in wind_raw:
                    continue
                wind_u, wind_v = wind_raw[(d, h)]
                dens = density_lut.get((d, h), [])
                for ang in RAIL_ANGLES:
                    for hdg in RAIL_DIRS:
                        for mc in range(MC_SAMPLES):
                            pert = {k: rng.normal(0, s) for k, s in MC_SIGMA.items()}
                            ang_pert = ang + pert["rail_angle_deg"]
                            hdg_pert = (hdg + pert["rail_heading_deg"]) % 360
                            inclination = 90.0 - ang_pert
                            rail_length_guess = float(np.clip(
                                RAIL_LEN + pert.get("rail_length_m", 0.0), *RAIL_LENGTH_RANGE_M))
                            job_specs.append({
                                "config": config,
                                "day": d,
                                "hour": h,
                                "ang": ang,
                                "hdg": hdg,
                                "mc": mc,
                                "pert": pert,
                                "ang_pert": ang_pert,
                                "hdg_pert": hdg_pert,
                                "inclination": inclination,
                                "rail_length_guess": rail_length_guess,
                                "dens": dens,
                                "wind_u": wind_u,
                                "wind_v": wind_v,
                            })

    total_jobs = len(job_specs)
    pending = {}
    max_inflight = max(1, N_WORKERS)
    count = 0
    timeout_s = 30.0

    def _emit_progress(done_count, label=""):
        elapsed = time.perf_counter() - t_start
        rate = done_count / elapsed if elapsed > 1e-3 else 0.0
        eta_min = (total_jobs - done_count) / rate / 60 if rate > 0 else 0.0
        pct = 100.0 * done_count / total_jobs if total_jobs else 0.0
        print(f"\n  [{done_count:>5}/{total_jobs}] {pct:5.1f}%  "
              f"{rate:.1f} sim/s  ETA {eta_min:.0f} min  "
              f"ok={n_ok} fail={n_fail}  ({label})", flush=True)
        if results:
            rec = results[-min(LOG_INTERVAL, len(results)):]
            def _s(k):
                v = [r[k] for r in rec]
                return f"{np.mean(v):.1f}±{np.std(v):.1f}"
            print(f"    last {len(rec)}: "
                  f"apo={_s('apogee_m_agl')}m  "
                  f"drift={_s('drift_m')}m  "
                  f"Mach={_s('max_mach')}  "
                  f"drogue_phys={_s('drogue_physics_ms')}m/s  "
                  f"drogue_ODE={_s('drogue_mean_ms')}m/s",
                  flush=True)

    def _handle_result(fut, job):
        nonlocal count, n_ok
        r = fut.result()
        cos_lat = math.cos(math.radians(SITE_LAT))
        drift_m = math.hypot(r["x_end"], r["y_end"])
        mass_drogue = r["sim_params"]["dry_mass"] + geom["casing_mass"]
        v_phys_apo, rho_apo, v_phys_main, rho_main = _parachute_physics_metrics(
            job["config"],
            r["apogee_agl"],
            r["sim_params"]["main_alt_agl"],
            mass_drogue,
            job["dens"],
            geom,
            main_cd=r["sim_params"].get("main_cd", geom["main_cd"]),
        )

        results.append({
            "config": job["config"],
            "config_label": CONFIG_LABELS[job["config"]],
            "day": job["day"],
            "hour": job["hour"],
            "angle_off_v": round(job["ang_pert"], 2),
            "heading": round(job["hdg_pert"], 1),
            "inclination": round(job["inclination"], 2),
            "mc_sample": job["mc"],
            "p_dry_mass_kg": round(r["sim_params"]["dry_mass"], 3),
            "p_dry_cg_m": round(r["sim_params"]["dry_cg"], 4),
            "p_thrust_scale": round(r["sim_params"]["thrust_scale"], 4),
            "p_cd_scale": round(r["sim_params"]["cd_scale"], 4),
            "p_main_cd_scale": round(r["sim_params"]["main_cd_scale"], 4),
            "p_main_cd": round(r["sim_params"]["main_cd"], 4),
            "p_main_alt_m": round(r["sim_params"]["main_alt_agl"], 1),
            "apogee_m_agl": round(r["apogee_agl"], 1),
            "apogee_ft_agl": round(r["apogee_agl"] * 3.28084, 0),
            "t_apogee": round(r["t_apogee"], 2),
            "max_mach": round(r["max_mach"], 4),
            "out_rail_v": round(r["out_rail_v"], 2),
            "drift_m": round(drift_m, 1),
            "impact_vel_ms": round(r["v_impact"], 2),
            "drogue_mean_ms": round(r["drogue_mean_ms"], 2),
            "drogue_max_ms": round(r["drogue_max_ms"], 2),
            "main_mean_ms": round(r["main_mean_ms"], 2),
            "main_max_ms": round(r["main_max_ms"], 2),
            "drogue_physics_ms": round(v_phys_apo, 2),
            "main_physics_ms": round(v_phys_main, 2),
            "rho_at_apogee": round(rho_apo, 4),
            "rho_at_main_alt": round(rho_main, 4),
            "static_margin": round(r["static_margin"], 3),
            "landing_lat": round(SITE_LAT + r["y_end"] / 111_000.0, 6),
            "landing_lon": round(SITE_LON + r["x_end"] / (111_000.0 * cos_lat), 6),
        })

        kmz_flights.append({
            "config": job["config"],
            "config_label": CONFIG_LABELS[job["config"]],
            "track": [(SITE_LAT + ly / 111_000.0, SITE_LON + lx / (111_000.0 * cos_lat), lz)
                       for (ly, lx, lz) in r["track"]],
            "apogee_m_agl": r["apogee_agl"],
            "launch_lat": SITE_LAT,
            "launch_lon": SITE_LON,
            "apogee_lat": SITE_LAT + r["apogee_y"] / 111_000.0,
            "apogee_lon": SITE_LON + r["apogee_x"] / (111_000.0 * cos_lat),
            "impact_lat": SITE_LAT + r["y_end"] / 111_000.0,
            "impact_lon": SITE_LON + r["x_end"] / (111_000.0 * cos_lat),
        })
        n_ok += 1
        count += 1
        if count % LOG_INTERVAL == 1 or count == total_jobs:
            _emit_progress(count, job["config"])

    def _handle_error(job, exc):
        nonlocal n_fail, count
        n_fail += 1
        err_key = f"{type(exc).__name__}:{str(exc)[:60]}"
        if not hasattr(main, "_seen_errs"):
            main._seen_errs = set()
        if err_key not in main._seen_errs and len(main._seen_errs) < 3:
            main._seen_errs.add(err_key)
            import traceback as _tb
            print(f"\n  [!] {type(exc).__name__}: {str(exc)[:120]}", flush=True)
            _tb.print_exc(limit=4)
        errors.append({
            "config": job["config"],
            "day": job["day"],
            "hour": job["hour"],
            "angle_off_v": round(job["ang_pert"], 2),
            "heading": round(job["hdg_pert"], 1),
            "mc_sample": job["mc"],
            "error_type": type(exc).__name__,
            "error": str(exc)[:200],
            "p_dry_mass_kg": round(job["pert"].get("dry_mass_kg", float("nan")), 3),
            "p_dry_cg_m": round(job["pert"].get("dry_cg_m", float("nan")), 4),
            "p_cd_scale": round(job["pert"].get("cd_scale", float("nan")), 4),
        })
        count += 1
        if count % LOG_INTERVAL == 1 or count == total_jobs:
            _emit_progress(count, job["config"])

    for spec in job_specs:
        while len(pending) >= max_inflight:
            done, _ = wait(pending, timeout=0.25, return_when=FIRST_COMPLETED)
            if not done:
                now = time.perf_counter()
                timed_out = [fut for fut, meta in list(pending.items()) if meta["deadline"] <= now]
                for fut in timed_out:
                    job = pending.pop(fut)
                    try:
                        fut.cancel()
                    except Exception:
                        pass
                    _handle_error(job, TimeoutError(f"sim exceeded {timeout_s}s — solver hang, skipping"))
                continue
            for fut in done:
                job = pending.pop(fut)
                try:
                    _handle_result(fut, job)
                except Exception as exc:
                    _handle_error(job, exc)

        future = executor.submit(
            _sim_worker,
            geom, nominal_thrust, spec["pert"], spec["config"], spec["wind_u"], spec["wind_v"],
            spec["rail_length_guess"], spec["inclination"], spec["hdg_pert"], SITE_ELEV,
        )
        pending[future] = {
            **spec,
            "started_at": time.perf_counter(),
            "deadline": time.perf_counter() + timeout_s,
        }

    while pending:
        done, _ = wait(pending, timeout=0.25, return_when=FIRST_COMPLETED)
        if not done:
            now = time.perf_counter()
            timed_out = [fut for fut, meta in list(pending.items()) if meta["deadline"] <= now]
            for fut in timed_out:
                job = pending.pop(fut)
                try:
                    fut.cancel()
                except Exception:
                    pass
                _handle_error(job, TimeoutError(f"sim exceeded {timeout_s}s — solver hang, skipping"))
            continue
        for fut in done:
            job = pending.pop(fut)
            try:
                _handle_result(fut, job)
            except Exception as exc:
                _handle_error(job, exc)

    executor.shutdown(wait=True, cancel_futures=False)

    elapsed = time.perf_counter() - t_start
    print("\n" + "-" * 70)
    print(f"  {n_ok} ok  |  {n_fail} failed  |  {elapsed:.1f}s  ({elapsed/max(n_ok,1):.2f}s/sim)")

    if results:
        def _stat(k):
            v = [r[k] for r in results]
            return f"{np.mean(v):.2f}±{np.std(v):.2f}  [{np.min(v):.2f}–{np.max(v):.2f}]"
        print(f"\n  ── SUMMARY ──────────────────────────────────────────────────────")
        print(f"  Apogee AGL       : {_stat('apogee_m_agl')} m")
        print(f"  Drift            : {_stat('drift_m')} m")
        print(f"  Max Mach         : {_stat('max_mach')}")
        print(f"  Impact vel       : {_stat('impact_vel_ms')} m/s")
        print(f"  Drogue (ODE)     : {_stat('drogue_mean_ms')} m/s mean  (should be ~37–42 m/s)")
        print(f"  Drogue (physics) : {_stat('drogue_physics_ms')} m/s term. @ apogee alt")
        print(f"  Main physics     : {_stat('main_physics_ms')} m/s term. @ 1500 ft")
        print(f"  rho @ apogee     : {_stat('rho_at_apogee')} kg/m³")

        # try timestamped fallback name if csv is locked
        csv_path = "monarch_doe.csv"
        if os.path.exists(csv_path):
            try:
                os.remove(csv_path)
            except PermissionError:
                csv_path = f"monarch_doe_{int(time.time())}.csv"
                print(f"  [!] monarch_doe.csv is locked — writing to {csv_path}")

        try:
            with open(csv_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=results[0].keys())
                w.writeheader()
                w.writerows(results)
            print(f"\n  -> {csv_path}  ({len(results)} rows)")
        except Exception as e:
            print(f"\n  [!] CSV write failed: {e}")

        if kmz_flights:
            export_kmz(kmz_flights, "monarch_trajectories.kmz")

        export_dashboard(results, "monarch_dashboard.html")

    if errors:
        with open("monarch_errors.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=errors[0].keys())
            w.writeheader()
            w.writerows(errors)
        etypes = {}
        for e in errors:
            etypes.setdefault(e.get("error_type", "?"), []).append(e)
        print(f"\n  Errors ({n_fail} total):")
        for et, errs in sorted(etypes.items()):
            print(f"    {et}: {len(errs)}")
            for e in errs[:2]:
                print(f"      {e['config']} d={e['day']} h={e['hour']}: {e['error'][:80]}")

    print("\nThis took me forever (god help)\n")
# I hope this dashboard is useful to the Monarch team and the rocketry community at large.
# Help.
# Great heavens.
# I will never do this again.
# Modify this at your own risk.
# (Unless you want to add more charts, in which case, please do — it's a pain to get right but the end result is pretty nice. Just follow the existing patterns and be sure to test with a small number of sims first.)
# If you want to run more sims, be sure to adjust the LOG_INTERVAL constant so you get regular progress updates without overwhelming the console. And maybe consider adding some multiprocessing if you're feeling brave.
# Cheers
# I don't run multiprocessing because of the complexity it adds to error handling and progress logging, but if you do add it, please share your implementation with me so I can learn from it! (it scares me)
# if you made it this far, congrats, you absolute madlad. I hope the code is at least somewhat readable and that the comments help clarify the intent behind the various steps. If you have any questions or suggestions for improvement, please don't hesitate to reach out to me on GitHub or via email. I'm always happy to discuss rocketry and code with fellow enthusiasts!
# P.S. If you want to see the raw simulation data, it's all in the results list (and the kmz_flights list for trajectory details). Just be careful with memory if you try to print it all out!
# This eats your CPU for breakfast, lunch, and dinner. I hope you have a good cooling system and maybe some snacks nearby. Remember to take breaks and stay hydrated while running this beast!
# If something breaks, it's your fault, not mine. I'm not responsible for any lost data, corrupted files, or existential crises that may result from running this code. Always make sure to back up your work and maybe test with a smaller number of simulations before going all in.
# If this breaks, it's becaused you touched it. Don't touch it. Seriously, just run it as is and enjoy the results. If you want to modify it, be sure to understand the code first and test your changes thoroughly before running a large batch of simulations.
# This has been tuned for weeks, i can assure you, it works on my machine. If it doesn't work on yours, well, that's unfortunate. Maybe check your Python version, dependencies, and file paths? Or just stare at the code until you understand it and can fix it yourself? Good luck!
# mrrp meow :3
# the KMZ export file is very laggy, open at your own risk.
# Launch Canada 2026! Go big or go home! Project MONARCH for the win! 🚀👑
# - Made by Gurroop, member of the Algonquin College rocketry team, for project Monarch and the wider rocketry community. If you find this code useful, please consider giving it a star on GitHub or sharing it with your rocketry friends. Happy flying!
if __name__ == "__main__":
    main(),