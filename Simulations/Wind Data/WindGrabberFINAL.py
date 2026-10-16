#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ERA5 WIND PROCESSOR - AREA AVERAGED
Extracts wind data averaged over 10 nautical mile diameter around launch site
Processes multiple NetCDF files from CDS
"""

import os
import sys
import numpy as np
import pandas as pd
import xarray as xr
from scipy.interpolate import interp1d

# =============================================================================
# CONFIGURATION
# =============================================================================

LATITUDE = 47.99
LONGITUDE = -81.85
RADIUS_NM = 15.0  # 15 nautical miles = 30 NM diameter
RADIUS_DEG = RADIUS_NM * (1.0 / 60.0)  # Convert NM to degrees (approximately)

# Directory where NCF files are located
DATA_DIR = r"C:\Users\jakwm\OneDrive - Algonquin College\Monarch 3'' Sim\Python Scripts\WindGrabberV4"

INPUT_FILES = [
    os.path.join(DATA_DIR, "CDFNET.nc"),
]

DOWNLOADS = os.path.join(os.path.expanduser("~"), "Downloads")
OUTPUT_FILE = os.path.join(DOWNLOADS, "WindGrabber_ERA5_Aug15-21_2025.csv")

# Rocket ascent parameters
# Based on Cesaroni M2245-IM-P motor
# RasAero CDX1 simulation data:
#   Launch weight: 29 lbs (13.15 kg)
#   Motor burn time: ~4.37 seconds
#   Max velocity: 2640.5 ft/s = 805 m/s (Mach 2.4)
#   Max altitude: 36,556 ft = 11,143 m
#
# Velocity profile during ascent:
#   Rail exit (~0.5s): ~100-150 m/s
#   Early burn (1s): ~200-250 m/s
#   Mid burn (2s): ~400-500 m/s
#   Late burn (3s): ~600-700 m/s
#   Burnout (~4.4s): ~805 m/s (Mach 2.4)
#   Coast to apogee: 805 m/s → 0 m/s
#
# Multiple scenarios for comprehensive fin AoA analysis:
ASCENT_VELOCITIES_MPS = {
    'rail_exit': 150,      # Rail exit to early flight
    'early_burn': 250,     # First half of burn
    'mid_burn': 400,       # Transonic region
    'late_burn': 650,      # High subsonic to supersonic
    'burnout': 805,        # Peak velocity (Mach 2.4)
}

# For this processing run, use mid-burn velocity (worst case for fin loading)
# This represents peak dynamic pressure region (transonic)
ASCENT_RATE_MPS = 400.0  # m/s (Mid-burn, transonic region, highest fin loads)

TARGET_ALTITUDES_M = np.array([
    10,
    100,
    250,
    1600 * 0.3048,
    2500 * 0.3048,
    3300 * 0.3048,
    5000 * 0.3048,
    6500 * 0.3048,
    8000 * 0.3048,
    10000 * 0.3048,
    12000 * 0.3048,
    14000 * 0.3048,
    18000 * 0.3048,
    30000 * 0.3048,
    40000 * 0.3048,
    100000 * 0.3048,
])


# =============================================================================
# FUNCTIONS
# =============================================================================


def pressure_to_altitude_m(p_hpa):
    """Convert pressure to altitude using ISA model"""
    return 44330.0 * (1.0 - (p_hpa / 1013.25) ** 0.1903)


def calculate_rocket_aoa(u_wind, v_wind, vertical_wind, ascent_rate):
    """
    Calculate angle of attack for rocket fin modeling
    
    ERA5/ECMWF wind units: All velocities in meters per second (m/s)
    
    Args:
        u_wind: East-West wind component (m/s, positive = eastward) [ERA5 native units]
        v_wind: North-South wind component (m/s, positive = northward) [ERA5 native units]
        vertical_wind: Vertical wind component (m/s, positive = upward) [ERA5 native units]
        ascent_rate: Rocket vertical velocity (m/s, positive = upward)
    
    Returns:
        Dictionary with:
            - u_component_mps: East-West wind (m/s) [ERA5 units]
            - v_component_mps: North-South wind (m/s) [ERA5 units]
            - vertical_wind_mps: Vertical wind (m/s) [ERA5 units]
            - horizontal_wind_speed_mps: Magnitude of horizontal wind (m/s)
            - wind_direction_deg: Meteorological wind direction (0-360°)
            - relative_vertical_speed_mps: Rocket vertical speed relative to air (m/s)
            - total_relative_wind_mps: Total 3D wind speed relative to rocket (m/s)
            - angle_of_attack_deg: AoA measured from horizontal x-axis (0° = horizontal, 90° = vertical)
    
    Note: ERA5 uses m/s for all velocity components (u, v, w). No unit conversion needed.
    """
    # Individual wind components (already in m/s from ERA5)
    u_component = float(u_wind)
    v_component = float(v_wind)
    vertical_component = float(vertical_wind)
    
    # Horizontal wind magnitude
    horizontal_wind_speed = np.sqrt(u_wind**2 + v_wind**2)
    
    # Meteorological wind direction (direction FROM which wind blows, 0-360°)
    wind_direction = (270.0 - np.degrees(np.arctan2(v_wind, u_wind))) % 360.0
    
    # Rocket's vertical speed relative to the air mass
    # If vertical_wind = +2 m/s (updraft) and ascent_rate = 5 m/s, 
    # then relative_vertical_speed = 5 - 2 = 3 m/s (rocket going up through rising air)
    relative_vertical_speed = ascent_rate - vertical_wind
    
    # Total 3D wind velocity relative to the rocket
    # This is the actual wind the rocket "feels"
    total_relative_wind = np.sqrt(
        u_wind**2 + v_wind**2 + relative_vertical_speed**2
    )
    
    # Angle of Attack measured from horizontal x-axis
    # 0° = rocket moving purely horizontally
    # 90° = rocket moving purely vertically
    # Formula: AoA = arctan(vertical_component / horizontal_component)
    if horizontal_wind_speed > 0.01:
        # Normal case: rocket has both horizontal and vertical components
        aoa_deg = np.degrees(np.arctan2(relative_vertical_speed, horizontal_wind_speed))
    else:
        # Edge case: no horizontal wind = purely vertical flight
        aoa_deg = 90.0
    
    return {
        "u_component_mps": u_component,
        "v_component_mps": v_component,
        "vertical_wind_mps": vertical_component,
        "horizontal_wind_speed_mps": float(horizontal_wind_speed),
        "wind_direction_deg": float(wind_direction),
        "relative_vertical_speed_mps": float(relative_vertical_speed),
        "total_relative_wind_mps": float(total_relative_wind),
        "angle_of_attack_deg": float(aoa_deg),
    }


# =============================================================================
# MAIN
# =============================================================================


def main():
    degree_symbol = "\N{DEGREE SIGN}"
    print("\n" + "=" * 80)
    print("ERA5 WIND PROCESSOR - AREA AVERAGED")
    print("=" * 80)
    print(f"Launch site: {LATITUDE}{degree_symbol}N, {LONGITUDE}{degree_symbol}W")
    print(f"Averaging radius: {RADIUS_NM} NM ({RADIUS_NM * 2} NM diameter)")
    print(f"Data directory: {DATA_DIR}")
    print("=" * 80)

    # Check which files exist
    print("\nLooking for NetCDF files...")
    existing_files = []
    for input_file in INPUT_FILES:
        filename = os.path.basename(input_file)
        if os.path.exists(input_file):
            file_size = os.path.getsize(input_file) / (1024 * 1024)
            existing_files.append(input_file)
            print(f"  ✓ Found: {filename} ({file_size:.2f} MB)")
        else:
            print(f"  ✗ Missing: {filename}")
    
    if not existing_files:
        print("\n" + "=" * 80)
        print("ERROR: No NetCDF files found!")
        print("=" * 80)
        print(f"\nLooking in: {DATA_DIR}")
        print("\nPlease ensure NCF1.nc, NCF2.nc, etc. are in this directory.")
        print("=" * 80)
        input("\nPress Enter to exit...")
        sys.exit(1)
    
    print(f"\n✓ Found {len(existing_files)} file(s), processing...\n")

    all_records = []

    for file_idx, input_file in enumerate(existing_files, 1):
        print("-" * 80)
        print(f"FILE {file_idx}/{len(existing_files)}: {os.path.basename(input_file)}")
        print("-" * 80)

        try:
            ds = xr.open_dataset(input_file)
            print("✓ Loaded")
        except Exception as e:
            print(f"✗ ERROR loading: {e}")
            continue

        # Get coordinate names
        if "pressure_level" in ds.coords:
            press_coord = "pressure_level"
        elif "level" in ds.coords:
            press_coord = "level"
        else:
            print(f"✗ ERROR: No pressure coordinate found")
            print(f"Available coords: {list(ds.coords)}")
            ds.close()
            continue

        pressure_levels = ds[press_coord].values
        print(f"Pressure levels: {pressure_levels}")

        # Convert longitude if needed (ERA5 uses 0-360)
        lon_use = (
            LONGITUDE
            if ds.longitude.min() < 0
            else (LONGITUDE + 360 if LONGITUDE < 0 else LONGITUDE)
        )

        # Define area bounds (10 NM diameter = 5 NM radius)
        lat_min = LATITUDE - RADIUS_DEG
        lat_max = LATITUDE + RADIUS_DEG
        lon_min = lon_use - RADIUS_DEG
        lon_max = lon_use + RADIUS_DEG

        print(f"Extracting area:")
        print(f"  Center: {LATITUDE}{degree_symbol}N, {lon_use}{degree_symbol}W")
        print(f"  Bounds: {lat_min:.2f}{degree_symbol}N to {lat_max:.2f}{degree_symbol}N")
        print(f"          {lon_min:.2f}{degree_symbol}E to {lon_max:.2f}{degree_symbol}W")

        # Select area and average
        try:
            # Find nearest grid point first
            ds_point = ds.sel(
                latitude=LATITUDE,
                longitude=lon_use,
                method="nearest",
            )

            # Get actual grid point coordinates
            lat0 = float(ds_point.latitude)
            lon0 = float(ds_point.longitude)

            # Expand to a box around that point
            ds_area = ds.sel(
                latitude=slice(lat0 + RADIUS_DEG, lat0 - RADIUS_DEG),
                longitude=slice(lon0 - RADIUS_DEG, lon0 + RADIUS_DEG),
            )

            print(f"  Grid points: {len(ds_area.latitude)} x {len(ds_area.longitude)}")
        except Exception as e:
            print(f"✗ ERROR selecting area: {e}")
            ds.close()
            continue

        # Average over the area
        print("  Computing spatial average...")
        ds_avg = ds_area.mean(dim=["latitude", "longitude"])
        print("✓ Area averaged")

        # Convert pressure to altitude
        altitudes_from_pressure = pressure_to_altitude_m(pressure_levels)
        sort_idx = np.argsort(altitudes_from_pressure)
        sorted_altitudes = altitudes_from_pressure[sort_idx]

        print(f"Altitude range: {sorted_altitudes.min():.0f}m - {sorted_altitudes.max():.0f}m")

        # Filter targets
        valid_targets = TARGET_ALTITUDES_M[
            (TARGET_ALTITUDES_M >= sorted_altitudes.min())
            & (TARGET_ALTITUDES_M <= sorted_altitudes.max())
        ]
        print(f"Valid target altitudes: {len(valid_targets)}")

        # Process data
        print(f"Processing {len(ds_avg.valid_time)} timesteps...")
        records = []

        for t_idx in range(len(ds_avg.valid_time)):
            time_val = ds_avg.valid_time[t_idx].values

            # Get area-averaged u and v at all pressure levels
            try:
                u_all_levels = ds_avg.u.isel(valid_time=t_idx).values
                v_all_levels = ds_avg.v.isel(valid_time=t_idx).values
            except Exception as e:
                print(f"✗ ERROR extracting wind at timestep {t_idx}: {e}")
                continue

            # Check for vertical wind component (w)
            if "w" in ds_avg.data_vars:
                try:
                    w_all_levels = ds_avg.w.isel(valid_time=t_idx).values
                except:
                    w_all_levels = np.zeros_like(u_all_levels)
            else:
                w_all_levels = np.zeros_like(u_all_levels)

            # Debug first timestep of first file
            if t_idx == 0 and file_idx == 1:
                print("\nDEBUG - First timestep (area-averaged):")
                print(f"  u values: {u_all_levels}")
                print(f"  v values: {v_all_levels}")
                print(f"  w values: {w_all_levels}")
                wind_speeds = np.sqrt(u_all_levels**2 + v_all_levels**2)
                print(f"  Wind speeds: {wind_speeds}")
                print(f"  Wind speed range: {wind_speeds.min():.2f} - {wind_speeds.max():.2f} m/s")
                print()

            # Sort by altitude
            u_profile = u_all_levels[sort_idx]
            v_profile = v_all_levels[sort_idx]
            w_profile = w_all_levels[sort_idx]

            # Interpolate to target altitudes
            try:
                f_u = interp1d(
                    sorted_altitudes,
                    u_profile,
                    kind="linear",
                    bounds_error=False,
                    fill_value="extrapolate",
                )
                f_v = interp1d(
                    sorted_altitudes,
                    v_profile,
                    kind="linear",
                    bounds_error=False,
                    fill_value="extrapolate",
                )
                f_w = interp1d(
                    sorted_altitudes,
                    w_profile,
                    kind="linear",
                    bounds_error=False,
                    fill_value="extrapolate",
                )

                u_interp = f_u(valid_targets)
                v_interp = f_v(valid_targets)
                w_interp = f_w(valid_targets)
            except Exception as e:
                print(f"✗ ERROR interpolating at timestep {t_idx}: {e}")
                continue

            # Calculate AoA for each altitude
            for alt_idx, alt_m in enumerate(valid_targets):
                u = float(u_interp[alt_idx])
                v = float(v_interp[alt_idx])
                w = float(w_interp[alt_idx])

                aoa_params = calculate_rocket_aoa(u, v, w, ASCENT_RATE_MPS)

                records.append({
                    "time_utc": str(time_val)[:19],
                    "altitude_m": round(alt_m, 1),
                    "altitude_ft": round(alt_m / 0.3048, 0),
                    "u_component_mps": round(aoa_params["u_component_mps"], 3),
                    "v_component_mps": round(aoa_params["v_component_mps"], 3),
                    "vertical_wind_mps": round(aoa_params["vertical_wind_mps"], 3),
                    "horizontal_wind_mps": round(aoa_params["horizontal_wind_speed_mps"], 3),
                    "wind_direction_deg": round(aoa_params["wind_direction_deg"], 2),
                    "ascent_rate_mps": round(ASCENT_RATE_MPS, 3),
                    "relative_vertical_mps": round(aoa_params["relative_vertical_speed_mps"], 3),
                    "total_relative_wind_mps": round(aoa_params["total_relative_wind_mps"], 3),
                    "angle_of_attack_deg": round(aoa_params["angle_of_attack_deg"], 2),
                })

            if (t_idx + 1) % 24 == 0 or (t_idx + 1) == len(ds_avg.valid_time):
                print(f"  Progress: {t_idx + 1}/{len(ds_avg.valid_time)}")

        print(f"✓ Processed {len(records)} data points")
        all_records.extend(records)
        ds.close()

    print("\n" + "=" * 80)
    print(f"TOTAL: {len(all_records)} data points from {len(existing_files)} file(s)")
    print("=" * 80)

    if not all_records:
        print("\n✗ ERROR: No data was processed!")
        input("\nPress Enter to exit...")
        sys.exit(1)

    # Create DataFrame
    df = pd.DataFrame(all_records)
    
    # Sort by time and altitude
    df = df.sort_values(['time_utc', 'altitude_m']).reset_index(drop=True)

    # Statistics
    print("\n" + "-" * 80)
    print("STATISTICS:")
    print("-" * 80)
    print(f"Time range: {df['time_utc'].min()} to {df['time_utc'].max()}")
    print(f"Altitude range: {df['altitude_m'].min():.0f}m - {df['altitude_m'].max():.0f}m")
    print(f"Horizontal wind: {df['horizontal_wind_mps'].min():.1f} - {df['horizontal_wind_mps'].max():.1f} m/s")
    print(f"Mean horizontal wind: {df['horizontal_wind_mps'].mean():.1f} m/s")
    print(f"AoA range: {df['angle_of_attack_deg'].min():.1f}{degree_symbol} - {df['angle_of_attack_deg'].max():.1f}{degree_symbol}")
    print(f"Mean AoA: {df['angle_of_attack_deg'].mean():.1f}{degree_symbol}")
    print("-" * 80)

    # Save to CSV
    os.makedirs(DOWNLOADS, exist_ok=True)
    df.to_csv(OUTPUT_FILE, index=False)

    file_size = os.path.getsize(OUTPUT_FILE) / 1024
    print(f"\n✓ Saved: {OUTPUT_FILE} ({file_size:.1f} KB)")

    print("\nFirst 10 rows:")
    print(df.head(10).to_string(index=False))

    print("\nLast 10 rows:")
    print(df.tail(10).to_string(index=False))

    print("\n" + "=" * 80)
    print("PROCESSING COMPLETE")
    print("=" * 80 + "\n")

    # Auto-open
    try:
        if os.name == "nt":
            os.startfile(OUTPUT_FILE)
            print("✓ File opened\n")
    except Exception:
        pass
    
    input("Press Enter to exit...")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user")
        input("\nPress Enter to exit...")
        sys.exit(0)
    except Exception as e:
        print(f"\n\n✗ FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")
        sys.exit(1)