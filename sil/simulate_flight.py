#                                         ::
#                                         ::
#                                         ::
#                                         ::
#                                         ::
#     ..    ..........    :.      ::      ::     .........  ..    ..........    ...      .
#     ::    ::            : .:.   ::     .::.       ::      ::    ::       :    :: :.    :
#     ::    ::   ..:::    :   .:. ::    ::::::      ::      ::    ::       :    ::   ::  :
#     ::    ::......::    :      :::    ::::::      ::      ::    ::.......:    ::     :::
#                                       ::::::
#                                       :.::.:
#                          .::::          ::          ::::
#                        .::::::::.       ::       .:::::::::
#                        ::::::::::::....::::.....:::::::::::
#                         .:::::::::::::::::::::::::::::::::.
#
#                     © Copyright of Ignition Software Department
#
####################################################################################################
# File:        simulate_flight.py
# Author:      Vidisha Jain
# Created On:  27/10/26
#
# Brief:
#   Software-in-the-Loop flight data generator for the CONSCIENTIA CanSat.
#
# Description:
#   Generates deterministic synthetic sensor data representing a complete
#   CONSCIENTIA mission profile for Software-in-the-Loop (SIL) testing.
#
#   Simulated mission sequence:
#
#       PRE_LAUNCH
#           |
#           v
#       ASCENT (0 -> ~100 m)
#           |
#           v
#       RELEASE / FREEFALL
#           |
#           v
#       PARACHUTE DESCENT
#           |
#           v
#       LANDED
#
#   The simulator generates representative measurements for:
#
#       - BMP280       : altitude, pressure, temperature
#       - MPU6050      : acceleration and angular velocity
#       - PMS5003      : PM1.0, PM2.5 and PM10
#       - MQ135        : synthetic air-quality response
#       - BH1750 x3    : light intensity
#       - L89HA GNSS   : latitude, longitude, speed and course
#
#   Samples are generated at 50 Hz to match the primary IMU sampling rate
#   and written to:
#
#       sil/data/simulated_flight.csv
#
#   This file only generates test INPUT data. Mission decisions such as
#   flight-state transitions, recovery deployment and payload processing
#   are intentionally not performed here. Those behaviours are validated
#   separately against the actual flight-software modules.
#
####################################################################################################
# HISTORY:
#
#       +----- (NEW | MODify | ADD | DELete)
#       |
# No#   |       when       who                  what
######+*********+**********+********************+***************************************************
# 000  NEW      27/10/26   Vidisha Jain         Initial SIL flight-profile generator
####################################################################################################

import csv
import math
import random
from pathlib import Path


# Simulation settings
DT = 0.02                  # 50 Hz, matches IMU rate
TOTAL_TIME_S = 55.0
MAX_ALTITUDE_M = 100.0

OUTPUT_DIR = Path(__file__).parent / "data"
OUTPUT_FILE = OUTPUT_DIR / "simulated_flight.csv"

# Arbitrary starting position for simulated GNSS.
# This is synthetic test data, not a real launch location.
START_LAT = 12.000000
START_LON = 80.000000


def pressure_from_altitude(altitude_m):
    """Approximate standard-atmosphere pressure in Pa."""
    sea_level_pressure = 101325.0
    return sea_level_pressure * (1.0 - altitude_m / 44330.0) ** 5.255


def generate_sample(t):
    """Generate one synthetic sensor sample."""

    # ---------------------------------------------------------
    # Mission phase / altitude
    # ---------------------------------------------------------

    if t < 5.0:
        phase = "PRE_LAUNCH"
        altitude = 0.0

    elif t < 25.0:
        phase = "ASCENT"
        progress = (t - 5.0) / 20.0
        altitude = MAX_ALTITUDE_M * progress

    elif t < 26.0:
        phase = "FREEFALL"
        altitude = MAX_ALTITUDE_M - (t - 25.0) * 5.0

    elif t < 50.0:
        phase = "DESCENT"
        progress = (t - 26.0) / 24.0
        altitude = 95.0 * (1.0 - progress)

    else:
        phase = "LANDED"
        altitude = 0.0

    altitude += random.gauss(0.0, 0.15)
    altitude = max(0.0, altitude)

    # ---------------------------------------------------------
    # BMP280
    # ---------------------------------------------------------

    pressure = pressure_from_altitude(altitude)
    pressure += random.gauss(0.0, 5.0)

    temperature = 28.0 - 0.0065 * altitude
    temperature += random.gauss(0.0, 0.1)

    # ---------------------------------------------------------
    # MPU6050
    # ---------------------------------------------------------

    if phase == "FREEFALL":
        # Low apparent acceleration during release/drop.
        ax = random.gauss(0.0, 0.03)
        ay = random.gauss(0.0, 0.03)
        az = random.gauss(0.10, 0.03)

    else:
        ax = random.gauss(0.0, 0.03)
        ay = random.gauss(0.0, 0.03)
        az = random.gauss(1.0, 0.03)

    if phase == "DESCENT":
        gx = random.gauss(10.0, 4.0)
        gy = random.gauss(8.0, 4.0)
        gz = random.gauss(5.0, 3.0)
    else:
        gx = random.gauss(0.0, 1.0)
        gy = random.gauss(0.0, 1.0)
        gz = random.gauss(0.0, 1.0)

    # ---------------------------------------------------------
    # PMS5003
    # ---------------------------------------------------------

    pm1 = 8.0 + 0.025 * altitude + random.gauss(0.0, 0.4)
    pm25 = 12.0 + 0.035 * altitude + random.gauss(0.0, 0.5)
    pm10 = 18.0 + 0.045 * altitude + random.gauss(0.0, 0.7)

    # ---------------------------------------------------------
    # MQ135 synthetic response
    # ---------------------------------------------------------

    air_quality_response = (
        30.0
        + 0.04 * altitude
        + random.gauss(0.0, 0.5)
    )

    # ---------------------------------------------------------
    # BH1750 x3
    # ---------------------------------------------------------

    base_lux = 35000.0 + 20.0 * altitude

    lux1 = base_lux + random.gauss(0.0, 300.0)
    lux2 = base_lux * 0.85 + random.gauss(0.0, 300.0)
    lux3 = base_lux * 0.90 + random.gauss(0.0, 300.0)

    # ---------------------------------------------------------
    # GNSS
    # ---------------------------------------------------------

    if phase == "DESCENT":
        ground_speed = 2.0 + random.gauss(0.0, 0.15)
        course = 60.0 + random.gauss(0.0, 2.0)
    else:
        ground_speed = max(0.0, random.gauss(0.05, 0.03))
        course = 60.0

    distance = ground_speed * t

    north = distance * math.cos(math.radians(course))
    east = distance * math.sin(math.radians(course))

    latitude = START_LAT + north / 111320.0

    longitude = START_LON + east / (
        111320.0 * math.cos(math.radians(START_LAT))
    )

    return {
        "time_s": round(t, 3),
        "expected_phase": phase,

        "altitude_m": round(altitude, 3),
        "pressure_pa": round(pressure, 2),
        "temperature_c": round(temperature, 2),

        "accel_x_g": round(ax, 4),
        "accel_y_g": round(ay, 4),
        "accel_z_g": round(az, 4),

        "gyro_x_dps": round(gx, 3),
        "gyro_y_dps": round(gy, 3),
        "gyro_z_dps": round(gz, 3),

        "pm1_ugm3": round(pm1, 2),
        "pm25_ugm3": round(pm25, 2),
        "pm10_ugm3": round(pm10, 2),

        "air_quality_response": round(air_quality_response, 2),

        "lux1": round(lux1, 2),
        "lux2": round(lux2, 2),
        "lux3": round(lux3, 2),

        "latitude": round(latitude, 7),
        "longitude": round(longitude, 7),
        "ground_speed_mps": round(ground_speed, 3),
        "course_deg": round(course, 2),
    }


def main():
    random.seed(42)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    samples = []

    t = 0.0
    while t <= TOTAL_TIME_S:
        samples.append(generate_sample(t))
        t += DT

    with OUTPUT_FILE.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=samples[0].keys(),
        )

        writer.writeheader()
        writer.writerows(samples)

    print(f"Generated {len(samples)} samples")
    print(f"Flight data written to: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()