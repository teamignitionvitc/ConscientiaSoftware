import csv
import math
from pathlib import Path

INPUT = Path("sil/data/simulated_flight.csv")
OUTPUT = Path("sil/data/sd_logs")

OUTPUT.mkdir(parents=True, exist_ok=True)


def write_csv(filename, header, rows):
    path = OUTPUT / filename

    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(rows)

    print(f"Generated: {path}")


with INPUT.open(newline="") as file:
    data = list(csv.DictReader(file))


# Launch altitude reference
base_altitude = float(data[0]["altitude_m"])


def agl(row):
    return float(row["altitude_m"]) - base_altitude


# ---------------------------------------------------------------------------
# Altitude
# ---------------------------------------------------------------------------

write_csv(
    "altitude_time.csv",
    ["timestamp_ms", "alt_agl_m"],
    [
        [
            round(float(r["time_s"]) * 1000),
            agl(r),
        ]
        for r in data
    ],
)


# ---------------------------------------------------------------------------
# Environmental
# ---------------------------------------------------------------------------

write_csv(
    "pressure_alt.csv",
    ["alt_agl_m", "pressure_pa"],
    [[agl(r), float(r["pressure_pa"])] for r in data],
)

write_csv(
    "temp_alt.csv",
    ["alt_agl_m", "temp_cdegc"],
    [
        [
            agl(r),
            round(float(r["temperature_c"]) * 100),
        ]
        for r in data
    ],
)


# ---------------------------------------------------------------------------
# Accelerometer
# ---------------------------------------------------------------------------

for axis in ("x", "y", "z"):
    write_csv(
        f"accel_{axis}_alt.csv",
        ["alt_agl_m", f"accel_{axis}_g"],
        [
            [
                agl(r),
                float(r[f"accel_{axis}_g"]),
            ]
            for r in data
        ],
    )


# ---------------------------------------------------------------------------
# Gyroscope
# ---------------------------------------------------------------------------

for axis in ("x", "y", "z"):
    write_csv(
        f"gyro_{axis}_alt.csv",
        ["alt_agl_m", f"gyro_{axis}_dps"],
        [
            [
                agl(r),
                float(r[f"gyro_{axis}_dps"]),
            ]
            for r in data
        ],
    )


# ---------------------------------------------------------------------------
# Particulate matter
# ---------------------------------------------------------------------------

pm_fields = {
    "pm1_alt.csv": ("pm1_0_ugm3", "pm1_ugm3"),
    "pm25_alt.csv": ("pm2_5_ugm3", "pm25_ugm3"),
    "pm10_alt.csv": ("pm10_ugm3", "pm10_ugm3"),
}

for filename, (output_field, input_field) in pm_fields.items():
    write_csv(
        filename,
        ["alt_agl_m", output_field],
        [
            [
                agl(r),
                float(r[input_field]),
            ]
            for r in data
        ],
    )


# ---------------------------------------------------------------------------
# Air quality
# ---------------------------------------------------------------------------

write_csv(
    "airqual_alt.csv",
    ["alt_agl_m", "aq_response"],
    [
        [
            agl(r),
            float(r["air_quality_response"]),
        ]
        for r in data
    ],
)


# ---------------------------------------------------------------------------
# Light fusion
# ---------------------------------------------------------------------------

write_csv(
    "light_alt.csv",
    ["alt_agl_m", "fused_lux"],
    [
        [
            agl(r),
            (
                float(r["lux1"])
                + float(r["lux2"])
                + float(r["lux3"])
            ) / 3.0,
        ]
        for r in data
    ],
)


# ---------------------------------------------------------------------------
# Wind approximation
# ---------------------------------------------------------------------------

write_csv(
    "windspd_alt.csv",
    ["alt_agl_m", "wind_speed_mps"],
    [
        [
            agl(r),
            float(r["ground_speed_mps"]),
        ]
        for r in data
    ],
)

write_csv(
    "winddir_alt.csv",
    ["alt_agl_m", "wind_dir_deg"],
    [
        [
            agl(r),
            float(r["course_deg"]),
        ]
        for r in data
    ],
)


# ---------------------------------------------------------------------------
# GNSS trajectory
# ---------------------------------------------------------------------------

write_csv(
    "gnss_track.csv",
    [
        "timestamp_ms",
        "lat",
        "lon",
        "alt_m",
        "speed_mps",
        "course_deg",
        "sats",
    ],
    [
        [
            round(float(r["time_s"]) * 1000),
            float(r["latitude"]),
            float(r["longitude"]),
            float(r["altitude_m"]),
            float(r["ground_speed_mps"]),
            float(r["course_deg"]),
            8,
        ]
        for r in data
    ],
)

print()
print(f"Generated {len(list(OUTPUT.glob('*.csv')))} SD-style CSV files.")