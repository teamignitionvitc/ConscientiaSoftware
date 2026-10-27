import csv
import socket
import time
from pathlib import Path

CSV_PATH = Path(__file__).parent / "data" / "simulated_flight.csv"

HOST = "127.0.0.1"
PORT = 5000

PHASE_MAP = {
    "PRE_LAUNCH": 0,
    "ASCENT": 1,
    "FREEFALL": 1,
    "DESCENT": 2,
    "LANDED": 3,
}

RECOVERY_DEPLOY_TIME_S = 25.06


def fused_lux(row):
    return (
        float(row["lux1"])
        + float(row["lux2"])
        + float(row["lux3"])
    ) / 3.0


def make_packet(row, sequence):
    t = float(row["time_s"])
    altitude = float(row["altitude_m"])

    values = [
        sequence,
        int(t * 1000),
        float(row["temperature_c"]),
        int(float(row["pressure_pa"])),
        altitude,

        float(row["accel_x_g"]),
        float(row["accel_y_g"]),
        float(row["accel_z_g"]),
        float(row["gyro_x_dps"]),
        float(row["gyro_y_dps"]),
        float(row["gyro_z_dps"]),

        float(row["lux1"]),
        float(row["lux2"]),
        float(row["lux3"]),
        fused_lux(row),

        float(row["pm1_ugm3"]),
        float(row["pm25_ugm3"]),
        float(row["pm10_ugm3"]),

        0,
        float(row["air_quality_response"]),

        float(row["latitude"]),
        float(row["longitude"]),
        altitude,
        float(row["ground_speed_mps"]),
        8,

        PHASE_MAP[row["expected_phase"]],
        1 if t >= RECOVERY_DEPLOY_TIME_S else 0,
        255,
    ]

    return ",".join(str(v) for v in values) + "\n"


def main():
    print(f"Loading: {CSV_PATH}")

    with CSV_PATH.open(newline="") as file:
        rows = list(csv.DictReader(file))
        
    print(f"Loaded {len(rows)} SIL samples.")
    print(f"Waiting for GLANCE on TCP {HOST}:{PORT} ...")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)

        conn, address = server.accept()

        with conn:
            print(f"GLANCE connected: {address}")
            print("Replaying simulated flight...")

            previous_time = float(rows[0]["time_s"])

            for sequence, row in enumerate(rows):
                current_time = float(row["time_s"])

                if sequence > 0:
                    delay = current_time - previous_time
                    if delay > 0:
                        time.sleep(delay)

                packet = make_packet(row, sequence)
                conn.sendall(packet.encode("ascii"))

                previous_time = current_time

            print("Replay complete.")


if __name__ == "__main__":
    main()