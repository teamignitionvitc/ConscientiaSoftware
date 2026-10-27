import csv
from pathlib import Path

import matplotlib.pyplot as plt

DATA_DIR = Path("sil/data/sd_logs")


def load_xy(filename):
    path = DATA_DIR / filename

    with path.open(newline="") as file:
        reader = csv.reader(file)
        header = next(reader)

        x = []
        y = []

        for row in reader:
            if len(row) >= 2:
                x.append(float(row[0]))
                y.append(float(row[1]))

    return header[0], header[1], x, y


GRAPHS = [
    ("altitude_time.csv", "Altitude vs Time"),
    ("pressure_alt.csv", "Pressure vs Altitude"),
    ("temp_alt.csv", "Temperature vs Altitude"),
    ("accel_z_alt.csv", "Acceleration Z vs Altitude"),
    ("gyro_z_alt.csv", "Gyroscope Z vs Altitude"),
    ("pm25_alt.csv", "PM2.5 vs Altitude"),
    ("airqual_alt.csv", "Air Quality vs Altitude"),
    ("light_alt.csv", "Fused Light vs Altitude"),
    ("windspd_alt.csv", "Wind Speed vs Altitude"),
    ("winddir_alt.csv", "Wind Direction vs Altitude"),
]


for filename, title in GRAPHS:
    x_name, y_name, x, y = load_xy(filename)

    plt.figure()
    plt.plot(x, y)
    plt.title(title)
    plt.xlabel(x_name)
    plt.ylabel(y_name)
    plt.grid(True)
    plt.tight_layout()

plt.show()