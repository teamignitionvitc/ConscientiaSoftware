'''
                                        ::
                                        ::
                                        ::
                                        ::
                                        ::
    ..    ..........    :.      ::      ::     .........  ..    ..........    ...      .
    ::    ::            : .:.   ::     .::.       ::      ::    ::       :    :: :.    :
    ::    ::   ..:::    :   .:. ::    ::::::      ::      ::    ::       :    ::   ::  :
    ::    ::......::    :      :::    ::::::      ::      ::    ::.......:    ::     :::
                                      ::::::
                                      :.::.:
                         .::::          ::          ::::
                       .::::::::.       ::       .:::::::::
                       ::::::::::::....::::.....:::::::::::
                        .:::::::::::::::::::::::::::::::::.

                    © Copyright of Ignition Software Department
'''
# File:        test_production_sil.py
# Author:      Vidisha Jain
# Created On:  27/10/26
#
# Brief:
#   Python ctypes runner for the CONSCIENTIA production SIL bridge.
#
# Note:
#   Retained as SIL infrastructure. On the current development machine the
#   MinGW host compiler is 32-bit while Python is 64-bit, so the standalone
#   production_sil.exe runner is used instead.

import csv
import ctypes
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DLL_PATH = ROOT / "host" / "conscientia_sil.dll"
DATA_PATH = ROOT / "data" / "simulated_flight.csv"


class SilResult(ctypes.Structure):
    _fields_ = [
        ("phase", ctypes.c_int),
        ("altitude_agl_m", ctypes.c_float),
        ("apogee_m", ctypes.c_float),
        ("vertical_rate_mps", ctypes.c_float),
        ("accel_mag_g", ctypes.c_float),
        ("recovery_deployed", ctypes.c_bool),
        ("recovery_reason", ctypes.c_int),
        ("recovery_detect_time_ms", ctypes.c_uint32),
        ("recovery_deploy_time_ms", ctypes.c_uint32),
        ("servo_deploy_count", ctypes.c_uint32),
    ]


lib = ctypes.CDLL(str(DLL_PATH))

lib.Sil_Init.argtypes = []
lib.Sil_Init.restype = None

lib.Sil_Update.argtypes = [
    ctypes.c_float,
    ctypes.c_bool,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_bool,
    ctypes.c_uint32,
]
lib.Sil_Update.restype = None

lib.Sil_GetResult.argtypes = [ctypes.POINTER(SilResult)]
lib.Sil_GetResult.restype = None


def get_result():
    result = SilResult()
    lib.Sil_GetResult(ctypes.byref(result))
    return result


def check(condition, message):
    if condition:
        print(f"[PASS] {message}")
        return True

    print(f"[FAIL] {message}")
    return False


def main():
    print("CONSCIENTIA SIL - Production C Validation")
    print("=" * 56)

    lib.Sil_Init()

    phases_seen = []
    recovery_first_seen_ms = None
    samples = 0

    with DATA_PATH.open(newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            now_ms = int(round(float(row["time_s"]) * 1000.0))

            lib.Sil_Update(
                float(row["altitude_m"]),
                True,
                float(row["accel_x_g"]),
                float(row["accel_y_g"]),
                float(row["accel_z_g"]),
                True,
                now_ms,
            )

            result = get_result()
            samples += 1

            if not phases_seen or phases_seen[-1] != result.phase:
                phases_seen.append(result.phase)

            if result.recovery_deployed and recovery_first_seen_ms is None:
                recovery_first_seen_ms = now_ms

    result = get_result()

    passed = True

    passed &= check(samples > 0, f"Processed {samples} simulated samples")

    passed &= check(
        phases_seen == [0, 1, 2, 3],
        f"Flight states progressed PRE_LAUNCH -> ASCENT -> DESCENT -> LANDED "
        f"(observed {phases_seen})",
    )

    passed &= check(
        result.phase == 3,
        "Final production flight state is LANDED",
    )

    passed &= check(
        result.apogee_m >= 90.0,
        f"Production flight-state apogee is plausible ({result.apogee_m:.2f} m AGL)",
    )

    passed &= check(
        result.recovery_deployed,
        "Production recovery logic deployed",
    )

    passed &= check(
        result.recovery_reason == 1,
        "Recovery deployment reason is FREEFALL",
    )

    passed &= check(
        result.servo_deploy_count == 1,
        f"Servo deployment occurred exactly once "
        f"(count={result.servo_deploy_count})",
    )

    passed &= check(
        result.recovery_detect_time_ms == result.recovery_deploy_time_ms,
        "Recovery triggers servo immediately after confirmation",
    )

    if recovery_first_seen_ms is not None:
        print(
            f"       Recovery observed : "
            f"{recovery_first_seen_ms / 1000.0:.3f} s"
        )

        print(
            f"       Detect timestamp  : "
            f"{result.recovery_detect_time_ms / 1000.0:.3f} s"
        )

        print(
            f"       Deploy timestamp  : "
            f"{result.recovery_deploy_time_ms / 1000.0:.3f} s"
        )

    passed &= check(
        recovery_first_seen_ms is not None
        and recovery_first_seen_ms <= 28000,
        "Recovery occurs well inside the 3000 ms release-response budget",
    )

    print("=" * 56)

    if passed:
        print("ALL PRODUCTION C SIL TESTS PASSED")
        return 0

    print("PRODUCTION C SIL TEST FAILED")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())