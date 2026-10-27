/*
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

                    © Copyright of Ignition Avionics Department
*/
/**************************************************************************************************
 * File:        production_sil.c
 * Author:      Vidisha Jain
 * Created On:  27/10/26
 *
 * \brief       Full-mission host Software-in-the-Loop validation runner.
 * \details     Replays the synthetic CONSCIENTIA flight dataset through the
 *              production flight-state and recovery modules. Validates mission
 *              phase progression, apogee tracking, free-fall recovery timing,
 *              immediate deployment and one-shot servo behaviour.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      27/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/
 
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "modules/flight_state.h"
#include "modules/recovery.h"
#include "servo_mock.h"

#define CSV_PATH "sil/data/simulated_flight.csv"
#define LINE_SIZE 1024

static int Check(bool condition, const char *message)
{
    if (condition)
    {
        printf("[PASS] %s\n", message);
        return 1;
    }

    printf("[FAIL] %s\n", message);
    return 0;
}

int main(void)
{
    FILE *file;
    char line[LINE_SIZE];

    unsigned int samples = 0U;
    unsigned int previous_phase = 999U;
    unsigned int phases[10];
    unsigned int phase_count = 0U;

    uint32_t recovery_first_seen_ms = 0U;
    bool recovery_seen = false;

    FsMetrics fs;
    RecStatus rec;

    printf("CONSCIENTIA SIL - Production C Validation\n");
    printf("========================================================\n");

    (void)FlightState_Init();
    (void)Recovery_Init();
    SilServoMock_Reset();

    file = fopen(CSV_PATH, "r");

    if (file == NULL)
    {
        printf("[FAIL] Could not open %s\n", CSV_PATH);
        return 1;
    }

    /* Skip CSV header. */
    if (fgets(line, sizeof(line), file) == NULL)
    {
        printf("[FAIL] CSV is empty\n");
        fclose(file);
        return 1;
    }

    while (fgets(line, sizeof(line), file) != NULL)
    {
        /*
         * Expected leading columns from simulate_flight.py:
         *
         * time_s,phase,altitude_m,pressure_pa,temperature_c,
         * accel_x_g,accel_y_g,accel_z_g,...
         */

        char *token;
        unsigned int column = 0U;

        float time_s = 0.0f;
        float altitude_m = 0.0f;
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;

        SmData data;
        memset(&data, 0, sizeof(data));

        token = strtok(line, ",");

        while (token != NULL)
        {
            switch (column)
            {
                case 0:
                    time_s = (float)atof(token);
                    break;

                case 2:
                    altitude_m = (float)atof(token);
                    break;

                case 5:
                    ax = (float)atof(token);
                    break;

                case 6:
                    ay = (float)atof(token);
                    break;

                case 7:
                    az = (float)atof(token);
                    break;

                default:
                    break;
            }

            column++;
            token = strtok(NULL, ",");
        }

        uint32_t now_ms = (uint32_t)(time_s * 1000.0f + 0.5f);

        data.baro_altitude_m   = altitude_m;
data.baro_timestamp_ms = now_ms;

/*
 * BMP280 runs at 10 Hz while the SIL dataset runs at 50 Hz.
 * Mark only every fifth sample as a fresh barometric reading.
 */
data.baro_valid = ((samples % 5U) == 0U);

        data.imu.accel_x_g     = ax;
        data.imu.accel_y_g     = ay;
        data.imu.accel_z_g     = az;
        data.imu_timestamp_ms  = now_ms;
        data.imu_valid         = true;

        /*
         * Same important ordering as the flight loop:
         * flight-state processing followed immediately by recovery.
         */
        (void)FlightState_Update(&data, now_ms);
        (void)Recovery_Update(&data, now_ms);

        FlightPhase phase = FlightState_GetPhase();

        if ((unsigned int)phase != previous_phase)
        {
            if (phase_count < 10U)
            {
                phases[phase_count++] = (unsigned int)phase;
            }

            previous_phase = (unsigned int)phase;
        }

        if (Recovery_IsDeployed() && !recovery_seen)
        {
            recovery_seen = true;
            recovery_first_seen_ms = now_ms;
        }

        samples++;
    }

    fclose(file);

    memset(&fs, 0, sizeof(fs));
    memset(&rec, 0, sizeof(rec));

    (void)FlightState_GetMetrics(&fs);
    (void)Recovery_GetStatus(&rec);

    printf("       Phase count       : %u\n", phase_count);
printf("       Phase sequence    : ");

for (unsigned int i = 0U; i < phase_count; i++)
{
    printf("%u ", phases[i]);
}

printf("\n");
printf("       Final phase       : %u\n", (unsigned int)fs.phase);
printf("       Final vert rate   : %.3f m/s\n", fs.vert_rate_mps);
printf("       Final accel mag   : %.3f g\n", fs.accel_mag_g);
printf("       Final altitude    : %.3f m AGL\n", fs.alt_agl_m);

    int passed = 1;

    char message[160];

    snprintf(
        message,
        sizeof(message),
        "Processed %u simulated samples",
        samples
    );
    passed &= Check(samples > 0U, message);

    bool correct_sequence =
        phase_count == 4U &&
        phases[0] == FS_PRE_LAUNCH &&
        phases[1] == FS_ASCENT &&
        phases[2] == FS_DESCENT &&
        phases[3] == FS_LANDED;

    passed &= Check(
        correct_sequence,
        "Flight states progressed PRE_LAUNCH -> ASCENT -> DESCENT -> LANDED"
    );

    passed &= Check(
        fs.phase == FS_LANDED,
        "Final production flight state is LANDED"
    );

    snprintf(
        message,
        sizeof(message),
        "Production apogee is plausible (%.2f m AGL)",
        fs.apogee_m
    );
    passed &= Check(fs.apogee_m >= 90.0f, message);

    passed &= Check(
        rec.deployed,
        "Production recovery logic deployed"
    );

    passed &= Check(
        rec.reason == REC_REASON_FREEFALL,
        "Recovery reason is FREEFALL"
    );

    snprintf(
        message,
        sizeof(message),
        "Servo deployment occurred exactly once (count=%u)",
        (unsigned int)SilServoMock_GetDeployCount()
    );
    passed &= Check(
        SilServoMock_GetDeployCount() == 1U,
        message
    );

    passed &= Check(
        rec.detect_time_ms == rec.deploy_time_ms,
        "Recovery triggers servo immediately after confirmation"
    );

    if (recovery_seen)
    {
        printf(
            "       Recovery observed : %.3f s\n",
            recovery_first_seen_ms / 1000.0
        );

        printf(
            "       Detect timestamp  : %.3f s\n",
            rec.detect_time_ms / 1000.0
        );

        printf(
            "       Deploy timestamp  : %.3f s\n",
            rec.deploy_time_ms / 1000.0
        );
    }

    /*
     * Synthetic FREEFALL begins at 25 s.
     * Deployment must therefore occur no later than 28 s.
     */
    passed &= Check(
        recovery_seen &&
        recovery_first_seen_ms >= 25000U &&
        recovery_first_seen_ms <= 28000U,
        "Recovery occurs within 3000 ms of simulated release"
    );

    printf("========================================================\n");

    if (passed)
    {
        printf("ALL PRODUCTION C SIL TESTS PASSED\n");
        return 0;
    }

    printf("PRODUCTION C SIL TEST FAILED\n");
    return 1;
}