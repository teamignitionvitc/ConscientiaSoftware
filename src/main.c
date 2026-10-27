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
                         .::::          ::          ::::.                                       
                       .::::::::.       ::       .:::::::::                                     
                       ::::::::::::....::::.....:::::::::::                                     
                        .:::::::::::::::::::::::::::::::::.         

                    © Copyright of Ignition Software Department 
*/

/**************************************************************************************************
 * File:        main.c
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * Brief:
 *   Main entry point for CONSCIENTIA CanSat flight software.
 *
 * Description:
 *   Super-loop architecture:
 *
 *     INIT:
 *       1. stdio / clocks
 *       2. Servo  (stowed position)
 *       3. SensorManager  (I2C + all 8 sensors)
 *       4. FlightState
 *       5. Recovery
 *       6. DataLogger  (SD + FAT mount + open CSVs)
 *       7. Telemetry   (LoRa E32 init)
 *       8. Payload modules  (air_quality, particulate, light, wind)
 *
 *     LOOP (every tick, ~1-5 ms):
 *       1. SensorManager_Update()          — poll all sensors
 *       2. SensorManager_GetData()         — snapshot
 *       3. FlightState_Update()            — phase machine
 *       4. Recovery_Update()               — DROP detection + servo
 *       5. Payload processing              — AQ, PM, light, wind
 *       6. CSV logging (rate-limited)      — one CSV per GLANCE graph
 *       7. Telemetry_Update() (rate-limited) — LoRa downlink
 *       8. DataLogger_Sync() (rate-limited) — f_sync
 *
 *   Recovery has the HIGHEST priority — it runs before any logging or
 *   telemetry so that Servo_Deploy() fires within the 3 s budget.
 *
 *   Module init failures are logged but do NOT halt the system.  Each
 *   subsystem checks its own init flag before operating.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 * 001  MOD      26/10/26   Vidisha Jain         Full avionics integration
 **************************************************************************************************/

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include "pico/stdlib.h"
#include "pico/time.h"
#include "config.h"

#include <string.h>

/* Hardware drivers */
#include "drivers/servo.h"

/* Modules */
#include "modules/sensor_manager.h"
#include "modules/flight_state.h"
#include "modules/recovery.h"
#include "modules/data_logger.h"
#include "modules/telemetry.h"

/* Payload */
#include "payload/air_quality.h"
#include "payload/particulate_analysis.h"
#include "payload/light_fusion.h"
#include "payload/wind_inference.h"

/**************************************************************************************************
 * Static Variables — Module-ready flags
 **************************************************************************************************/
static bool gSensorOk   = false;
static bool gFlightOk   = false;
static bool gRecoveryOk = false;
static bool gLoggerOk   = false;
static bool gTelemetryOk = false;
static bool gAirQualOk  = false;
static bool gPartOk     = false;
static bool gLightOk    = false;
static bool gWindOk     = false;

/* Logging rate-limit timestamps (1 CSV = 1 graph) */
static uint32_t gLastBaroLogMs  = 0;   /* altitude, pressure, temp */
static uint32_t gLastImuLogMs   = 0;   /* accel, gyro              */
static uint32_t gLastPmLogMs    = 0;   /* PM vs alt                */
static uint32_t gLastAqLogMs    = 0;   /* AQ response vs alt       */
static uint32_t gLastLightLogMs = 0;   /* fused lux vs alt         */
static uint32_t gLastWindLogMs  = 0;   /* wind speed/dir vs alt    */
static uint32_t gLastGnssLogMs  = 0;   /* GNSS trajectory          */

/**************************************************************************************************
 * Static Helpers
 **************************************************************************************************/

/**
 * \brief   Map FlightPhase to LfMotionHint (decoupled enum).
 */
static LfMotionHint PhaseToMotion(FlightPhase p)
{
    switch (p)
    {
        case FS_PRE_LAUNCH: return LF_MOTION_STATIC;
        case FS_ASCENT:     return LF_MOTION_DYNAMIC;
        case FS_DESCENT:    return LF_MOTION_DYNAMIC;
        case FS_LANDED:     return LF_MOTION_STATIC;
        default:            return LF_MOTION_UNKNOWN;
    }
}

/**
 * \brief   Returns true if interval has elapsed since *last, and updates it.
 */
static bool RateLimitCheck(uint32_t now_ms, uint32_t *last, uint32_t interval)
{
    if ((now_ms - *last) >= interval)
    {
        *last = now_ms;
        return true;
    }
    return false;
}

/**************************************************************************************************
 * Initialisation
 **************************************************************************************************/
static void SystemInit(void)
{
    /* ── 1. stdio / USB debug ──────────────────────────────────────── */
    stdio_init_all();
    sleep_ms(100);   /* Let USB enumerate for debug output */

    /* ── 2. Servo — stow immediately ───────────────────────────────── */
    (void)Servo_Init();
    (void)Servo_Stow();

    /* ── 3. Sensor Manager (I2C buses + all 8 sensors) ─────────────── */
    SmResult smr = SensorManager_Init();
    gSensorOk = (smr == SM_OK || smr == SM_ERR_INIT_PARTIAL);
    /* Partial OK — some sensors may have failed but we continue */

    /* ── 4. Flight State ───────────────────────────────────────────── */
    gFlightOk = (FlightState_Init() == FS_OK);

    /* ── 5. Recovery ───────────────────────────────────────────────── */
    gRecoveryOk = (Recovery_Init() == REC_OK);

    /* ── 6. Data Logger (SD + FAT + CSVs) ──────────────────────────── */
    if (DataLogger_Init() == DL_OK)
    {
        gLoggerOk = (DataLogger_Open() == DL_OK);
    }

    /* ── 7. Telemetry (LoRa E32) ───────────────────────────────────── */
    gTelemetryOk = (Telemetry_Init() == TM_OK);

    /* ── 8. Payload modules ────────────────────────────────────────── */
    gAirQualOk = (AirQuality_Init() == AQ_OK);
    gPartOk    = (PartAnalysis_Init() == PA_OK);
    gLightOk   = (LightFusion_Init() == LF_OK);
    gWindOk    = (WindInfer_Init() == WI_OK);
}

/**************************************************************************************************
 * Main Loop
 **************************************************************************************************/
int main(void)
{
    SystemInit();

    while (true)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* ════════════════════════════════════════════════════════════
         * 1. SENSOR MANAGER — poll all sensors (non-blocking)
         * ════════════════════════════════════════════════════════════ */
        if (gSensorOk)
        {
            SensorManager_Update();
        }

        /* Snapshot the latest data for all downstream consumers */
        SmData sm;
        if (gSensorOk)
        {
            SensorManager_GetData(&sm);
        }
        else
        {
            /* Zero-init so downstream code sees all-invalid */
            memset(&sm, 0, sizeof(sm));
        }

        /* ════════════════════════════════════════════════════════════
         * 2. FLIGHT STATE — evaluate phase transitions
         * ════════════════════════════════════════════════════════════ */
        if (gFlightOk)
        {
            FlightState_Update(&sm, now);
        }

        FsMetrics fsm;
        FlightPhase phase = FS_PRE_LAUNCH;
        float alt_agl     = 0.0f;

        if (gFlightOk)
        {
            FlightState_GetMetrics(&fsm);
            phase   = fsm.phase;
            alt_agl = fsm.alt_agl_m;
        }

        /* ════════════════════════════════════════════════════════════
         * 3. RECOVERY — highest priority, before any I/O
         * ════════════════════════════════════════════════════════════ */
        if (gRecoveryOk)
        {
            Recovery_Update(&sm, now);
        }

        /* ════════════════════════════════════════════════════════════
         * 4. PAYLOAD PROCESSING
         * ════════════════════════════════════════════════════════════ */

        /* Air Quality */
        AqData aq;
        bool aqFresh = false;
        if (gAirQualOk && RateLimitCheck(now, &gLastAqLogMs, SAMPLE_INTERVAL_AIR_MS))
        {
            aqFresh = (AirQuality_Read(&aq) == AQ_OK);
        }

        /* Particulate Analysis */
        PaData pa;
        bool paFresh = false;
        if (gPartOk)
        {
            PartAnalysis_Update(&sm, alt_agl);
            if (sm.pm_valid)
            {
                PartAnalysis_GetData(&pa);
                paFresh = pa.paired_valid;
            }
        }

        /* Light Fusion */
        LfData lf;
        bool lfFresh = false;
        if (gLightOk)
        {
            LightFusion_Update(&sm, alt_agl, PhaseToMotion(phase));
            if (LightFusion_GetData(&lf) == LF_OK)
            {
                lfFresh = lf.fused_valid;
            }
        }

        /* Wind Inference */
        WiData wi;
        bool wiFresh = false;
        if (gWindOk)
        {
            WindInfer_Update(&sm, alt_agl, now);
            if (WindInfer_GetData(&wi) == WI_OK)
            {
                wiFresh = wi.sample_valid;
            }
        }

        /* ════════════════════════════════════════════════════════════
         * 5. CSV LOGGING — 1 CSV = 1 graph, rate-limited
         * ════════════════════════════════════════════════════════════ */
        if (gLoggerOk)
        {
            /* Baro group: altitude_time, pressure_alt, temp_alt */
            if (sm.baro_valid &&
                RateLimitCheck(now, &gLastBaroLogMs, SAMPLE_INTERVAL_BARO_MS))
            {
                DataLogger_LogAltTime(now, alt_agl);
                DataLogger_LogPressure(alt_agl, sm.baro.pressure_pa);
                DataLogger_LogTemp(alt_agl, sm.baro.temperature_cdegc);
            }

            /* IMU group: accel_alt, gyro_alt */
            if (sm.imu_valid &&
                RateLimitCheck(now, &gLastImuLogMs, SAMPLE_INTERVAL_IMU_MS))
            {
                DataLogger_LogAccelX(alt_agl, sm.imu.accel_x_g);
                DataLogger_LogAccelY(alt_agl, sm.imu.accel_y_g);
                DataLogger_LogAccelZ(alt_agl, sm.imu.accel_z_g);
                DataLogger_LogGyroX(alt_agl, sm.imu.gyro_x_dps);
                DataLogger_LogGyroY(alt_agl, sm.imu.gyro_y_dps);
                DataLogger_LogGyroZ(alt_agl, sm.imu.gyro_z_dps);
            }

            /* PM vs altitude */
            if (paFresh &&
                RateLimitCheck(now, &gLastPmLogMs, SAMPLE_INTERVAL_PM_MS))
            {
                DataLogger_LogPM1(alt_agl, pa.pm1_0_atm);
                DataLogger_LogPM25(alt_agl, pa.pm2_5_atm);
                DataLogger_LogPM10(alt_agl, pa.pm10_atm);
            }

            /* AQ response vs altitude */
            if (aqFresh)
            {
                DataLogger_LogAirQual(alt_agl, aq.air_quality_response);
            }

            /* Fused lux vs altitude */
            if (lfFresh &&
                RateLimitCheck(now, &gLastLightLogMs, SAMPLE_INTERVAL_LIGHT_MS))
            {
                DataLogger_LogLight(alt_agl, lf.fused_lux);
            }

            /* Wind speed + direction vs altitude */
            if (wiFresh &&
                RateLimitCheck(now, &gLastWindLogMs, SAMPLE_INTERVAL_GNSS_MS))
            {
                DataLogger_LogWindSpd(alt_agl, wi.wind_speed_mps);
                DataLogger_LogWindDir(alt_agl, wi.wind_dir_deg);
            }

            /* GNSS trajectory */
            if (sm.gnss_valid && sm.gnss.fix_valid &&
                RateLimitCheck(now, &gLastGnssLogMs, SAMPLE_INTERVAL_GNSS_MS))
            {
                DataLogger_LogGnss(now,
                    sm.gnss.latitude_deg, sm.gnss.longitude_deg,
                    sm.gnss.altitude_m, sm.gnss.speed_mps,
                    sm.gnss.course_deg, sm.gnss.satellites);
            }
        }

        /* ════════════════════════════════════════════════════════════
         * 6. TELEMETRY — LoRa downlink (internally rate-limited)
         * ════════════════════════════════════════════════════════════ */
        if (gTelemetryOk)
        {
            Telemetry_Update(&sm, now);
        }

        /* ════════════════════════════════════════════════════════════
         * 7. DATA LOGGER SYNC — periodic f_sync to SD
         * ════════════════════════════════════════════════════════════ */
        if (gLoggerOk)
        {
            DataLogger_Sync(now);
        }
    }

    /* Never reached — for completeness */
    if (gLoggerOk) { DataLogger_Close(); }
    return 0;
}