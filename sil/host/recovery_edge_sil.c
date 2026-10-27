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

                    © Copyright of Ignition Software Department
*/

/**************************************************************************************************
 * File:        recovery_edge_sil.c
 * Author:      Vidisha Jain
 * Created On:  27/10/26
 *
 * \brief       Recovery-system edge-case Software-in-the-Loop tests.
 * \details     Exercises the production recovery algorithm against normal
 *              acceleration, incomplete and interrupted low-g sequences,
 *              invalid IMU data, repeated post-deployment samples and varying
 *              altitude to verify immediate, latched, IMU-only deployment.
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
#include <stdbool.h>
#include <string.h>

#include "modules/recovery.h"
#include "servo_mock.h"

static int sPassed = 0;
static int sFailed = 0;

static void Check(bool condition, const char *name)
{
    if (condition)
    {
        printf("[PASS] %s\n", name);
        sPassed++;
    }
    else
    {
        printf("[FAIL] %s\n", name);
        sFailed++;
    }
}

static void FeedImu(float accel_g, bool valid, uint32_t time_ms)
{
    SmData data;

    memset(&data, 0, sizeof(data));

    data.imu.accel_x_g = 0.0f;
    data.imu.accel_y_g = 0.0f;
    data.imu.accel_z_g = accel_g;
    data.imu_valid = valid;
    data.imu_timestamp_ms = time_ms;

    (void)Recovery_Update(&data, time_ms);
}

static void ResetTest(void)
{
    (void)Recovery_Init();
    SilServoMock_Reset();
}

static void TestNormalOneGDoesNotDeploy(void)
{
    ResetTest();

    for (uint32_t i = 0; i < 100; i++)
    {
        FeedImu(1.0f, true, i * 20U);
    }

    Check(!Recovery_IsDeployed(),
          "Normal 1 g motion does not deploy recovery");

    Check(SilServoMock_GetDeployCount() == 0U,
          "Servo is not commanded during normal 1 g");
}

static void TestTwoLowGSamplesDoNotDeploy(void)
{
    ResetTest();

    FeedImu(0.1f, true, 0U);
    FeedImu(0.1f, true, 20U);

    Check(!Recovery_IsDeployed(),
          "Two low-g samples do not deploy recovery");

    Check(SilServoMock_GetDeployCount() == 0U,
          "Servo remains untouched before confirmation");
}

static void TestThreeLowGSamplesDeploy(void)
{
    ResetTest();

    FeedImu(0.1f, true, 0U);
    FeedImu(0.1f, true, 20U);
    FeedImu(0.1f, true, 40U);

    Check(Recovery_IsDeployed(),
          "Three consecutive low-g samples deploy recovery");

    Check(SilServoMock_GetDeployCount() == 1U,
          "Confirmed free-fall commands servo exactly once");
}

static void TestInterruptedLowGDoesNotDeploy(void)
{
    ResetTest();

    FeedImu(0.1f, true, 0U);
    FeedImu(0.1f, true, 20U);

    /* Break the consecutive sequence. */
    FeedImu(1.0f, true, 40U);

    FeedImu(0.1f, true, 60U);
    FeedImu(0.1f, true, 80U);

    Check(!Recovery_IsDeployed(),
          "Interrupted low-g sequence is rejected");

    Check(SilServoMock_GetDeployCount() == 0U,
          "Interrupted sequence does not command servo");
}

static void TestInvalidImuResetsConfirmation(void)
{
    ResetTest();

    FeedImu(0.1f, true, 0U);
    FeedImu(0.1f, true, 20U);

    /* Invalid reading must destroy the partial confirmation. */
    FeedImu(0.0f, false, 40U);

    FeedImu(0.1f, true, 60U);
    FeedImu(0.1f, true, 80U);

    Check(!Recovery_IsDeployed(),
          "Invalid IMU sample resets free-fall confirmation");

    FeedImu(0.1f, true, 100U);

    Check(Recovery_IsDeployed(),
          "Fresh three-sample sequence deploys after invalid IMU");
}

static void TestDeploymentIsLatched(void)
{
    ResetTest();

    FeedImu(0.1f, true, 0U);
    FeedImu(0.1f, true, 20U);
    FeedImu(0.1f, true, 40U);

    for (uint32_t t = 60U; t <= 1000U; t += 20U)
    {
        FeedImu(0.1f, true, t);
    }

    Check(Recovery_IsDeployed(),
          "Recovery remains deployed after trigger");

    Check(SilServoMock_GetDeployCount() == 1U,
          "Latched recovery never commands servo twice");
}

static void TestAltitudeIsIrrelevant(void)
{
    SmData data;

    ResetTest();

    memset(&data, 0, sizeof(data));

    /*
     * Deliberately absurd altitude values.
     * Recovery must depend on IMU free-fall, not altitude.
     */
    data.baro_valid = true;
    data.imu_valid = true;
    data.imu.accel_z_g = 0.1f;

    data.baro_altitude_m = 0.0f;
    (void)Recovery_Update(&data, 0U);

    data.baro_altitude_m = 100.0f;
    (void)Recovery_Update(&data, 20U);

    data.baro_altitude_m = 50.0f;
    (void)Recovery_Update(&data, 40U);

    Check(Recovery_IsDeployed(),
          "Recovery deployment has no altitude dependency");

    Check(SilServoMock_GetDeployCount() == 1U,
          "Altitude changes do not interfere with free-fall deployment");
}

int main(void)
{
    printf("CONSCIENTIA SIL - Recovery Edge Cases\n");
    printf("========================================================\n");

    TestNormalOneGDoesNotDeploy();
    TestTwoLowGSamplesDoNotDeploy();
    TestThreeLowGSamplesDeploy();
    TestInterruptedLowGDoesNotDeploy();
    TestInvalidImuResetsConfirmation();
    TestDeploymentIsLatched();
    TestAltitudeIsIrrelevant();

    printf("========================================================\n");
    printf("Passed: %d\n", sPassed);
    printf("Failed: %d\n", sFailed);

    if (sFailed == 0)
    {
        printf("ALL RECOVERY EDGE-CASE TESTS PASSED\n");
        return 0;
    }

    printf("RECOVERY EDGE-CASE SIL FAILED\n");
    return 1;
}