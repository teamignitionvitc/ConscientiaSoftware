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
 * File:        ground_station.ino
 * Author:      Vidisha Jain
 * Created On:  27/10/26
 *
 * \brief       CONSCIENTIA ESP32 LoRa ground-station firmware.
 * \details     Receives telemetry from the E32-433T3D through UART2,
 *              validates the CONSCIENTIA XOR checksum and forwards valid
 *              packets over USB Serial for GLANCE.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      27/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

/*==================================================================================================
 * INCLUDES
 *================================================================================================*/

#include <Arduino.h>

/*==================================================================================================
 * CONFIGURATION
 *================================================================================================*/

#define GS_USB_BAUD            115200

#define E32_UART_BAUD          9600
#define E32_RX_PIN             16
#define E32_TX_PIN             17

#define E32_M0_PIN             25
#define E32_M1_PIN             26
#define E32_AUX_PIN            27

#define TELEMETRY_BUFFER_SIZE  256

/*==================================================================================================
 * STATIC DATA
 *================================================================================================*/

static char     sPacket[TELEMETRY_BUFFER_SIZE];
static uint16_t sPacketIndex = 0U;

/*==================================================================================================
 * STATIC FUNCTION DECLARATIONS
 *================================================================================================*/

static bool ValidatePacket(const char *packet);
static int  HexValue(char c);

/*==================================================================================================
 * SETUP
 *================================================================================================*/

void setup()
{
    Serial.begin(GS_USB_BAUD);

    pinMode(E32_M0_PIN, OUTPUT);
    pinMode(E32_M1_PIN, OUTPUT);
    pinMode(E32_AUX_PIN, INPUT);

    /*
     * E32 normal transparent transmission mode.
     */
    digitalWrite(E32_M0_PIN, LOW);
    digitalWrite(E32_M1_PIN, LOW);

    Serial2.begin(
        E32_UART_BAUD,
        SERIAL_8N1,
        E32_RX_PIN,
        E32_TX_PIN
    );

    sPacketIndex = 0U;
}

/*==================================================================================================
 * MAIN LOOP
 *================================================================================================*/

void loop()
{
    while (Serial2.available() > 0)
    {
        char c = (char)Serial2.read();

        /*
         * Ignore CR. LF marks the end of one telemetry packet.
         */
        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            if (sPacketIndex > 0U)
            {
                sPacket[sPacketIndex] = '\0';

                if (ValidatePacket(sPacket))
{
    /*
     * Packet has already passed framing and checksum validation.
     * Forward only the numeric CSV payload to GLANCE.
     */
    char *payload = sPacket + 7;   /* Skip "$$IGNI," */
    char *asterisk = strrchr(payload, '*');

    if (asterisk != NULL)
    {
        *asterisk = '\0';
        Serial.println(payload);
    }
}

                sPacketIndex = 0U;
            }

            continue;
        }

        /*
         * Prevent malformed/oversized radio data from overflowing the buffer.
         */
        if (sPacketIndex < (TELEMETRY_BUFFER_SIZE - 1U))
        {
            sPacket[sPacketIndex++] = c;
        }
        else
        {
            /*
             * Oversized packet: discard it and wait for the next newline.
             */
            sPacketIndex = 0U;
        }
    }
}

/*==================================================================================================
 * STATIC FUNCTION DEFINITIONS
 *================================================================================================*/

static bool ValidatePacket(const char *packet)
{
    if (packet == NULL)
    {
        return false;
    }

    /*
     * Expected framing:
     *
     * $$IGNI,<payload>*XX
     *
     * XX = two hexadecimal characters containing the XOR checksum.
     */

    if (strncmp(packet, "$$IGNI,", 7U) != 0)
    {
        return false;
    }

    const char *asterisk = strrchr(packet, '*');

    if (asterisk == NULL)
    {
        return false;
    }

    if (asterisk <= (packet + 2))
    {
        return false;
    }

    /*
     * Exactly two checksum characters must follow '*'.
     */
    if (asterisk[1] == '\0' ||
        asterisk[2] == '\0' ||
        asterisk[3] != '\0')
    {
        return false;
    }

    int high = HexValue(asterisk[1]);
    int low  = HexValue(asterisk[2]);

    if (high < 0 || low < 0)
    {
        return false;
    }

    uint8_t receivedChecksum =
        (uint8_t)((high << 4) | low);

    uint8_t calculatedChecksum = 0U;

    /*
     * The '$$' framing bytes are excluded.
     * XOR everything from 'I' in IGNI through the byte immediately
     * before '*'.
     */
    const char *p = packet + 2;

    while (p < asterisk)
    {
        calculatedChecksum ^= (uint8_t)(*p);
        p++;
    }

    return calculatedChecksum == receivedChecksum;
}

static int HexValue(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }

    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }

    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }

    return -1;
}