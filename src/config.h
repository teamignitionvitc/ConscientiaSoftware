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

                    © Copyright of Ignition Avionics Department
*/

/**************************************************************************************************
 * File:        config.h
 * Author:      Vidisha Jain
 * Created On:  26/10/26
 *
 * \brief       Central hardware and system configuration for CONSCIENTIA
 * \details     Defines all pin assignments, bus parameters, sensor addresses,
 *              timing constants, and system-level settings for the Waveshare
 *              RP2040-Zero flight computer.  This file is the single source of
 *              truth for hardware configuration.
 ***************************************************************************************************
 * HISTORY:
 *
 *       +----- (NEW | MODify | ADD | DELete)
 *       |
 * No#   |       when       who                  what
 ******+*********+**********+********************+**************************************************
 * 000  NEW      26/10/26   Vidisha Jain         Initial creation
 **************************************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

/**************************************************************************************************
 * Includes
 **************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************************************
 * Board / System Constants
 **************************************************************************************************/
#define CONSCIENTIA_BOARD               "Waveshare RP2040-Zero"
#define CONSCIENTIA_MCU                 "RP2040"
#define CONSCIENTIA_SYS_CLK_KHZ        (133000U)

/* Onboard WS2812 RGB LED */
#define PIN_ONBOARD_LED                 (16U)

/**************************************************************************************************
 * I2C0 Configuration — Shared Sensor Bus
 *   Devices: MPU6050, BMP280, BH1750 #1, BH1750 #2
 **************************************************************************************************/
#define I2C0_INSTANCE                   i2c0
#define PIN_I2C0_SDA                    (28U)
#define PIN_I2C0_SCL                    (29U)
#define I2C0_BAUDRATE_HZ               (400000U)

/**************************************************************************************************
 * I2C1 Configuration — Dedicated BH1750 #3 Bus
 *   Resolves address conflict: BH1750 supports only 0x23 / 0x5C.
 *   Two sensors on I2C0, third on I2C1 at 0x23.
 **************************************************************************************************/
#define I2C1_INSTANCE                   i2c1
#define PIN_I2C1_SDA                    (14U)
#define PIN_I2C1_SCL                    (15U)
#define I2C1_BAUDRATE_HZ               (400000U)

/**************************************************************************************************
 * I2C Sensor Addresses (7-bit)
 **************************************************************************************************/

/* MPU6050 — 6-axis IMU  (AD0 -> GND) */
#define MPU6050_I2C_ADDR                (0x68U)

/* BMP280 — Barometric pressure + temperature  (SDO -> GND) */
#define BMP280_I2C_ADDR                 (0x76U)

/* BH1750 — Ambient light sensors
 *   #1 on I2C0, ADDR -> GND  = 0x23
 *   #2 on I2C0, ADDR -> VCC  = 0x5C
 *   #3 on I2C1, ADDR -> GND  = 0x23  (separate bus avoids conflict)
 */
#define BH1750_ADDR_LOW                 (0x23U)
#define BH1750_ADDR_HIGH                (0x5CU)

/* L89HA GNSS — I2C slave address (documented; UART is the primary interface) */
#define L89HA_I2C_ADDR                  (0x3AU)

/**************************************************************************************************
 * SPI1 Configuration — Micro SD Card
 **************************************************************************************************/
#define SPI1_INSTANCE                   spi1
#define PIN_SPI1_SCK                    (10U)
#define PIN_SPI1_MOSI                   (11U)
#define PIN_SPI1_MISO                   (12U)
#define PIN_SD_CS                       (13U)
#define SPI_SD_INIT_BAUDRATE_HZ        (1000000U)
#define SPI_SD_DATA_BAUDRATE_HZ        (25000000U)

/**************************************************************************************************
 * UART0 Configuration — LoRa E32 433T3D (Telemetry Downlink)
 **************************************************************************************************/
#define UART0_INSTANCE                  uart0
#define PIN_UART0_TX                    (0U)
#define PIN_UART0_RX                    (1U)
#define UART0_BAUDRATE                  (9600U)

/* LoRa E32 control pins */
#define PIN_LORA_M0                     (2U)
#define PIN_LORA_M1                     (3U)
#define PIN_LORA_AUX                    (6U)

/**************************************************************************************************
 * UART1 Configuration — L89HA GNSS Module
 **************************************************************************************************/
#define UART1_INSTANCE                  uart1
#define PIN_UART1_TX                    (4U)
#define PIN_UART1_RX                    (5U)
#define UART1_BAUDRATE                  (9600U)

/**************************************************************************************************
 * PIO UART Configuration — PMS5003 Particulate Sensor
 *   Uses PIO0 state machines for software UART (uart_tx.pio / uart_rx.pio)
 **************************************************************************************************/
#define PIO_PMS_INSTANCE                pio0
#define PIO_PMS_SM_TX                   (0U)
#define PIO_PMS_SM_RX                   (1U)
#define PIN_PMS_TX                      (8U)
#define PIN_PMS_RX                      (9U)
#define PMS5003_BAUDRATE                (9600U)
#define PMS5003_FRAME_LEN               (32U)

/**************************************************************************************************
 * ADC Configuration — MQ135 Air Quality Sensor
 *   Analog output via external voltage divider (5 V -> 3.3 V).
 *   V_sensor = V_adc * (R1 + R2) / R2
 **************************************************************************************************/
#define PIN_MQ135_ADC                   (26U)
#define MQ135_ADC_CHANNEL               (0U)
#define MQ135_DIVIDER_R1_OHMS          (33000U)
#define MQ135_DIVIDER_R2_OHMS          (56000U)
#define ADC_VREF_MV                    (3300U)
#define ADC_RESOLUTION_BITS            (12U)
#define ADC_MAX_COUNT                  (4095U)

/**
 * \brief   MQ135 load resistor value (hardware-defined, in circuit).
 * \details Standard 10 kΩ load resistor (RL) used in the MQ135 module.
 *          Confirm against the actual module schematic before calibration.
 */
#define MQ135_RL_OHMS                  (10000U)

/**
 * \brief   MQ135 provisional clean-air baseline resistance R0 (Ohms).
 * \details This is a PROVISIONAL placeholder value derived from typical
 *          MQ135 datasheet characteristics in clean air (Rs/R0 ≈ 3.6,
 *          RL = 10 kΩ, Vcc = 5 V).  It is NOT laboratory-calibrated.
 *
 * \todo    Replace with the R0 measured during a physical clean-air
 *          calibration session (sensor warm-up ≥ 24 h, known-clean
 *          environment, stable temperature and humidity).
 */
#define MQ135_R0_OHMS_PROVISIONAL      (10000U)

/**************************************************************************************************
 * PWM Configuration — MG90 Servo (Recovery Deployment)
 **************************************************************************************************/
#define PIN_SERVO                       (7U)
#define SERVO_PWM_FREQ_HZ              (50U)
#define SERVO_PULSE_MIN_US             (500U)
#define SERVO_PULSE_MAX_US             (2400U)
#define SERVO_PULSE_NEUTRAL_US         (1450U)

/**************************************************************************************************
 * Timing / Sample-Rate Constants (milliseconds)
 **************************************************************************************************/
#define SAMPLE_INTERVAL_IMU_MS         (20U)
#define SAMPLE_INTERVAL_BARO_MS        (100U)
#define SAMPLE_INTERVAL_LIGHT_MS       (250U)
#define SAMPLE_INTERVAL_AIR_MS         (1000U)
#define SAMPLE_INTERVAL_PM_MS          (1000U)
#define SAMPLE_INTERVAL_GNSS_MS        (1000U)

/**************************************************************************************************
 * Telemetry / Logging Timing Constants (milliseconds)
 **************************************************************************************************/
#define TELEMETRY_TX_INTERVAL_MS       (1000U)
#define DATALOG_FLUSH_INTERVAL_MS      (500U)
#define HEALTH_CHECK_INTERVAL_MS       (5000U)

/**************************************************************************************************
 * Flight / Recovery Constants
 **************************************************************************************************/

/**
 * \brief   Maximum allowable response time from release / descent onset
 *          to completed recovery deployment.
 * \details This is a response-time BUDGET, not a delay.
 *
 *          Desired sequence:
 *            1. CanSat released — descent begins
 *            2. Sensors detect the event as quickly as possible
 *            3. Flight software confirms release / descent
 *            4. Recovery mechanism is triggered immediately
 *            5. Servo deploys recovery system
 *
 *          Steps 1-5 must complete within this budget.
 *
 * \todo    Final timing tolerance and measurement reference (e.g. from
 *          ejection signal, barometric peak, or accelerometer transient)
 *          need team confirmation.
 */
#define RECOVERY_MAX_RESPONSE_TIME_MS  (3000U)

#endif /* CONFIG_H_ */
