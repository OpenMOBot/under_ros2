/*

MIT License

Copyright (c) [2024] [OpenMOBOt]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#ifndef _APPLICATIONCONFIGURATION_h
#define _APPLICATIONCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#pragma region Options

/**
 * @brief 
 * 
 */
#define ENABLE_DEBUG_PORT

/**
 * @brief 
 * 
 */
#define ENABLE_WIFI

// #define ENABLE_ETH

// #define ENABLE_OTA

// #define ENABLE_NTP

/**
 * @brief Enable status LED.
 *
 */
#define ENABLE_STATUS_LED

/**
 * @brief Enable motors of the robot features.
 *
 */
#define ENABLE_MOTORS

/**
 * @brief Enable sonar servo feature.
 *
 */
#define ENABLE_SONAR_SERVO

/**
 * @brief Enable sonar sensor.
 *
 */
#define ENABLE_SONAR

/**
 * @brief Enable PID regulators.
 *
 */
#define ENABLE_PID

/**
 * @brief Enable ROS communication.
 * 
 */
#define ENABLE_ROS

#pragma endregion

#pragma region Definitions

/**
 * @brief Time interval for update cycle.
 *
 */
#define UPDATE_INTERVAL_MS 100

#if defined(ENABLE_DEBUG_PORT)

/**
 * @brief Serial port baudrate.
 *
 */
#define SERIAL_BAUDRATE 115200

#endif // ENABLE_DEBUG_PORT

#if defined(ENABLE_WIFI)

/**
 * @brief WiFi SSID.
 *
 */
// #define WIFI_SSID "MikroTik"
// #define WIFI_SSID "RemoteDome"
// #define WIFI_SSID "IOsoft@home"
#define WIFI_SSID "Stynex-24"

/**
 * @brief WiFi Password
 *
 */
// #define WIFI_PASS "8801290000"
// #define WIFI_PASS "ot128do256"
// #define WIFI_PASS "pic18f4620"
#define WIFI_PASS "ot1_do10"

#endif // ENABLE_WIFI

#if defined(ENABLE_OTA)
#define OTA_PASS "admin"
#endif // ENABLE_OTA

#if defined(ENABLE_PID)

/**
 * @brief PID update interval.
 *
 */
#define PID_UPDATE_INTERVAL_MS 100

/**
 * @brief P value
 *
 */
#define CONST_P 1.0

/**
 * @brief I value
 *
 */
#define CONST_I 0.00

/**
 * @brief D value
 *
 */
#define CONST_D 0.00

#endif // ENABLE_PID

#if defined(ENABLE_MOTORS)
/**
 * @brief Motors PWM absolute maximum.
 *
 */
#define PWM_ABSOLUTE_MAX 255

/**
 * @brief Motors PWM software limitation.
 *
 */
#define PWM_MAX 200

/**
 * @brief Motors PWM acceleration step.
 *
 */
#define PWM_STEP 5
#endif // ENABLE_MOTORS

#if defined(ENABLE_SONAR_SERVO)
/**
 * @brief Sonar default servo position.
 *
 */
#define DEFAULT_SERVO_POS 90
#endif // defined(ENABLE_SONAR_SERVO)

#if defined(ENABLE_ROS)

/**
 * @brief 
 * 
 */
#define NODE_NAME "add_twoints_client_rclc"

/**
 * @brief 
 * 
 */
#define SERVICE_NAME "/addtwoints"

/**
 * @brief ROS DDS Address.
 *
 */
#define DDS_ADDRESS "192.168.1.25"

/**
 * @brief ROS DDS Port
 *
 */
#define DDS_PORT 7411

/**
 * @brief 
 * 
 */
#define PING_AGENT_ATTEMPTS 1

/**
 * @brief 
 * 
 */
#define SPIN_TIME_MS 100

/**
 * @brief 
 * 
 */
#define PING_AGENT_TIME_MS 100

/**
 * @brief 
 * 
 */
#define WAIT_AGENT_TIME_MS 500

#endif // ENABLE_ROS

#pragma endregion

#endif // _APPLICATIONCONFIGURATION_h
