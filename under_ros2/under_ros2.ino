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

#pragma region Definitions

#pragma endregion // Definitions

#pragma region Headers

#include "ApplicationConfiguration.h"

#include "OpenMOBot.h"
#include "DebugPort.h"

#if defined(ENABLE_SONAR_SERVO)
#include <ESP32Servo.h>
#endif // ENABLE_SONAR_SERVO

#if defined(ENABLE_PID)
#include <PID_v1.h>
#endif // ENABLE_PID

#if defined(ENABLE_SONAR)
// nasko - 
#include "HCSR04.h"
#endif // ENABLE_SONAR

#if defined(ENABLE_ROS)
#include <micro_ros_arduino.h>
#include <example_interfaces/srv/add_two_ints.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64.h>
#endif // ENABLE_ROS

#pragma endregion // Headers

#pragma region Macros

#if defined(ENABLE_ROS)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

#endif // ENABLE_ROS

#pragma endregion // Macros

#pragma region Enums

enum RosAppState_t {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} RosAppState_g;

#pragma endregion // Enums

#pragma region Variables

#if defined(ENABLE_STATUS_LED)
/**
 * @brief Blink led last state flag.
 */
int StateStatusLED_g = LOW;
#endif // ENABLE_STATUS_LED

#if defined(ENABLE_MOTORS) || defined(ENABLE_PID)
/**
 * @brief Left motor PWM.
 *
 */
double PWMLeft_g = 0;

/**
 * @brief Right motor PWM.
 *
 */
double PWMRight_g = 0;

/**
 * @brief Feedback from left encoder.
 *
 */
double FBLeft_g = 0;

/**
 * @brief Feedback from right encoder.
 *
 */
double FBRight_g = 0;
#endif // defined(ENABLE_MOTORS) || defined(ENABLE_PID)

#if defined(ENABLE_SONAR_SERVO)
/**
 * @brief Create servo object to control a servo.
 */
Servo UsServo_g;

/**
 * @brief Servo J position.
 *
 */
uint8_t SonarServoPos_g = DEFAULT_SERVO_POS;
#endif // defined(ENABLE_SONAR_SERVO)

#if defined(ENABLE_SONAR)
/**
 * @brief Ultrasonic sensor.
 */
HCSR04 HCSR04_g;

/**
 * @brief Distance from sonar.
 *
 */
float Distance_g = 0.0;
#endif // ENABLE_SONAR

#if defined(ENABLE_PID)
/**
 * @brief Define Variables we'll be connecting to
 */
double OutLeft_g, OutRight_g;

/**
 * @brief PID regulator for left wheel.
 */
PID *PIDLeft_g;

/**
 * @brief PID regulator for right wheel.
 */
PID *PIDRight_g;

#endif // ENABLE_PID

#if defined(ENABLE_ROS)
/**
 * @brief 
 * 
 */
rcl_node_t Node_g;

/**
 * @brief 
 * 
 */
rclc_support_t Support_g;

/**
 * @brief 
 * 
 */
rcl_allocator_t Allocator_g;

/**
 * @brief 
 * 
 */
rclc_executor_t Executor_g;

/**
 * @brief 
 * 
 */
rcl_service_t Service_g;

/**
 * @brief 
 * 
 */
example_interfaces__srv__AddTwoInts_Response res;

/**
 * @brief 
 * 
 */
example_interfaces__srv__AddTwoInts_Request req;
#endif // ENABLE_ROS

/**
 * @brief Update timer instance.
 */
FxTimer *UpdateTimer_g;

#pragma endregion // Variables

#pragma region Prototypes

#if defined(ENABLE_MOTORS)
/** @brief Interrupt Service Routine for handling left encoder.
 *  @return Void.
 */
void ISR_Left_Encoder();

/** @brief Interrupt Service Routine for handling right encoder.
 *  @return Void.
 */
void ISR_Right_Encoder();
#endif // ENABLE_MOTORS

#if defined(ENABLE_ROS)
/**
 * @brief Create a entities object
 * 
 * @return true 
 * @return false 
 */
bool create_entities();

/**
 * @brief 
 * 
 */
void destroy_entities();

/**
 * @brief 
 * 
 * @param req 
 * @param res 
 */
void cb_service(const void * req, void * res);

/**
 * @brief 
 * 
 */
void update_ros_app();
#endif // ENABLE_ROS

#pragma endregion

void setup()
{
    // Init serial
    Serial.begin(DEFAULT_BAUD);

#if defined(ENABLE_STATUS_LED)
    pinMode(PIN_USER_LED, OUTPUT);
#endif // ENABLE_STATUS_LED

#if defined(ENABLE_MOTORS)
    // Attach the Interrupts to their ISR's
    pinMode(PIN_LEFT_ENCODER, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENCODER, INPUT_PULLUP);

    // Increase counter 1 when speed sensor pin goes High.
    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENCODER), ISR_Left_Encoder, RISING);
    // Increase counter 2 when speed sensor pin goes High.
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENCODER), ISR_Right_Encoder, RISING);

    // Setup the motor driver.
    MotorModel_t MotorModelL = {
        PIN_L_F,
        PIN_L_B,
        PIN_L_PWM,
        PIN_R_F,
        PIN_R_B,
        PIN_R_PWM,
        WHEEL_DIAMETER,
        DISTANCE_BETWEEN_WHEELS,
        ENCODER_TRACKS};

    // Initialize the motor controller.
    MotorController.init(&MotorModelL);
#endif // ENABLE_MOTORS

#if defined(ENABLE_SONAR_SERVO)
    // Attaches servo to the servo object.
    UsServo_g.attach(PIN_US_SERVO);
#endif // ENABLE_SONAR_SERVO

#if defined(ENABLE_SONAR)
    // Initialize the ultrasonic.
    HCSR04_g.init(PIN_US_TRIG, PIN_US_ECHO);
#endif // ENABLE_SONAR

#if defined(ENABLE_PID)
    // Set the PID regulators.
    PIDLeft_g = new PID(&FBLeft_g, &OutLeft_g, &PWMLeft_g, CONST_P, CONST_I, CONST_D, DIRECT);
    PIDLeft_g->SetMode(AUTOMATIC);
    PIDLeft_g->SetSampleTime(PID_UPDATE_INTERVAL_MS);
    PIDLeft_g->SetOutputLimits(-PWM_MAX, PWM_MAX);

    PIDRight_g = new PID(&FBRight_g, &OutRight_g, &PWMRight_g, CONST_P, CONST_I, CONST_D, DIRECT);
    PIDRight_g->SetMode(AUTOMATIC);
    PIDRight_g->SetSampleTime(PID_UPDATE_INTERVAL_MS);
    PIDRight_g->SetOutputLimits(-PWM_MAX, PWM_MAX);
#endif // ENABLE_PID

#if defined(ENABLE_WIFI) && defined(ENABLE_ROS)
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, DDS_ADDRESS, DDS_PORT);
    RosAppState_g = WAITING_AGENT;
#endif // ENABLE_WIFI && ENABLE_ROS

    UpdateTimer_g = new FxTimer();
    UpdateTimer_g->setExpirationTime(UPDATE_INTERVAL_MS);
    UpdateTimer_g->updateLastTime();
}

void loop()
{
    // Update the timer.
    UpdateTimer_g->update();
    if (UpdateTimer_g->expired())
    {
        UpdateTimer_g->updateLastTime();
        UpdateTimer_g->clear();

#if defined(ENABLE_SONAR)
        long MicrosecL = HCSR04_g.timing();
        Distance_g = HCSR04_g.convert(MicrosecL, HCSR04::CM);
#endif // ENABLE_SONAR

#if defined(ENABLE_MOTORS)
        // Update motor speeds.
        MotorController.update();
#endif // ENABLE_MOTORS

#if defined(ENABLE_PID)
        PIDLeft_g->Compute();
        PIDRight_g->Compute();
#if defined(ENABLE_MOTORS)
        // Set the input feedback.
        FBLeft_g = MotorController.GetLeftMotorRPM();   // / 5.0;
        FBRight_g = MotorController.GetRightMotorRPM(); // / 5.0;

        // Set the output from the regulator.
        MotorController.SetPWM(OutLeft_g, OutRight_g);
#endif // ENABLE_MOTORS
#else // ENABLE_PID
#if defined(ENABLE_MOTORS)
        // Set the PWMs to the motors.
        MotorController.SetPWM(PWMLeft_g, PWMRight_g);
#endif // ENABLE_MOTORS
#endif // !ENABLE_PID

#if defined(ENABLE_SONAR_SERVO)
        // Set servo J position.
        UsServo_g.write(int(map(SonarServoPos_g, 0, 180, 180, 0)));
#endif // ENABLE_SONAR_SERVO

#if defined(ENABLE_STATUS_LED) && defined(ENABLE_ROS)
        if (RosAppState_g == AGENT_CONNECTED)
        {
            // set the LED with the StateStatusLED_g of the variable:
            StateStatusLED_g = !StateStatusLED_g;
            digitalWrite(PIN_USER_LED, StateStatusLED_g);
        }
        else
        {
            digitalWrite(PIN_USER_LED, LOW);
        }
#endif // ENABLE_STATUS_LED && ENABLE_ROS
    }
#if defined(ENABLE_ROS)
  update_ros_app();
#endif // ENABLE_ROS
}

#pragma region Functions

#if defined(ENABLE_MOTORS)
/** @brief Interrupt Service Routine for handling left encoder.
 *  @return Void.
 */
void ISR_Left_Encoder()
{
    MotorController.UpdateLeftEncoder();
}

/** @brief Interrupt Service Routine for handling right encoder.
 *  @return Void.
 */
void ISR_Right_Encoder()
{
    MotorController.UpdateRightEncoder();
}
#endif // ENABLE_MOTORS

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

#if defined(ENABLE_ROS)
/**
 * @brief Create a entities object
 * 
 * @return true 
 * @return false 
 */
bool create_entities()
{
    Allocator_g = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&Support_g, 0, NULL, &Allocator_g));

    // create Node_g
    RCCHECK(rclc_node_init_default(&Node_g, NODE_NAME, "", &Support_g));

    // create Service_g
    RCCHECK(rclc_service_init_default(&Service_g, &Node_g, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), SERVICE_NAME));

    // create Executor_g
    RCCHECK(rclc_executor_init(&Executor_g, &Support_g.context, 1, &Allocator_g));
    RCCHECK(rclc_executor_add_service(&Executor_g, &Service_g, &req, &res, cb_service));

    return true;
}

/**
 * @brief 
 * 
 */
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&Support_g.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&Executor_g);
  rcl_node_fini(&Node_g);
  rclc_support_fini(&Support_g);
}

/**
 * @brief 
 * 
 * @param req 
 * @param res 
 */
void cb_service(const void * req, void * res)
{
    example_interfaces__srv__AddTwoInts_Request * req_in = (example_interfaces__srv__AddTwoInts_Request *) req;
    example_interfaces__srv__AddTwoInts_Response * res_in = (example_interfaces__srv__AddTwoInts_Response *) res;

    //printf("Service request value: %d + %d.\n", (int) req_in->a, (int) req_in->b);

    res_in->sum = req_in->a + req_in->b;
}

/**
 * @brief 
 * 
 */
void update_ros_app()
{
    switch (RosAppState_g)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(WAIT_AGENT_TIME_MS, RosAppState_g = (RMW_RET_OK == rmw_uros_ping_agent(PING_AGENT_TIME_MS, PING_AGENT_ATTEMPTS)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            RosAppState_g = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (RosAppState_g == WAITING_AGENT)
            {
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, RosAppState_g = (RMW_RET_OK == rmw_uros_ping_agent(PING_AGENT_TIME_MS, PING_AGENT_ATTEMPTS)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (RosAppState_g == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&Executor_g, RCL_MS_TO_NS(SPIN_TIME_MS));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            RosAppState_g = WAITING_AGENT;
            break;
        default:
            break;
    }
}
#endif // ENABLE_ROS

#pragma endregion // Functions
