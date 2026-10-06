#ifndef APP_MODE_H
#define APP_MODE_H

/*
 * Integrated hardware-validation build.
 *
 * Sensors and telemetry remain active while binary command packets drive the
 * real mixer.  The final hardware write is limited to neutral +/- 60 us so a
 * malformed but otherwise valid command cannot request full thrust during
 * the dry bench test.
 */
#define APP_SENSOR_TASK_ENABLED        1U
#define APP_MOTOR_NEUTRAL_TEST         0U
#define APP_SINGLE_MOTOR_TEST_ENABLED  0U
#define APP_MOTOR_OUTPUTS_ENABLED      1U
#define APP_MOTOR_OUTPUT_LIMIT_US      60U

#if (APP_MOTOR_NEUTRAL_TEST != 0U) && (APP_MOTOR_OUTPUTS_ENABLED != 0U)
#error "Neutral-test mode cannot enable commanded motor outputs"
#endif

#if (APP_SINGLE_MOTOR_TEST_ENABLED != 0U) && (APP_MOTOR_NEUTRAL_TEST == 0U)
#error "Single-motor test requires neutral-test mode"
#endif

#if (APP_MOTOR_OUTPUT_LIMIT_US > 300U)
#error "Motor validation limit cannot exceed the configured PWM range"
#endif

#endif /* APP_MODE_H */
