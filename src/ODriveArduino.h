#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"
#include "ODriveEnums.h"

class ODriveArduino {
public:
    /**
     * @brief Constructs an ODriveArduino instance that will communicate over the specified serial port.
     * @param serial The serial port to communicate over.
     */
    ODriveArduino(Stream& serial);

    /**
     * @brief Clears the error status of the ODrive and restarts the brake resistor if it was disabled due to an error.
     */
    void clearErrors();

    // Commands

    /**
     * @brief Sends a new position setpoint.
     * @param motor_number The motor number (0 or 1).
     * @param position The desired position.
     */
    void SetPosition(int motor_number, float position);

    /**
     * @brief Sends a new position setpoint with a velocity feedforward term.
     * @param motor_number The motor number (0 or 1).
     * @param position The desired position.
     * @param velocity_feedforward The desired velocity feedforward.
     */
    void SetPosition(int motor_number, float position, float velocity_feedforward);

    /**
     * @brief Sends a new position setpoint with velocity and current feedforward terms.
     * @param motor_number The motor number (0 or 1).
     * @param position The desired position.
     * @param velocity_feedforward The desired velocity feedforward.
     * @param current_feedforward The desired current feedforward.
     */
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);

    /**
     * @brief Sends a new velocity setpoint.
     * @param motor_number The motor number (0 or 1).
     * @param velocity The desired velocity.
     */
    void SetVelocity(int motor_number, float velocity);

    /**
     * @brief Sends a new velocity setpoint with a current feedforward term.
     * @param motor_number The motor number (0 or 1).
     * @param velocity The desired velocity.
     * @param current_feedforward The desired current feedforward.
     */
    void SetVelocity(int motor_number, float velocity, float current_feedforward);

    /**
     * @brief Sends new velocity setpoints for two motors.
     * @param velocity_M0 The desired velocity for motor 0.
     * @param velocity_M1 The desired velocity for motor 1.
     */
    void SetVelocity_tow(float velocity_M0, float velocity_M1);

    /**
     * @brief Sends a new current setpoint.
     * @param motor_number The motor number (0 or 1).
     * @param current The desired current.
     */
    void SetCurrent(int motor_number, float current);

    /**
     * @brief Puts the ODrive into trapezoidal trajectory mode and sends a new position setpoint.
     * @param motor_number The motor number (0 or 1).
     * @param position The desired position.
     */
    void TrapezoidalMove(int motor_number, float position);

    /**
     * @brief Sends velocity setpoints for two motors and retrieves feedback and bus voltage.
     * @param set_VM0 The desired velocity for motor 0.
     * @param set_VM1 The desired velocity for motor 1.
     * @param vbus Pointer to store the bus voltage.
     * @param velocity_M0 Pointer to store the velocity feedback for motor 0.
     * @param position_M0 Pointer to store the position feedback for motor 0.
     * @param velocity_M1 Pointer to store the velocity feedback for motor 1.
     * @param position_M1 Pointer to store the position feedback for motor 1.
     */
    void SetVelocity_GetFeedback_vbus(float set_VM0, float set_VM1, float *vbus, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);

    /**
     * @brief Sends velocity setpoints for two motors and retrieves feedback.
     * @param set_VM0 The desired velocity for motor 0.
     * @param set_VM1 The desired velocity for motor 1.
     * @param velocity_M0 Pointer to store the velocity feedback for motor 0.
     * @param position_M0 Pointer to store the position feedback for motor 0.
     * @param velocity_M1 Pointer to store the velocity feedback for motor 1.
     * @param position_M1 Pointer to store the position feedback for motor 1.
     */
    void SetVelocity_GetFeedback(float set_VM0, float set_VM1, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);

    /**
     * @brief Sends velocity setpoints for two motors and retrieves the bus voltage.
     * @param set_VM0 The desired velocity for motor 0.
     * @param set_VM1 The desired velocity for motor 1.
     * @param vbus Pointer to store the bus voltage.
     */
    void SetVelocity_get_vbus(float set_VM0, float set_VM1, float *vbus);

    // Getters

    /**
     * @brief Requests the latest velocity estimate.
     * @param motor_number The motor number (0 or 1).
     * @return The velocity estimate.
     */
    float GetVelocity(int motor_number);

    /**
     * @brief Requests the latest position estimate.
     * @param motor_number The motor number (0 or 1).
     * @return The position estimate.
     */
    float GetPosition(int motor_number);

    /**
     * @brief Requests the latest position and velocity estimates for both motors.
     * @param velocity_M0 Pointer to store the velocity feedback for motor 0.
     * @param position_M0 Pointer to store the position feedback for motor 0.
     * @param velocity_M1 Pointer to store the velocity feedback for motor 1.
     * @param position_M1 Pointer to store the position feedback for motor 1.
     */
    void GetFeedback(float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);

    // Generic parameter access

    /**
     * @brief Gets a parameter from the ODrive as a string.
     * @param path The parameter path.
     * @return The parameter value as a string.
     */
    String getParameterAsString(const String& path);

    /**
     * @brief Sets a parameter on the ODrive.
     * @param path The parameter path.
     * @param value The parameter value as a string.
     */
    void setParameter(const String& path, const String& value);

    /**
     * @brief Sets a parameter on the ODrive.
     * @param path The parameter path.
     * @param value The parameter value as an integer.
     */
    void setParameter(const String& path, long value) { setParameter(path, String(value)); }

    // State helper

    /**
     * @brief Tells the ODrive to change its axis state.
     * @param axis The axis number (0 or 1).
     * @param requested_state The requested state.
     * @param wait_for_idle If true, waits until the axis is idle.
     * @param timeout Timeout in seconds.
     * @return True if the state change was successful.
     */
    bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f);

private:
    /**
     * @brief Reads a line from the serial with a timeout.
     * @param timeout The timeout in milliseconds.
     * @return The read string.
     */
    String readString(unsigned long timeout = 5000);

    /**
     * @brief Cleans the serial buffer.
     */
    void Clean_serial();

    /**
     * @brief Reads a float from the serial.
     * @return The read float.
     */
    float readFloat();

    /**
     * @brief Reads an integer from the serial.
     * @return The read integer.
     */
    int32_t readInt();

    Stream& serial_;
};

#endif //ODriveArduino_h
