
#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"
#include "ODriveEnums.h"

class ODriveArduino {
public:
    ODriveArduino(Stream& serial);

    void clearErrors();

    // Commands
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity_tow(float velocity_M0, float velocity_M1);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    void SetCurrent(int motor_number, float current);
    void TrapezoidalMove(int motor_number, float position);

    void SetVelocity_GetFeedback_vbus(float set_VM0, float set_VM1, float *vbus, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);
    void SetVelocity_GetFeedback(float set_VM0, float set_VM1, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);
    void SetVelocity_get_vbus(float set_VM0, float set_VM1, float *vbus);

    // Getters
    float GetVelocity(int motor_number);
    float GetPosition(int motor_number);
    void GetFeedback(float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);
    // General params
    float readFloat();
    int32_t readInt();

    // State helper
    bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f);
private:
    String readString(unsigned long timeout = 5000);
    void Clean_serial();

    Stream& serial_;
};

#endif //ODriveArduino_h
