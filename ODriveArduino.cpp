
#include "Arduino.h"
#include "ODriveArduino.h"

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 1); return obj; }

ODriveArduino::ODriveArduino(Stream& serial)
    : serial_(serial) {}

void ODriveArduino::clearErrors() {
    serial_ << "sc\n";
}

void ODriveArduino::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveArduino::SetVelocity_tow(float velocity_M0, float velocity_M1) {
    serial_ << "v 0 " << velocity_M0 << "\nv 1" << velocity_M1 << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveArduino::SetCurrent(int motor_number, float current) {
    serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::TrapezoidalMove(int motor_number, float position) {
    serial_ << "t " << motor_number << " " << position << "\n";
}

void ODriveArduino::SetVelocity_GetFeedback_vbus(float set_VM0, float set_VM1, float *vbus, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1){
    ODriveArduino::Clean_serial();
    serial_ << "f 0\nf 1\nr vbus_voltage\nv 0 " << set_VM0 << "\nv 1 " << set_VM1 << "\n";
    *position_M0 =  ODriveArduino::readFloat();
    *velocity_M0 =  ODriveArduino::readFloat();
    *position_M1 =  ODriveArduino::readFloat();
    *velocity_M1 =  ODriveArduino::readFloat();
    *vbus =  ODriveArduino::readFloat();
}

void ODriveArduino::SetVelocity_GetFeedback(float set_VM0, float set_VM1, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1){
    ODriveArduino::Clean_serial();
    serial_ << "f 0\nf 1\nv 0 " << set_VM0 << "\nv 1 " << set_VM1 << "\n";
    *position_M0 =  ODriveArduino::readFloat();
    *velocity_M0 =  ODriveArduino::readFloat();
    *position_M1 =  ODriveArduino::readFloat();
    *velocity_M1 =  ODriveArduino::readFloat();
}

void ODriveArduino::SetVelocity_get_vbus(float set_VM0, float set_VM1, float *vbus){
    ODriveArduino::Clean_serial();
    serial_ << "r vbus_voltage\nv 0 " << set_VM0 << "\nv 1 " << set_VM1 << "\n";
    *vbus =  ODriveArduino::readFloat();
}

float ODriveArduino::readFloat() {
    return readString().toFloat();
}

float ODriveArduino::GetVelocity(int motor_number) {
	serial_<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return ODriveArduino::readFloat();
}

float ODriveArduino::GetPosition(int motor_number) {
    serial_ << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return ODriveArduino::readFloat();
}

void ODriveArduino::GetFeedback(float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1){
    ODriveArduino::Clean_serial();
    serial_ << "f 0\nf 1\n";
    *position_M0 =  ODriveArduino::readFloat();
    *velocity_M0 =  ODriveArduino::readFloat();
    *position_M1 =  ODriveArduino::readFloat();
    *velocity_M1 =  ODriveArduino::readFloat();
}

int32_t ODriveArduino::readInt() {
    return readString().toInt();
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait_for_idle, float timeout) {
    int timeout_ctr = (int)(timeout * 10.0f);
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(100);
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

String ODriveArduino::readString(unsigned long timeout) {
    String str = "";
    unsigned long timeout_start = micros();
    for (;;) {
        while (!serial_.available()) {
            if (micros() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n' || c == ' ')
            break;
        str += c;
    }
    return str;
}

void ODriveArduino::Clean_serial(){
    while (serial_.available()) {
        char c = serial_.read();
    }
}
