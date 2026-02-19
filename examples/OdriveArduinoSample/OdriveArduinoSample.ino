/**
 * @file OdriveArduinoSample.ino
 * @brief Ejemplo básico con ODrive 3.6 + ODriveArduino.
 *
 * Comandos por monitor serial:
 * - 'c': calibrar eje 0
 * - 'e': entrar a lazo cerrado (closed loop)
 * - 's': detener motor (velocidad 0)
 */

#include <HardwareSerial.h>
#include "ODriveArduino.h"

HardwareSerial &odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

const int AXIS = 0;
float target_velocity = 1.0f;
bool closed_loop_enabled = false;

void printHelp()
{
    Serial.println();
    Serial.println("=== ODrive 3.6 - Ejemplo Basico ===");
    Serial.println("c: Calibrar eje 0");
    Serial.println("e: Entrar a lazo cerrado");
    Serial.println("s: Stop (velocidad 0)");
    Serial.println("+: Aumentar velocidad +0.5");
    Serial.println("-: Disminuir velocidad -0.5");
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    odrive_serial.begin(115200);

    while (!Serial){;}

    Serial.println("Iniciando...");

    odrive.ClearErrors();
    odrive.WriteProperty(AXIS, "controller.config.vel_limit", 10.0f);
    odrive.WriteProperty(AXIS, "motor.config.current_lim", 20.0f);

    printHelp();
}

void loop()
{
    if (Serial.available())
    {
        char cmd = Serial.read();

        if (cmd == 'c')
        {
            Serial.println("Calibrando eje 0...");
            bool ok = odrive.run_state(AXIS, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true, 60.0f);
            Serial.println(ok ? "Calibracion OK" : "Calibracion timeout/error");
        }

        if (cmd == 'e')
        {
            odrive.run_state(AXIS, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
            closed_loop_enabled = true;
            Serial.println("Lazo cerrado habilitado");
        }

        if (cmd == 's')
        {
            target_velocity = 0.0f;
            odrive.SetVelocity(AXIS, 0.0f);
            Serial.println("Motor detenido");
        }

        if (cmd == '+')
        {
            target_velocity += 0.5f;
            Serial.print("target_velocity = ");
            Serial.println(target_velocity);
        }

        if (cmd == '-')
        {
            target_velocity -= 0.5f;
            Serial.print("target_velocity = ");
            Serial.println(target_velocity);
        }
    }

    if (closed_loop_enabled)
        odrive.SetVelocity(AXIS, target_velocity);

    delay(10);
}
