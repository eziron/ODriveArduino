/**
 * @file ODriveArduinoTest.ino
 * @author Your Name (@eziron1)
 * @brief Sketch de ejemplo para la librería ODriveArduino con funciones optimizadas.
 *
 * Este sketch ofrece un menú serial interactivo para probar las funciones
 * principales del ODrive, con énfasis en las llamadas optimizadas para
 * comunicación de alta frecuencia.
 *
 * --- COMANDOS DEL MENÚ ---
 * 'h': Imprimir este menú de ayuda
 * '0', '1': Calibrar motor 0 o 1
 * 's': Ejecutar movimiento sinusoidal de posición en ambos motores
 * 'v': Ejecutar prueba de control de velocidad en motor 0
 * 't': Ejecutar prueba de movimiento trapezoidal en motor 0
 * 'c': Ejecutar prueba de control de corriente (torque) en motor 0
 * 'o': Ejecutar prueba OPTIMIZADA de lazo con feedback
 * 'b': Leer voltaje de bus del ODrive
 * 'e': Verificar errores en ambos ejes
 * 'r': Reiniciar el ODrive
 *
 */

// Inclusiones
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include "ODriveArduino.h"

////////////////////////////////
// Configuración de pines serie hacia ODrive
////////////////////////////////

// Teensy 3 y 4 (todas las versiones) - Serial1
// pin 0: RX - conectar a TX de ODrive (GPIO2)
// pin 1: TX - conectar a RX de ODrive (GPIO1)
HardwareSerial &odrive_serial = Serial1;

// Arduino Mega o Due - Serial1
// pin 19: RX - conectar a TX de ODrive (GPIO2)
// pin 18: TX - conectar a RX de ODrive (GPIO1)
// HardwareSerial& odrive_serial = Serial1;

// Arduino sin puertos serie extra (como UNO) debe usar SoftwareSerial.
// Nota: puede ser menos confiable.
// pin 8: RX - conectar a TX de ODrive (GPIO2)
// pin 9: TX - conectar a RX de ODrive (GPIO1)
// SoftwareSerial odrive_serial(8, 9);

// Objeto ODrive
ODriveArduino odrive(odrive_serial);

void printMenu();

void setup()
{
  // UART con ODrive
  odrive_serial.begin(115200);

  // Serial de depuración hacia PC
  Serial.begin(115200);
  while (!Serial)
    ; // Espera apertura del monitor serial

  Serial.println("Sketch de prueba de la libreria ODriveArduino");
  Serial.println("Configurando parametros...");

  // Configuración base con funciones de librería
  for (int axis = 0; axis < 2; ++axis)
  {
    odrive.WriteProperty(axis, "controller.config.vel_limit", 20.0f);
    odrive.WriteProperty(axis, "motor.config.current_lim", 25.0f);
    odrive.WriteProperty(axis, "motor.config.requested_current_range", 30.0f); // Recomendado en motores gimbal
  }

  Serial.println("Listo. Usa el menu:");
  printMenu();
}

void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();

    switch (c)
    {
    case 'h':
      printMenu();
      break;
    case '0':
    case '1':
      calibrateMotor(c - '0');
      break;
    case 's':
      testSinusoidalMove();
      break;
    case 'v':
      testVelocityControl();
      break;
    case 't':
      testTrapezoidalMove();
      break;
    case 'c':
      testCurrentControl();
      break;
    case 'o':
      testOptimizedFeedbackLoop();
      break;
    case 'b':
      readBusVoltage();
      break;
    case 'e':
      checkErrors();
      break;
    case 'r':
      Serial.println("Reiniciando ODrive...");
      odrive.Reboot();
      break;
    }
  }
}

void printMenu()
{
  Serial.println("\n--- Menu de Pruebas ODrive ---");
  Serial.println("'h': Mostrar este menu");
  Serial.println("'0', '1': Calibrar motor 0 o 1");
  Serial.println("'s': Prueba sinusoidal de posicion");
  Serial.println("'v': Prueba de control de velocidad (M0)");
  Serial.println("'t': Prueba de movimiento trapezoidal (M0)");
  Serial.println("'c': Prueba de control de corriente/torque (M0)");
  Serial.println("'o': Prueba OPTIMIZADA de feedback");
  Serial.println("'b': Leer voltaje de bus");
  Serial.println("'e': Verificar errores");
  Serial.println("'r': Reiniciar ODrive\n");
}

// Calibra un eje y lo deja en lazo cerrado.
void calibrateMotor(int motor_num)
{
  Serial.print("Calibrando eje ");
  Serial.print(motor_num);
  Serial.println("...");

  if (!odrive.run_state(motor_num, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true, 25.0f))
  {
    Serial.println("ERROR: Fallo la calibracion");
    return;
  }

  // Opcional: pasar a control en lazo cerrado al finalizar
  odrive.run_state(motor_num, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  Serial.print("Eje ");
  Serial.print(motor_num);
  Serial.println(" calibrado y en control de lazo cerrado.");
}

// Mueve ambos ejes con trayectoria sinusoidal en posicion.
void testSinusoidalMove()
{
  Serial.println("Ejecutando prueba sinusoidal por 5 segundos...");
  long start_time = millis();
  while (millis() - start_time < 5000)
  {
    float phase = (millis() - start_time) / 1000.0f * 2.0f * M_PI;
    float pos_m0 = 2.0f * cos(phase);
    float pos_m1 = 2.0f * sin(phase);
    odrive.SetPosition(0, pos_m0);
    odrive.SetPosition(1, pos_m1);
    delay(10);
  }
  Serial.println("Movimiento finalizado.");
}

// Prueba escalonada de velocidad en el motor 0.
void testVelocityControl()
{
  Serial.println("Probando control de velocidad en Motor 0...");
  Serial.println("Subiendo a 5 vueltas/seg...");
  odrive.SetVelocity(0, 5.0f);
  delay(2000);
  Serial.println("Bajando a -5 vueltas/seg...");
  odrive.SetVelocity(0, -5.0f);
  delay(2000);
  Serial.println("Llevando velocidad a 0.");
  odrive.SetVelocity(0, 0.0f);
  Serial.println("Prueba finalizada.");
}

// Prueba del planificador trapezoidal en el motor 0.
void testTrapezoidalMove()
{
  Serial.println("Probando movimiento trapezoidal de Motor 0 a posicion 5...");
  odrive.TrapezoidalMove(0, 5.0f);
  Serial.println("Comando enviado. Esperando fin de movimiento...");
  odrive.WaitIdle(0, 10.0f); // Espera estado IDLE del motor 0

  Serial.print("Movimiento finalizado. Posicion final: ");
  Serial.println(odrive.GetPosition(0));
}

// Prueba de control de corriente/torque en el motor 0.
void testCurrentControl()
{
  Serial.println("Probando control de corriente/torque en Motor 0.");
  Serial.println("Aplicando 0.5A durante 3 segundos.");
  odrive.SetCurrent(0, 0.5f);
  delay(3000);
  Serial.println("Liberando torque.");
  odrive.SetCurrent(0, 0.0f);
  Serial.println("Prueba finalizada.");
}

// Lazo optimizado: setpoint de ambos ejes + feedback + vbus.
void testOptimizedFeedbackLoop()
{
  Serial.println("\n--- Prueba de Lazo OPTIMIZADO ---");
  Serial.println("Envia comandos a ambos motores y lee todo el feedback en una sola transaccion.");
  Serial.println("Ejecutando por 5 segundos...\n");

  long start_time = millis();
  int iterations = 0;
  float vbus, vel0, pos0, vel1, pos1;

  while (millis() - start_time < 5000)
  {
    // Genera setpoints de velocidad simples
    float set_v0 = 2.0f * sin((millis() - start_time) / 1000.0f * M_PI);
    float set_v1 = 2.0f * cos((millis() - start_time) / 1000.0f * M_PI);

    // Llamada optimizada principal
    odrive.SetVelocityBoth_GetFeedback_Vbus(set_v0, set_v1, &vbus, &vel0, &pos0, &vel1, &pos1);

    iterations++;
    // Muestra feedback cada cierto tiempo para no saturar el puerto serie
    if (iterations % 50 == 0)
    {
      Serial.print("Vbus: ");
      Serial.print(vbus);
      Serial.print("V, M0_pos: ");
      Serial.print(pos0);
      Serial.print(", M0_vel: ");
      Serial.print(vel0);
      Serial.print(", M1_pos: ");
      Serial.print(pos1);
      Serial.print(", M1_vel: ");
      Serial.println(vel1);
    }
    delay(10); // Simula trabajo adicional del loop
  }

  // Detiene motores al finalizar
  odrive.SetVelocityBoth(0.0f, 0.0f);

  long duration = millis() - start_time;
  Serial.println();
  Serial.print("Prueba finalizada. Se ejecutaron ");
  Serial.print(iterations);
  Serial.print(" ciclos en ");
  Serial.print(duration);
  Serial.println(" ms.");
  Serial.print(" Frecuencia promedio del loop: ");
  Serial.print((float)iterations / (duration / 1000.0f));
  Serial.println(" Hz");
}

// Lee el voltaje de bus de ODrive.
void readBusVoltage()
{
  float vbus = 0.0f;
  // Usa lectura de propiedades de la libreria
  odrive.ReadProperty(0, "vbus_voltage", &vbus);
  Serial.print("Voltaje Vbus: ");
  Serial.print(vbus);
  Serial.println(" V");
}

// Lee el registro de error de cada eje.
void checkErrors()
{
  Serial.println("Verificando errores...");
  for (int axis = 0; axis < 2; ++axis)
  {
    int error = 0;
    odrive.ReadProperty(axis, "error", &error);
    if (error != 0)
    {
      Serial.print("ERROR en eje ");
      Serial.print(axis);
      Serial.print(". Codigo: 0x");
      Serial.println(String(error, HEX));
    }
    else
    {
      Serial.print("Eje ");
      Serial.print(axis);
      Serial.println(": Sin errores.");
    }
  }
  Serial.println("Consulta la documentacion de ODrive para interpretar codigos.");
}