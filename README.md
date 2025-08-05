# ODriveArduino Extended: Optimized for High-Frequency Control

Esta es una librería de Arduino para controlar los drivers de motor ODrive (v3.x) a través de su interfaz UART de texto.

## Una Extensión de la Librería Oficial

Esta librería comenzó como un fork de la [**librería oficial ODriveArduino**](https://github.com/odriverobotics/ODriveArduino) y extiende su funcionalidad. Mantiene la compatibilidad con la API original pero agrega nuevas funciones optimizadas para aplicaciones de alto rendimiento, como la robótica.

## Característica Clave: Comunicación Optimizada

El principal objetivo de esta extensión es reducir la latencia de la comunicación UART. En aplicaciones de control en tiempo real, enviar múltiples comandos por separado puede crear un cuello de botella.

**Método Tradicional (Múltiples Transacciones):**
```cpp
// 3 viajes de ida y vuelta por el puerto serie
odrive.SetVelocity(0, 5.0);
odrive.SetVelocity(1, -5.0);
float vbus = odrive.readFloat(); // Requiere enviar "r vbus_voltage"
```

**Método Optimizado (Una Sola Transacción):**
Esta librería introduce funciones que agrupan varios comandos en un solo envío, reduciendo drásticamente la latencia.
```cpp
// 1 solo viaje de ida y vuelta para hacer todo
float vbus, vel0, pos0, vel1, pos1;
odrive.SetVelocityBoth_GetFeedback_Vbus(5.0, -5.0, &vbus, &vel0, &pos0, &vel1, &pos1);
```

## Instalación

1.  Descarga este repositorio como un archivo `.zip`.
2.  Abre tu IDE de Arduino.
3.  Ve a `Programa > Incluir Librería > Añadir biblioteca .ZIP`.
4.  Selecciona el archivo `.zip` que acabas de descargar.
5.  ¡Listo! Ahora puedes usar `#include "ODriveArduino.h"` en tus sketches.

## Uso Básico

Primero, incluye la librería y crea un objeto `ODriveArduino`, pasándole el objeto `Stream` que usarás para la comunicación (normalmente un `HardwareSerial`).

```cpp
#include <HardwareSerial.h>
#include "ODriveArduino.h"

// Selecciona el puerto serie conectado al ODrive
HardwareSerial& odrive_serial = Serial1;

// Crea el objeto ODrive
ODriveArduino odrive(odrive_serial);

void setup() {
  // Inicia la comunicación serial con el ODrive a 115200 baud
  odrive_serial.begin(115200);

  // Inicia el serial para el monitor de depuración
  Serial.begin(115200);

  // Calibra el motor 0 y ponlo en modo de control de bucle cerrado
  odrive.run_state(0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true);
  odrive.run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
}
```

## Documentación de la API (Funciones Principales)

Las funciones se agrupan por propósito. Las funciones sobrecargadas se explican una sola vez.

---

### Configuración y Estado

*   `bool run_state(int axis, int requested_state, bool wait_for_idle, ...)`
    Cambia el estado del eje (p. ej., a `AXIS_STATE_CLOSED_LOOP_CONTROL`). Opcionalmente, puede esperar a que el eje vuelva al estado `IDLE`.

*   `void WaitIdle(int motor_number, ...)`
    Pausa la ejecución hasta que uno o ambos motores (`motor_number = -1`) lleguen al estado `IDLE`.

*   `SaveConfig()`, `EraseConfig()`, `Reboot()`, `ClearErrors()`
    Funciones de una línea para guardar la configuración, borrarla, reiniciar el ODrive o limpiar los errores.

---

### Comandos de Movimiento (Un solo motor)

*   `SetPosition(int motor_number, float position, ...)`
    Comanda al motor a una posición. Opcionalmente, puedes añadir valores de *feedforward* para velocidad y corriente.

*   `SetVelocity(int motor_number, float velocity, ...)`
    Comanda al motor a una velocidad. Opcionalmente, puedes añadir un valor de *feedforward* de corriente.

*   `SetCurrent(int motor_number, float current)`
    Comanda al motor a un torque/corriente específico.

*   `TrapezoidalMove(int motor_number, float position)`
    Comanda al motor a una posición utilizando el planificador de trayectoria trapezoidal.

---

### Lectura de Feedback (Getters)

*   `GetPosition(int motor_number)` / `GetVelocity(int motor_number)`
    Devuelven la posición o velocidad estimada del motor especificado.

*   `GetFeedback(int motor_number, float* velocity, float* position)`
    Obtiene la velocidad y posición de un solo motor en una sola llamada.

---

### **Funciones Optimizadas (Ambos motores)**

Estas funciones son la principal ventaja de esta librería.

*   `SetVelocityBoth(float velocity_M0, float velocity_M1)`
    Establece la velocidad de ambos motores en una sola transacción.

*   `GetFeedbackBoth(float* vel_M0, float* pos_M0, ...)`
    Obtiene la posición y velocidad de ambos motores en una sola transacción.

*   `SetVelocityBoth_GetFeedback(...)` / `SetVelocityBoth_GetFeedback_Vbus(...)`
    **Las funciones más potentes.** Establecen la velocidad de ambos motores y, en la misma transacción, leen el feedback de posición/velocidad y opcionalmente el voltaje del bus. Ideal para bucles de control de alta frecuencia.

### Ejemplo Avanzado: Bucle de Control Optimizado

Este ejemplo muestra cómo usar las funciones optimizadas para un control eficiente.

```cpp
void loop() {
  // Variables para almacenar el feedback
  float vbus, vel0, pos0, vel1, pos1;

  // Genera comandos de velocidad para ambos motores
  float set_v0 = 2.0f * sin(millis() / 1000.0f * M_PI);
  float set_v1 = 2.0f * cos(millis() / 1000.0f * M_PI);

  // ¡Una sola función para enviar comandos y recibir todo el feedback!
  odrive.SetVelocityBoth_GetFeedback_Vbus(set_v0, set_v1, &vbus, &vel0, &pos0, &vel1, &pos1);
  
  // Imprime los datos recibidos
  Serial.print("Vbus: "); Serial.print(vbus);
  Serial.print(" | M0_pos: "); Serial.print(pos0);
  Serial.print(" | M1_pos: "); Serial.println(pos1);
  
  delay(10); // Simula el resto del trabajo del bucle
}
```

### Enumeraciones (`ODriveEnums.h`)

Para evitar el uso de "números mágicos", esta librería incluye un archivo `ODriveEnums.h` con todas las constantes importantes del ODrive, como:
- `AXIS_STATE_...`
- `CONTROL_MODE_...`
- `INPUT_MODE_...`

El uso de estas constantes hace que tu código sea mucho más legible y robusto.