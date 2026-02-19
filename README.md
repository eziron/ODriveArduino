# ODriveArduino Extended

Librería de Arduino para controlar ODrive por UART (protocolo ASCII), con foco en control de alta frecuencia y baja latencia.

Este proyecto parte de la librería oficial de ODrive y mantiene su estilo de uso, agregando funciones optimizadas para enviar comandos y leer feedback en menos transacciones.

Repositorio original de referencia:

- https://github.com/odriverobotics/ODriveArduino

Documentación oficial del protocolo ASCII (ODrive 0.5.6 / ODrive 3.6):

- https://docs.odriverobotics.com/v/0.5.6/ascii-protocol.html

## Características principales

- API simple para comandos de posición, velocidad y corriente.
- Funciones de lectura agrupada de feedback (posición/velocidad y `vbus`).
- Funciones combinadas para **setpoint + feedback** en una sola secuencia de UART.
- Soporte de constantes de estado/modo mediante `ODriveEnums.h`.

## Compatibilidad

- ODrive con interfaz UART ASCII habilitada.
- Pensada para ODrive v3.x (puede funcionar con otras versiones si mantienen comandos ASCII compatibles).
- Cualquier `Stream` de Arduino (`HardwareSerial`, `SoftwareSerial`, etc.).

## Instalación

1. Descarga el repositorio como `.zip`.
2. En Arduino IDE: `Programa > Incluir Librería > Añadir biblioteca .ZIP`.
3. Selecciona el archivo `.zip` descargado.
4. Incluye en tu sketch:

```cpp
#include "ODriveArduino.h"
```

## Inicio rápido

```cpp
#include <HardwareSerial.h>
#include "ODriveArduino.h"

HardwareSerial& odrive_serial = Serial1;
ODriveArduino odrive(odrive_serial);

void setup() {
    Serial.begin(115200);
    odrive_serial.begin(115200);

    odrive.run_state(0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true);
    odrive.run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
}

void loop() {
    odrive.SetVelocity(0, 2.0f);
    delay(10);
}
```

## API de la librería

### 1) Configuración y estado

- `void SaveConfig()`
- `void EraseConfig()`
- `void Reboot()`
- `void ClearErrors()`
- `void SendCommand(const char* command)`
- `ODriveAxisState read_state(int axis)`
- `bool run_state(int axis, ODriveAxisState requested_state, bool wait_for_idle, float timeout = 10.0f)`
- `void WaitIdle(int motor_number = -1, float timeout = 10.0f)`

Notas:

- `motor_number = -1` en `WaitIdle()` espera ambos ejes.
- `run_state()` devuelve `true` si no expira el timeout.

### 2) Lectura/escritura de propiedades genéricas

- `void WriteProperty(int motor_number, const char* property, double value)`
- `void WriteProperty(int motor_number, const char* property, float value)`
- `void WriteProperty(int motor_number, const char* property, int value)`
- `void ReadProperty(int motor_number, const char* property, double* value)`
- `void ReadProperty(int motor_number, const char* property, float* value)`
- `void ReadProperty(int motor_number, const char* property, int* value)`

Ejemplo:

```cpp
float vel_limit;
odrive.ReadProperty(0, "controller.config.vel_limit", &vel_limit);
odrive.WriteProperty(0, "controller.config.vel_limit", 15.0f);
```

### 3) Comandos de movimiento (motor individual)

- `void SetPosition(int motor_number, float position)`
- `void SetPosition(int motor_number, float position, float velocity_feedforward)`
- `void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward)`
- `void SetVelocity(int motor_number, float velocity)`
- `void SetVelocity(int motor_number, float velocity, float current_feedforward)`
- `void SetCurrent(int motor_number, float current)`
- `void TrapezoidalMove(int motor_number, float position)`

### 4) Feedback y getters

- `float GetVelocity(int motor_number)`
- `float GetPosition(int motor_number)`
- `float GetVbus()`
- `void GetFeedback(int motor_number, float* velocity_M, float* position_M)`
- `void GetFeedbackBoth(float* velocity_M0, float* position_M0, float* velocity_M1, float* position_M1)`
- `void GetFeedback_Vbus(int motor_number, float* velocity_M, float* position_M, float* vbus)`
- `void GetFeedbackBoth_Vbus(float* velocity_M0, float* position_M0, float* velocity_M1, float* position_M1, float* vbus)`

### 5) Funciones optimizadas (alta frecuencia)

- `void SetVelocityBoth(float velocity_M0, float velocity_M1)`
- `void SetVelocity_GetFeedback(int motor_number, float set_VM, float* velocity_M, float* position_M)`
- `void SetVelocityBoth_GetFeedback(float set_VM0, float set_VM1, float* velocity_M0, float* position_M0, float* velocity_M1, float* position_M1)`
- `void SetVelocityBoth_GetFeedback_Vbus(float set_VM0, float set_VM1, float* vbus, float* velocity_M0, float* position_M0, float* velocity_M1, float* position_M1)`
- `void SetVelocityBoth_GetVbus(float set_VM0, float set_VM1, float* vbus)`

Estas funciones reducen transacciones UART al combinar en una sola secuencia:

- escritura de setpoints,
- lectura de feedback,
- y opcionalmente lectura de `vbus`.

## Ejemplo de lazo optimizado

```cpp
void loop() {
    float vbus, vel0, pos0, vel1, pos1;

    float set_v0 = 2.0f;
    float set_v1 = -2.0f;

    odrive.SetVelocityBoth_GetFeedback_Vbus(
        set_v0, set_v1,
        &vbus,
        &vel0, &pos0,
        &vel1, &pos1
    );

    Serial.print("Vbus: "); Serial.print(vbus);
    Serial.print(" | M0 vel/pos: "); Serial.print(vel0); Serial.print(" / "); Serial.print(pos0);
    Serial.print(" | M1 vel/pos: "); Serial.print(vel1); Serial.print(" / "); Serial.println(pos1);

    delay(10);
}
```

## Detalles de implementación útiles

- El operador `<<` está sobrecargado para `Print`, y para `float` se imprime con 4 decimales.
- Las funciones de feedback llaman internamente a `CleanSerial()` para vaciar datos pendientes antes de la transacción.
- Lecturas de bajo nivel disponibles:
  - `float readFloat(unsigned long timeout = 100)`
  - `int32_t readInt(unsigned long timeout = 100)`
- El timeout de lectura se mide internamente con `micros()`.

## Recomendación para UART de alta velocidad (ODrive 3.6, UART A)

Si quieres usar UART a velocidades altas (por ejemplo `1M` o `2M` baudios), se recomienda eliminar el filtro pasa bajos pasivo en los GPIO 1 y 2 de ODrive 3.6 (UART A).

Estos GPIO incluyen filtro RC con jumpers soldables (SMD):

- GPIO 1:
  - `R52` en paralelo a `J5` (NO)
  - `C63` en serie con `J6` (NC)
- GPIO 2:
  - `R53` en paralelo con `J10`
  - `C64` en paralelo con `J15`

Opciones para deshabilitar el filtro:

- Opción 1:
  - Soldar puentes `J5` y `J10` (bypass de `R52` y `R53`).
  - Opcionalmente cortar `J6` y `J15` para desconectar `C63` y `C64`.
- Opción 2:
  - Remover `R52`, `R53`, `C63`, `C64`.
  - Soldar resistencias `0R` en `R52` y `R53`.

Notas:

- En algunos clones chinos, la numeración de componentes puede variar respecto al esquemático original.
- Esta modificación permite aumentar significativamente la frecuencia efectiva de actualización del setpoint de velocidad (por ejemplo, ~1000 Hz o más, según firmware, CPU y calidad de enlace UART).

## Enumeraciones

`ODriveEnums.h` incluye constantes como:

- `AXIS_STATE_*`
- `CONTROL_MODE_*`
- `INPUT_MODE_*`

Usarlas mejora la legibilidad y evita valores mágicos.

## Ejemplo incluido

Puedes revisar un ejemplo funcional en:

- `examples/ODriveArduinoTest/ODriveArduinoTest.ino`
