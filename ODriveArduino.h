#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"
#include "ODriveEnums.h"

/** @brief Operador de impresión genérico para Stream/Print. */
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
    obj.print(arg);
    return obj;
}

/** @brief Impresión de float con 4 decimales para comandos ODrive. */
template <>
inline Print &operator<<(Print &obj, float arg)
{
    obj.print(arg, 4);
    return obj;
}

class ODriveArduino
{
public:
    /** @brief Crea la interfaz usando un Stream UART.
     *  @param serial Puerto serie usado para comunicarse con ODrive.
     */
    ODriveArduino(Stream &serial);

    /** @name Configuración rápida */
    /** @{ */
    /** @brief Guarda configuración en memoria no volátil. */
    inline void SaveConfig() { serial_ << "ss\n"; }
    /** @brief Borra configuración almacenada. */
    inline void EraseConfig() { serial_ << "se\n"; }
    /** @brief Reinicia la controladora ODrive. */
    inline void Reboot() { serial_ << "sr\n"; }
    /** @brief Limpia errores activos. */
    inline void ClearErrors() { serial_ << "sc\n"; }
    /** @brief Envía un comando ASCII directo.
     *  @param command Cadena de comando sin salto de línea.
     */
    inline void SendCommand(const char *command) { serial_ << command << "\n"; }
    /** @} */

    /** @name Acceso genérico y espera de estado */
    /** @{ */
    /** @brief Espera estado IDLE de uno o ambos ejes.
     *  @param motor_number Eje objetivo (0, 1, -1 ambos).
     *  @param timeout Tiempo máximo en segundos.
     */
    void WaitIdle(int motor_number = -1, float timeout = 10.0f);

    /** @brief Escribe propiedad de eje (double).
     *  @param motor_number Eje (0 o 1).
     *  @param property Ruta relativa a axis (ej: "controller.config.vel_limit").
     *  @param value Valor a escribir.
     */
    void WriteProperty(int motor_number, const char *property, double value);
    /** @brief Escribe propiedad de eje (float).
     *  @param motor_number Eje (0 o 1).
     *  @param property Ruta relativa a axis.
     *  @param value Valor a escribir.
     */
    void WriteProperty(int motor_number, const char *property, float value);
    /** @brief Escribe propiedad de eje (int).
     *  @param motor_number Eje (0 o 1).
     *  @param property Ruta relativa a axis.
     *  @param value Valor a escribir.
     */
    void WriteProperty(int motor_number, const char *property, int value);

    /** @brief Lee propiedad de eje en double.
     *  @param motor_number Eje (0 o 1).
     *  @param property Ruta relativa a axis.
     *  @param value Puntero donde guardar el valor leído.
     */
    void ReadProperty(int motor_number, const char *property, double *value);
    /** @brief Lee propiedad de eje en float.
     *  @param motor_number Eje (0 o 1).
     *  @param property Ruta relativa a axis.
     *  @param value Puntero donde guardar el valor leído.
     */
    void ReadProperty(int motor_number, const char *property, float *value);
    /** @brief Lee propiedad de eje en int.
     *  @param motor_number Eje (0 o 1).
     *  @param property Ruta relativa a axis.
     *  @param value Puntero donde guardar el valor leído.
     */
    void ReadProperty(int motor_number, const char *property, int *value);
    /** @} */

    /** @name Movimiento de un eje */
    /** @{ */
    /** @brief Comanda posición sin feedforward.
     *  @param motor_number Eje (0 o 1).
     *  @param position Setpoint de posición.
     */
    void SetPosition(int motor_number, float position);
    /** @brief Comanda posición con feedforward de velocidad.
     *  @param motor_number Eje (0 o 1).
     *  @param position Setpoint de posición.
     *  @param velocity_feedforward Feedforward de velocidad.
     */
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    /** @brief Comanda posición con feedforward completo.
     *  @param motor_number Eje (0 o 1).
     *  @param position Setpoint de posición.
     *  @param velocity_feedforward Feedforward de velocidad.
     *  @param current_feedforward Feedforward de corriente.
     */
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    /** @brief Comanda velocidad sin feedforward de corriente.
     *  @param motor_number Eje (0 o 1).
     *  @param velocity Setpoint de velocidad.
     */
    void SetVelocity(int motor_number, float velocity);
    /** @brief Comanda velocidad con feedforward de corriente.
     *  @param motor_number Eje (0 o 1).
     *  @param velocity Setpoint de velocidad.
     *  @param current_feedforward Feedforward de corriente.
     */
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    /** @brief Comanda corriente/torque directo.
     *  @param motor_number Eje (0 o 1).
     *  @param current Setpoint de corriente.
     */
    void SetCurrent(int motor_number, float current);
    /** @brief Movimiento trapezoidal a posición objetivo.
     *  @param motor_number Eje (0 o 1).
     *  @param position Posición objetivo.
     */
    void TrapezoidalMove(int motor_number, float position);

    /** @brief Envía velocidad y lee feedback de un eje en la misma transacción.
     *  @param motor_number Eje (0 o 1).
     *  @param set_VM Setpoint de velocidad.
     *  @param velocity_M Puntero de salida para velocidad estimada.
     *  @param position_M Puntero de salida para posición estimada.
     */
    void SetVelocity_GetFeedback(int motor_number, float set_VM, float *velocity_M, float *position_M);
    /** @} */

    /** @name Movimiento optimizado para dos ejes */
    /** @{ */
    /** @brief Comanda velocidad de ambos ejes.
     *  @param velocity_M0 Setpoint de velocidad eje 0.
     *  @param velocity_M1 Setpoint de velocidad eje 1.
     */
    void SetVelocityBoth(float velocity_M0, float velocity_M1);
    /** @brief Comanda ambos ejes y lee feedback de ambos.
     *  @param set_VM0 Setpoint de velocidad eje 0.
     *  @param set_VM1 Setpoint de velocidad eje 1.
     *  @param velocity_M0 Salida velocidad eje 0.
     *  @param position_M0 Salida posición eje 0.
     *  @param velocity_M1 Salida velocidad eje 1.
     *  @param position_M1 Salida posición eje 1.
     */
    void SetVelocityBoth_GetFeedback(float set_VM0, float set_VM1, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);
    /** @brief Comanda ambos ejes, lee feedback y `vbus`.
     *  @param set_VM0 Setpoint de velocidad eje 0.
     *  @param set_VM1 Setpoint de velocidad eje 1.
     *  @param vbus Salida de tensión de bus.
     *  @param velocity_M0 Salida velocidad eje 0.
     *  @param position_M0 Salida posición eje 0.
     *  @param velocity_M1 Salida velocidad eje 1.
     *  @param position_M1 Salida posición eje 1.
     */
    void SetVelocityBoth_GetFeedback_Vbus(float set_VM0, float set_VM1, float *vbus, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);
    /** @brief Comanda ambos ejes y lee `vbus`.
     *  @param set_VM0 Setpoint de velocidad eje 0.
     *  @param set_VM1 Setpoint de velocidad eje 1.
     *  @param vbus Salida de tensión de bus.
     */
    void SetVelocityBoth_GetVbus(float set_VM0, float set_VM1, float *vbus);
    /** @} */

    /** @name Getters y feedback */
    /** @{ */
    /** @brief Lee velocidad estimada de un eje.
     *  @param motor_number Eje (0 o 1).
     *  @return Velocidad estimada.
     */
    float GetVelocity(int motor_number);
    /** @brief Lee posición estimada de un eje.
     *  @param motor_number Eje (0 o 1).
     *  @return Posición estimada.
     */
    float GetPosition(int motor_number);
    /** @brief Lee tensión del bus DC.
     *  @return Valor de `vbus_voltage`.
     */
    float GetVbus();
    /** @brief Lee posición y velocidad de un eje.
     *  @param motor_number Eje (0 o 1).
     *  @param velocity_M Salida velocidad estimada.
     *  @param position_M Salida posición estimada.
     */
    void GetFeedback(int motor_number, float *velocity_M, float *position_M);
    /** @brief Lee feedback de ambos ejes.
     *  @param velocity_M0 Salida velocidad eje 0.
     *  @param position_M0 Salida posición eje 0.
     *  @param velocity_M1 Salida velocidad eje 1.
     *  @param position_M1 Salida posición eje 1.
     */
    void GetFeedbackBoth(float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);
    /** @brief Lee feedback de un eje y tensión de bus.
     *  @param motor_number Eje (0 o 1).
     *  @param velocity_M Salida velocidad estimada.
     *  @param position_M Salida posición estimada.
     *  @param vbus Salida tensión de bus.
     */
    void GetFeedback_Vbus(int motor_number, float *velocity_M, float *position_M, float *vbus);
    /** @brief Lee feedback de ambos ejes y tensión de bus.
     *  @param velocity_M0 Salida velocidad eje 0.
     *  @param position_M0 Salida posición eje 0.
     *  @param velocity_M1 Salida velocidad eje 1.
     *  @param position_M1 Salida posición eje 1.
     *  @param vbus Salida tensión de bus.
     */
    void GetFeedbackBoth_Vbus(float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1, float *vbus);
    /** @} */

    /** @name Lectura de bajo nivel */
    /** @{ */
    /** @brief Lee un valor float de UART.
     *  @param timeout Timeout en microsegundos.
     *  @return Valor parseado (0 si no hay dato válido).
     */
    float readFloat(unsigned long timeout = 100);
    /** @brief Lee un valor entero de UART.
     *  @param timeout Timeout en microsegundos.
     *  @return Valor parseado (0 si no hay dato válido).
     */
    int32_t readInt(unsigned long timeout = 100);
    /** @} */

    /** @name Control de estado */
    /** @{ */
    /** @brief Lee estado actual de un eje.
     *  @param axis Eje (0 o 1).
     *  @return Estado actual del eje.
     */
    ODriveAxisState read_state(int axis);
    /** @brief Solicita estado de eje y opcionalmente espera IDLE.
     *  @param axis Eje (0 o 1).
     *  @param requested_state Estado solicitado.
     *  @param wait_for_idle `true` para esperar hasta IDLE.
     *  @param timeout Timeout en segundos cuando `wait_for_idle` es `true`.
     *  @return `true` si termina antes de timeout.
     */
    bool run_state(int axis, ODriveAxisState requested_state, bool wait_for_idle, float timeout = 10.0f);
    /** @} */

    /** @brief Vacía bytes pendientes del buffer RX UART. */
    void CleanSerial();

private:
    /** @brief Lee token ASCII hasta separador o timeout.
     *  @param timeout Timeout en microsegundos.
     *  @return Token leído.
     */
    String readString(unsigned long timeout = 500);
    Stream &serial_;
};

#endif // ODriveArduino_h