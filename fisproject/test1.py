import pigpio
import time

# Iniciar el daemon de pigpio
# sudo pigpiod
# Ir al directorio del proyecto
# cd Desktop/angelo_test/
# Ejecutar el script con Python 3
# sudo python3 example.py

# Configuración de los pines GPIO
R_PWM_PIN = 12  # Pin para el ESC del motor derecho
L_PWM_PIN = 13  # Pin para el ESC del motor izquierdo

# Inicializar pigpio
pi = pigpio.pi()

# Frecuencia del PWM (normalmente los ESC usan 50 Hz)
PWM_FREQ = 50

# Rango del PWM (1000–2000 es el rango típico para ESC)
PWM_RANGE = 2000

# Inicializar PWM en los pines
pi.set_PWM_frequency(R_PWM_PIN, PWM_FREQ)
pi.set_PWM_frequency(L_PWM_PIN, PWM_FREQ)
pi.set_PWM_range(R_PWM_PIN, PWM_RANGE)
pi.set_PWM_range(L_PWM_PIN, PWM_RANGE)

# Función para ajustar el acelerador (throttle)
def set_throttle(pin, value):
    # Limitar el valor al rango permitido por el ESC
    if value < 1000:
        value = 1000
    elif value > 2000:
        value = 2000
    # Enviar ancho de pulso al ESC
    pi.set_servo_pulsewidth(pin, value)

# Arrancar el ESC enviando la señal mínima de aceleración
set_throttle(R_PWM_PIN, 1000)
set_throttle(L_PWM_PIN, 1000)
time.sleep(2)  # Esperar a que el ESC se inicialice

# Aumentar progresivamente la aceleración
for throttle in range(1000, 1100, 10):
    set_throttle(R_PWM_PIN, throttle)
    set_throttle(L_PWM_PIN, throttle)
    time.sleep(2)

# Disminuir progresivamente la aceleración
for throttle in range(1100, 1000, -10):
    set_throttle(R_PWM_PIN, throttle)
    set_throttle(L_PWM_PIN, throttle)
    time.sleep(2)

# Detener los motores enviando la señal mínima
set_throttle(R_PWM_PIN, 1000)
set_throttle(L_PWM_PIN, 1000)
time.sleep(1)

# Apagar PWM y liberar recursos
pi.set_PWM_dutycycle(R_PWM_PIN, 0)
pi.set_PWM_dutycycle(L_PWM_PIN, 0)
pi.stop()
