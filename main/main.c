#include <stdio.h> // Funciones de entrada/salida estándar
#include "freertos/FreeRTOS.h" // Núcleo de FreeRTOS para ESP32
#include "freertos/task.h" // Funciones para manejo de tareas y delays
#include "driver/gpio.h" // Control de pines GPIO
#include "driver/ledc.h" // Controlador LEDC para PWM de servomotor

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← LED: GND, GPIO9
//                   ← MG90S: 5V, GND, GPIO26 (PWM)

// PINES DE LA PLACA
#define PIN_LED 9 // GPIO9 para LED (cuidado: conflicto con SPI flash)
#define PIN_SERVO 26 // GPIO26 para señal PWM del servomotor

// CONFIGURACIÓN PWM PARA SERVOMOTOR
#define CANAL_PWM_SERVO LEDC_CHANNEL_0 // Canal 0 del controlador LEDC
#define TIMER_PWM_SERVO LEDC_TIMER_0 // Timer 0 del controlador LEDC
#define RESOLUCION_PWM_SERVO LEDC_TIMER_13_BIT // Resolución de 13 bits (8192 pasos)
#define FRECUENCIA_PWM_SERVO 50 // Frecuencia de 50Hz para servomotores (período 20ms)

// RANGO REAL MG90S CON 13 BITS A 50HZ
#define VALOR_MIN_PWM 205 // Aproximadamente 0.5ms (posición 0 grados)
#define VALOR_MAX_PWM 1024 // Aproximadamente 2.5ms (posición 180 grados)

// POSICIONES PARA PINZA (AJUSTABLES)
#define POSICION_CERRADA 0 // Pinza completamente cerrada
#define POSICION_ABIERTA 40 // Pinza abierta 40 grados

// Configuración del PWM para servomotor
static void configurar_pwm_servomotor(void)
{
    // Configurar timer PWM
    ledc_timer_config_t configuracion_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = RESOLUCION_PWM_SERVO,
        .timer_num = TIMER_PWM_SERVO,
        .freq_hz = FRECUENCIA_PWM_SERVO,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    ledc_timer_config(&configuracion_timer);

    // Configurar canal PWM
    ledc_channel_config_t configuracion_canal = {
        .gpio_num = PIN_SERVO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = CANAL_PWM_SERVO,
        .timer_sel = TIMER_PWM_SERVO,
        .duty = 0, // Ciclo de trabajo inicial 0%
        .hpoint = 0
    };
    
    ledc_channel_config(&configuracion_canal);
}

// Configuración del pin del LED
static void configurar_pin_led(void)
{
    // Configurar pin como salida GPIO
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    
    // Configurar nivel lógico inicial apagado
    gpio_set_level(PIN_LED, 0);
}

// Mover servomotor a posición específica en grados (implementación mejorada)
static void mover_servomotor_grados(int grados)
{
    // Limitar rango de grados a 0-180
    if (grados < 0) grados = 0;
    if (grados > 180) grados = 180;
    
    // Calcular valor PWM usando fórmula proporcional
    // (VALOR_MAX_PWM - VALOR_MIN_PWM) / 180 = incremento por grado
    int ciclo_trabajo = VALOR_MIN_PWM + 
                       ((VALOR_MAX_PWM - VALOR_MIN_PWM) * grados) / 180;
    
    // Establecer ciclo de trabajo
    ledc_set_duty(LEDC_LOW_SPEED_MODE, CANAL_PWM_SERVO, ciclo_trabajo);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CANAL_PWM_SERVO);
}

// Punto de partida
void app_main(void)
{
    // Configurar pin del LED
    configurar_pin_led();
    
    // Configurar PWM para servomotor
    configurar_pwm_servomotor();
    
    // Variable para estado del LED
    int estado_led = 0;
    
    // Variable para estado de la pinza
    int pinza_abierta = 0; // 0 = cerrada, 1 = abierta
    
    // Inicializar servomotor en posición cerrada
    mover_servomotor_grados(POSICION_CERRADA);
    
    // Pequeña pausa inicial
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Bucle principal: ciclos independientes de LED y servomotor
    while (1)
    {
        // CICLO DEL LED: cambiar estado cada segundo
        estado_led = !estado_led; // Cambiar estado (0→1 o 1→0)
        gpio_set_level(PIN_LED, estado_led); // Aplicar al pin físico
        
        // CICLO DEL SERVOMOTOR: alternar entre abrir y cerrar cada segundo
        if (pinza_abierta)
        {
            // Cerrar pinza
            mover_servomotor_grados(POSICION_CERRADA);
            pinza_abierta = 0; // Actualizar estado
        }
        else
        {
            // Abrir pinza
            mover_servomotor_grados(POSICION_ABIERTA);
            pinza_abierta = 1; // Actualizar estado
        }
        
        // Esperar 1 segundo para ambos ciclos
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}