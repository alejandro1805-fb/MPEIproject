/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Carro autónomo STM32F103 (Blue Pill)
  *                   - Puente H TB6612FNG (avance / reversa / freno)
  *                   - 2 encoders HC-020K (medición de distancia y giros)
  *                   - Sensor ultrasónico HC-SR04 (se detiene a <= 10 cm)
  *                   - Recorre un polígono regular N lados, L cm, V vueltas
  *                   - Pantalla OLED 1.3" SPI (SH1106) con un personaje
  *                     animado que reacciona a lo que hace el carro
  *                   - Control por Bluetooth HC-05 (USART1 con interrupción)
  *                   - WiFi ESP8266 (USART2 con interrupción): envía batería,
  *                     temperatura, distancia, alertas de ruedas, color de
  *                     la pelota y estado por HTTP a Node-RED, que los guarda
  *                     en MariaDB (base "auto", tabla "lecturas")
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  ADELANTE = 0,
  ATRAS    = 1
} Sentido;

/* Lleva la cuenta de lo que cada rueda DEBIÓ recorrer y lo que REALMENTE
   recorrió, para que el error de un tramo se corrija en el siguiente.   */
typedef struct
{
  float    ideal_A;
  float    ideal_B;
  uint32_t real_A;
  uint32_t real_B;
  float    exceso_A;
  float    exceso_B;
} Acumulado;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* =====================================================================
   MODO DE FUNCIONAMIENTO
   0 = RECORRIDO del polígono
   1 = PRUEBA de motores y encoders (uno por uno)
   2 = CALIBRAR GIRO: gira en su sitio NUM_LADOS veces sin avanzar
   3 = CALIBRAR DISTANCIA: avanza en línea recta DIST_CALIBRACION_CM
   4 = SOLO PANTALLA: no mueve motores, solo muestra batería y temperatura
   5 = VER ENCODERS: con el carro levantado, cada motor gira su rueda
       5 vueltas y la pantalla muestra los pulsos de cada encoder.
   6 = BLUETOOTH: se maneja desde el celular con el HC-05 (UART1 con
       interrupción). Frena solo si hay un obstáculo a 10 cm o menos.
   7 = PRUEBA WIFI: no mueve motores. La pantalla muestra el estado del
       ESP8266 y cuántos envíos a Node-RED salieron bien o mal.

   En los modos 0, 3, 4 y 6 el carro también envía los datos por WiFi
   cada PERIODO_ENVIO_MS (si WIFI_ACTIVO = 1).
   ===================================================================== */

#define MODO                   4

/* =====================================================================
   PARÁMETROS DEL RECORRIDO  (CAMBIAR AQUÍ)
   ===================================================================== */

#define NUM_LADOS              4        /* Número de lados: 3 a 6         */
#define LONGITUD_LADO_CM       30.0f    /* Longitud de cada lado en cm    */
#define NUM_VUELTAS            1        /* Veces que recorre el polígono  */

/* =====================================================================
   CALIBRACIÓN DEL MOVIMIENTO
   ===================================================================== */

/* FACTOR_nuevo = FACTOR_actual * 360 / grados_que_giró_en_total (MODO 2) */
#define FACTOR_GIRO            1.00f

/* FACTOR_nuevo = FACTOR_actual * distancia_pedida / distancia_medida (MODO 3) */
#define FACTOR_DISTANCIA       0.94f

#define DIST_CALIBRACION_CM    50.0f
/* Pulsos que el carro se pasa por inercia al frenar (se descuentan del
   objetivo; luego el programa los va ajustando solo).
   Cada pulso de recta = 0.49 cm.                                        */
#define EXCESO_INICIAL_RECTA   1.0f
#define EXCESO_INICIAL_GIRO    1.0f

/* Pulsos que debe dar la rueda DERECHA por cada pulso de la IZQUIERDA
   para que ambas avancen lo mismo (corrige ruedas un poco distintas).
   - Si el carro se va hacia la DERECHA,   súbelo  (1.02, 1.04 ...)
   - Si el carro se va hacia la IZQUIERDA, bájalo  (0.98, 0.96 ...)     */
#define RELACION_DERECHA       1.00f

/* =====================================================================
   GEOMETRÍA DEL CARRO
   ===================================================================== */

#define DIAMETRO_RUEDA_CM      6.2f
#define ANCHO_EJE_CM           13.0f    /* Centro a centro de las ruedas  */

#define RANURAS_ENCODER        20
#define FLANCOS_POR_RANURA     2
#define PULSOS_POR_VUELTA      (RANURAS_ENCODER * FLANCOS_POR_RANURA)

#define PI_F                   3.14159265f
#define CM_POR_PULSO           ((PI_F * DIAMETRO_RUEDA_CM) / (float)PULSOS_POR_VUELTA)

/* =====================================================================
   VELOCIDADES (en % de PWM)
   ===================================================================== */

#define VEL_RECTA              55
#define VEL_GIRO               50
#define VEL_LENTA              38
#define VEL_MINIMA             30
#define VEL_PRUEBA             60
#define PULSOS_DESACELERAR     8
#define KP_SINCRONIA           3
#define CORRECCION_MAX         15

#define INVERTIR_MOTOR_A       0        /* Motor A = rueda IZQUIERDA      */
#define INVERTIR_MOTOR_B       0        /* Motor B = rueda DERECHA        */

/* =====================================================================
   SENSOR ULTRASÓNICO
   ===================================================================== */

#define DISTANCIA_LIMITE       10
#define DISTANCIA_REANUDAR     12
#define TIMEOUT_ECHO           30000
#define PERIODO_MEDICION_MS    60

/* =====================================================================
   ADC: BATERÍA Y LM35
   ===================================================================== */

/* Voltaje de referencia del ADC (pin 3.3 V de la Blue Pill).
   Para más precisión, mídelo con el multímetro y ponlo aquí.            */
#define VDDA_V                 3.30f

/* Divisor de voltaje de la batería:
   Batería(+) --- R_ARRIBA ---+--- R_ABAJO --- GND
                              |
                             PB0
   Con 20k y 10k se pueden medir hasta 9.9 V.                            */
#define R_ARRIBA_OHM           20000.0f
#define R_ABAJO_OHM            10000.0f
#define DIVISOR_BATERIA        ((R_ARRIBA_OHM + R_ABAJO_OHM) / R_ABAJO_OHM)

/* Ajuste fino: compara con el multímetro.
   FACTOR_nuevo = FACTOR_actual * voltaje_multimetro / voltaje_pantalla   */
#define FACTOR_BATERIA         1.00f

/* =====================================================================
   BLUETOOTH HC-05 (USART1)
   Comandos de la app "Arduino Car" (ONE DAY OF CODE):
     F adelante   B atrás   L izquierda   R derecha   S parar
     Q adelante-izq   E adelante-der   Z atrás-izq   C atrás-der
     X sube velocidad   Y baja velocidad
     (luces M/m N/n, servos J/K, T y D se ignoran)
   ===================================================================== */

#define BT_BAUDIOS             9600     /* Velocidad por defecto del HC-05 */
#define VEL_BT_INICIAL         60       /* % de PWM al arrancar            */
#define PASO_VEL_BT            10       /* Cuánto cambian X / Y la velocidad */
#define BT_BUFFER              64       /* Bytes guardados por la interrupción */

/* Si pasa este tiempo sin recibir nada, el carro se detiene (seguridad
   por si se pierde la conexión). 0 = desactivado.                       */
#define TIMEOUT_BT_MS          0

/* =====================================================================
   WIFI ESP8266 (USART2)  ->  Node-RED (HTTP)  ->  MariaDB
   El carro hace un POST con los datos en JSON a:
       http://SERVIDOR_IP:SERVIDOR_PUERTO/carro
   ===================================================================== */

#define WIFI_ACTIVO            1        /* 0 = no usar el módulo WiFi       */

/* Red WiFi de 2.4 GHz (el ESP8266 NO funciona en 5 GHz). Lo más fácil
   es usar el hotspot del celular, con el Mac conectado a la misma red. */
#define WIFI_SSID     "SANBAL_2G"
#define WIFI_CLAVE             "52365239"

/* IP del Mac donde corre Node-RED (en el Mac: Ajustes del Sistema ->
   Wi-Fi -> Detalles de la red -> Dirección IP)                          */
#define SERVIDOR_IP            "192.168.128.18"
#define SERVIDOR_PUERTO        "1880"   /* Puerto de Node-RED (texto)     */
#define SERVIDOR_RUTA          "/carro"

#define PERIODO_ENVIO_MS       2000     /* Cada cuánto se envían los datos */
#define ESP_BAUDIOS            115200   /* Si no responde, prueba 9600     */
#define ESP_BUFFER             512      /* Bytes guardados por la interrupción */

/* Datos de la misión que se guardan en la base de datos.
   Cuando agreguen el sensor de color, asignen la variable color_pelota
   con el color detectado (máx. 20 letras).
   estado_carro solo puede ser "buscar_pelota" o "buscar_canasta".      */
#define COLOR_PELOTA           "rojo"
#define ESTADO_INICIAL         "buscar_pelota"

#define CANAL_BATERIA          8        /* PB0 = ADC1_IN8                 */
#define CANAL_LM35             9        /* PB1 = ADC1_IN9                 */
#define MUESTRAS_ADC           16       /* Promedio para quitar ruido     */

/* =====================================================================
   SEGURIDAD
   ===================================================================== */

#define TIMEOUT_ATASCO_MS      1500
#define PULSOS_MINIMOS_PRUEBA  5

/* =====================================================================
   OTROS
   ===================================================================== */

#define PAUSA_ENTRE_TRAMOS_MS  400
#define ESPERA_INICIAL_MS      2000
#define DEBOUNCE_US            250      /* Filtro de ruido del encoder.
                                           Si es muy alto, se pierden
                                           pulsos a alta velocidad.      */
#define VUELTAS_PRUEBA_ENC     5        /* Vueltas por rueda en MODO 5  */
#define PWM_PERIODO            3600     /* 72 MHz / 3600 = 20 kHz         */
#define PERIODO_PANTALLA_MS    100      /* Refresco de la OLED (animación) */
#define TIEMPO_DORMIR_MS       8000     /* Quieta este tiempo -> se duerme */

/* Rango de la batería para el ícono (vacía -> llena). Ajústalo a tu
   batería: 2 celdas de litio = 6.4 V a 8.4 V                            */
#define BATERIA_MIN_V          6.4f
#define BATERIA_MAX_V          8.4f

/* Controlador de la OLED: 1 = SH1106 (la de 1.3"), 0 = SSD1306 (0.96").
   Si la imagen sale corrida 2 píxeles o con basura a un lado, cámbialo. */
#define OLED_SH1106            1
/* 1 = girar la imagen 180° (si la pantalla queda de cabeza en el carro) */
#define OLED_VOLTEAR           0
#define TIM2_PERIODO           60000U   /* TIM2 cuenta 0..59999 us        */

/* =====================================================================
   PINES
   ===================================================================== */

/* HC-SR04 */
#define ECHO_PORT     GPIOA
#define ECHO_PIN      GPIO_PIN_0
#define TRIG_PORT     GPIOA
#define TRIG_PIN      GPIO_PIN_1

/* TB6612FNG */
/* ¡CAMBIÓ! AIN1/AIN2 pasaron de PA2/PA3 a PB10/PB11 para dejar la
   USART2 (PA2/PA3) al módulo WiFi. (El STM32F103x6 no tiene USART3.)  */
#define AIN1_PORT     GPIOB
#define AIN1_PIN      GPIO_PIN_10
#define AIN2_PORT     GPIOB
#define AIN2_PIN      GPIO_PIN_11
#define BIN1_PORT     GPIOA
#define BIN1_PIN      GPIO_PIN_4
#define BIN2_PORT     GPIOA
#define BIN2_PIN      GPIO_PIN_5
#define STBY_PORT     GPIOA
#define STBY_PIN      GPIO_PIN_8        /* ¡CAMBIÓ! Antes era PB0         */
/* PWMA -> PA6 (TIM3_CH1)   PWMB -> PA7 (TIM3_CH2) */

/* Encoders HC-020K */
#define ENC_A_PORT    GPIOB
#define ENC_A_PIN     GPIO_PIN_6        /* Rueda izquierda */
#define ENC_B_PORT    GPIOB
#define ENC_B_PIN     GPIO_PIN_7        /* Rueda derecha   */

/* Bluetooth HC-05 (USART1) */
#define BT_PORT       GPIOA
#define BT_TX_PIN     GPIO_PIN_9        /* PA9  -> RXD del HC-05 */
#define BT_RX_PIN     GPIO_PIN_10       /* PA10 <- TXD del HC-05 */

/* Entradas analógicas */
#define BAT_PORT      GPIOB
#define BAT_PIN       GPIO_PIN_0        /* ADC1_IN8 */
#define LM35_PORT     GPIOB
#define LM35_PIN      GPIO_PIN_1        /* ADC1_IN9 */

/* WiFi ESP8266 (USART2) */
#define ESP_PORT      GPIOA
#define ESP_TX_PIN    GPIO_PIN_2        /* PA2 -> RX del ESP8266 */
#define ESP_RX_PIN    GPIO_PIN_3        /* PA3 <- TX del ESP8266 */

/* Pantalla OLED 1.3" SPI de 7 pines (SPI por software) */
#define OLED_PORT     GPIOB
#define OLED_CS       GPIO_PIN_8        /* CS   */
#define OLED_RES      GPIO_PIN_12       /* RES  */
#define OLED_CLK      GPIO_PIN_13       /* CLK  */
#define OLED_DC       GPIO_PIN_14       /* DC   */
#define OLED_MOSI     GPIO_PIN_15       /* MOSI */

/* LED de la Blue Pill (activo en bajo) */
#define LED_PORT      GPIOC
#define LED_PIN       GPIO_PIN_13
#define LED_ON()      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()     HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

/* Validación de parámetros en compilación */
#if (NUM_LADOS < 3) || (NUM_LADOS > 6)
#error "NUM_LADOS debe estar entre 3 y 6"
#endif

#if (NUM_VUELTAS < 1)
#error "NUM_VUELTAS debe ser 1 o más"
#endif

#if (MODO < 0) || (MODO > 7)
#error "MODO debe ser 0, 1, 2, 3, 4, 5, 6 o 7"
#endif

#if (MODO == 7) && (WIFI_ACTIVO == 0)
#error "El MODO 7 necesita WIFI_ACTIVO = 1"
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* Variables compartidas con la interrupción de los encoders */
volatile uint32_t cuenta_A = 0;
volatile uint32_t cuenta_B = 0;
volatile uint32_t objetivo_A = 0;
volatile uint32_t objetivo_B = 0;
volatile uint8_t  fin_A = 0;
volatile uint8_t  fin_B = 0;
volatile uint8_t  tramo_activo = 0;
volatile uint32_t ultimo_A_us = 0;
volatile uint32_t ultimo_B_us = 0;

volatile uint8_t  led_encoder = 0;
volatile uint8_t  en_recta = 0;     /* 1 mientras se recorre un lado */

/* Resultados (visibles en Live Expressions del depurador) */
volatile uint32_t prueba_izq_adelante = 0;
volatile uint32_t prueba_izq_atras    = 0;
volatile uint32_t prueba_der_adelante = 0;
volatile uint32_t prueba_der_atras    = 0;
volatile uint8_t  codigo_error        = 0;
volatile float    bateria_v           = 0.0f;
volatile float    temperatura_c       = 0.0f;
volatile float    distancia_cm        = 0.0f;

static Acumulado acc_recta;
static Acumulado acc_giro;

/* Buffer circular que llena la interrupción de la UART (Bluetooth) */
volatile uint8_t bt_buffer[BT_BUFFER];
volatile uint8_t bt_cabeza = 0;     /* Donde escribe la interrupción */
volatile uint8_t bt_cola   = 0;     /* Donde lee el programa principal */

/* Buffer circular que llena la interrupción de la UART2 (WiFi) */
volatile uint8_t  esp_buffer[ESP_BUFFER];
volatile uint16_t esp_cabeza = 0;
volatile uint16_t esp_cola   = 0;

/* Estado que se envía a la base de datos */
volatile uint8_t  hay_obstaculo = 0;
volatile uint8_t  alerta_llantas = 0;   /* 0 ninguna, 1 izq, 2 der, 3 ambas */
const char       *color_pelota  = COLOR_PELOTA;
const char       *estado_carro  = ESTADO_INICIAL;

/* Para la carita de la OLED */
volatile char     movimiento       = 'S';  /* F, B, L, R o S (quieto)      */
volatile uint8_t  mision_terminada = 0;
volatile uint8_t  wifi_en_uso      = 0;    /* 1 si este modo usa el WiFi   */
volatile uint8_t  wifi_ok          = 0;    /* 1 si el último envío llegó   */

/* Resultados del WiFi (visibles en Live Expressions) */
volatile uint32_t envios_ok    = 0;
volatile uint32_t envios_error = 0;
const char       *wifi_texto   = "WiFi apagado";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */

static uint32_t Micros(void);
static uint32_t Medir_Distancia_cm(void);

static void MotorA_Direccion(Sentido s);
static void MotorB_Direccion(Sentido s);
static void MotorA_Freno(void);
static void MotorB_Freno(void);
static void MotorA_Libre(void);
static void MotorB_Libre(void);
static void MotorA_Velocidad(int32_t pct);
static void MotorB_Velocidad(int32_t pct);

static void OLED_Iniciar(void);
static void OLED_Enviar(void);
__attribute__((unused)) static void Pantalla_Texto(uint8_t fila, const char *texto);
static void Pantalla_Saludo(const char *mensaje);

static void     ADC_Iniciar(void);
static uint16_t ADC_Leer(uint32_t canal);
static float    ADC_Voltios(uint32_t canal);

__attribute__((unused)) static char *Agregar_Texto(char *p, const char *s);
__attribute__((unused)) static char *Agregar_Numero(char *p, uint32_t n);
__attribute__((unused)) static char *Agregar_Decimal(char *p, uint32_t valor, uint8_t decimales);
static float Distancia_Recorrida_cm(void);
static void  Leer_Sensores(void);
static void  Pantalla_Actualizar(void);

__attribute__((unused)) static void WiFi_Iniciar(void);
static void  WiFi_Tarea(void);

static void Iniciar_Acumulado(Acumulado *acc, float exceso_inicial);
static uint32_t Calcular_Objetivo(float ideal, uint32_t real, float exceso);
static void Arrancar_Motores(Sentido sA, Sentido sB);
static void Ajustar_Velocidades(int32_t vel, uint32_t objA, uint32_t objB);
static void Pausar_Por_Obstaculo(Sentido sA, Sentido sB);
__attribute__((unused))
static void Ejecutar_Tramo(Sentido sA, Sentido sB, float pulsos,
                           Acumulado *acc, int32_t vel,
                           uint8_t vigilar_obstaculo);

__attribute__((unused)) static const char *Texto_Alerta(uint8_t n);
static void Esperar_ms(uint32_t ms);
static void Senal_Error(uint8_t n);
static void Falla(uint8_t n);

#if (MODO == 1)
static uint32_t Probar_Motor(uint8_t motor, Sentido s, uint32_t ms);
static void Prueba_Hardware(void);
#endif

#if (MODO == 6)
static void    BT_Iniciar(void);
static int     BT_Leer(void);
static void    BT_Enviar_Texto(const char *s);
static uint8_t Es_Adelante(char m);
static void    BT_Guardar_Distancia(void);
static void    BT_Aplicar(char m);
static uint8_t BT_Procesar(int c);
static void    Modo_Bluetooth(void);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =====================================================================
   TIEMPO
   ===================================================================== */

/* Microsegundos con SysTick (se usa en la interrupción de encoders) */
static uint32_t Micros(void)
{
  uint32_t ms  = HAL_GetTick();
  uint32_t val = SysTick->VAL;

  if (ms != HAL_GetTick())
  {
    ms  = HAL_GetTick();
    val = SysTick->VAL;
  }

  return (ms * 1000U) +
         ((SysTick->LOAD + 1U - val) / (SystemCoreClock / 1000000U));
}

/* =====================================================================
   HC-SR04: devuelve la distancia en cm (999 si no hay eco)
   ===================================================================== */

static uint32_t Medir_Distancia_cm(void)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
  while (__HAL_TIM_GET_COUNTER(&htim2) < 10) { }
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
  {
    if (__HAL_TIM_GET_COUNTER(&htim2) > TIMEOUT_ECHO)
    {
      return 999;
    }
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
  {
    if (__HAL_TIM_GET_COUNTER(&htim2) > TIMEOUT_ECHO)
    {
      return 999;
    }
  }

  return __HAL_TIM_GET_COUNTER(&htim2) / 58;
}

/* =====================================================================
   CONTROL DEL TB6612FNG
   ===================================================================== */

static void MotorA_Direccion(Sentido s)
{
#if INVERTIR_MOTOR_A
  s = (s == ADELANTE) ? ATRAS : ADELANTE;
#endif
  if (s == ADELANTE)
  {
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
  }
}

static void MotorB_Direccion(Sentido s)
{
#if INVERTIR_MOTOR_B
  s = (s == ADELANTE) ? ATRAS : ADELANTE;
#endif
  if (s == ADELANTE)
  {
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
  }
}

static void MotorA_Freno(void)
{
  HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
}

static void MotorB_Freno(void)
{
  HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
}

static void MotorA_Libre(void)
{
  HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
}

static void MotorB_Libre(void)
{
  HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
}

static void MotorA_Velocidad(int32_t pct)
{
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(pct * PWM_PERIODO / 100));
}

static void MotorB_Velocidad(int32_t pct)
{
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)(pct * PWM_PERIODO / 100));
}

/* =====================================================================
   PANTALLA OLED 1.3" (SH1106 / SSD1306) POR SPI DE SOFTWARE
   Se dibuja todo en oled_buf (memoria del micro) y luego OLED_Enviar()
   pasa la imagen completa a la pantalla.
   ===================================================================== */

#define OLED_ALTO_PIN(p)  (OLED_PORT->BSRR = (p))
#define OLED_BAJO_PIN(p)  (OLED_PORT->BRR  = (p))

static void OLED_Byte(uint8_t b)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (b & 0x80) OLED_ALTO_PIN(OLED_MOSI); else OLED_BAJO_PIN(OLED_MOSI);
    OLED_ALTO_PIN(OLED_CLK);
    OLED_BAJO_PIN(OLED_CLK);
    b <<= 1;
  }
}

static void OLED_Comando(uint8_t c)
{
  OLED_BAJO_PIN(OLED_DC);              /* DC = 0: comando */
  OLED_BAJO_PIN(OLED_CS);
  OLED_Byte(c);
  OLED_ALTO_PIN(OLED_CS);
}

static void OLED_Iniciar(void)
{
  OLED_ALTO_PIN(OLED_CS);
  OLED_BAJO_PIN(OLED_CLK);

  OLED_ALTO_PIN(OLED_RES);             /* Reinicio por hardware */
  HAL_Delay(5);
  OLED_BAJO_PIN(OLED_RES);
  HAL_Delay(10);
  OLED_ALTO_PIN(OLED_RES);
  HAL_Delay(10);

  OLED_Comando(0xAE);                  /* Pantalla apagada              */
  OLED_Comando(0xD5); OLED_Comando(0x80);   /* Reloj                    */
  OLED_Comando(0xA8); OLED_Comando(0x3F);   /* 64 filas                 */
  OLED_Comando(0xD3); OLED_Comando(0x00);   /* Sin desplazamiento       */
  OLED_Comando(0x40);                  /* Línea de inicio 0             */
#if OLED_SH1106
  OLED_Comando(0xAD); OLED_Comando(0x8B);   /* Fuente interna (SH1106)  */
#else
  OLED_Comando(0x8D); OLED_Comando(0x14);   /* Bomba de carga (SSD1306) */
  OLED_Comando(0x20); OLED_Comando(0x02);   /* Direccionamiento por página */
#endif
#if OLED_VOLTEAR
  OLED_Comando(0xA0);                  /* Sin espejo horizontal         */
  OLED_Comando(0xC0);                  /* Barrido normal                */
#else
  OLED_Comando(0xA1);                  /* Espejo horizontal             */
  OLED_Comando(0xC8);                  /* Barrido invertido             */
#endif
  OLED_Comando(0xDA); OLED_Comando(0x12);
  OLED_Comando(0x81); OLED_Comando(0xCF);   /* Contraste                */
  OLED_Comando(0xD9); OLED_Comando(0xF1);
  OLED_Comando(0xDB); OLED_Comando(0x40);
  OLED_Comando(0xA4);                  /* Mostrar la memoria            */
  OLED_Comando(0xA6);                  /* Normal (no invertido)         */
  OLED_Comando(0xAF);                  /* Pantalla encendida            */
}

/* =====================================================================
   GRÁFICOS PARA LA OLED 128x64 (memoria de imagen + dibujo)
   ===================================================================== */

#define OLED_ANCHO   128
#define OLED_ALTO    64

static uint8_t oled_buf[OLED_ANCHO * OLED_ALTO / 8];   /* 1 bit por píxel */

static const uint8_t fuente5x7[] =
{
  0x00, 0x00, 0x00, 0x00, 0x00,  /*   */
  0x00, 0x00, 0x2F, 0x00, 0x00,  /* ! */
  0x00, 0x07, 0x00, 0x07, 0x00,  /* " */
  0x14, 0x3E, 0x14, 0x3E, 0x14,  /* # */
  0x04, 0x2A, 0x3E, 0x2A, 0x10,  /* $ */
  0x13, 0x08, 0x04, 0x32, 0x00,  /* % */
  0x14, 0x2A, 0x14, 0x20, 0x00,  /* & */
  0x00, 0x00, 0x07, 0x00, 0x00,  /* comilla */
  0x00, 0x1E, 0x21, 0x00, 0x00,  /* ( */
  0x00, 0x21, 0x1E, 0x00, 0x00,  /* ) */
  0x00, 0x2A, 0x1C, 0x2A, 0x00,  /*  *  */
  0x08, 0x08, 0x3E, 0x08, 0x08,  /* + */
  0x00, 0x40, 0x30, 0x10, 0x00,  /* , */
  0x08, 0x08, 0x08, 0x08, 0x00,  /* - */
  0x00, 0x30, 0x30, 0x00, 0x00,  /* . */
  0x10, 0x08, 0x04, 0x02, 0x00,  /*  /  */
  0x00, 0x1E, 0x21, 0x1E, 0x00,  /* 0 */
  0x00, 0x22, 0x3F, 0x20, 0x00,  /* 1 */
  0x22, 0x31, 0x29, 0x26, 0x00,  /* 2 */
  0x11, 0x25, 0x25, 0x1B, 0x00,  /* 3 */
  0x0C, 0x0A, 0x3F, 0x08, 0x00,  /* 4 */
  0x17, 0x25, 0x25, 0x19, 0x00,  /* 5 */
  0x1E, 0x25, 0x25, 0x18, 0x00,  /* 6 */
  0x01, 0x31, 0x0D, 0x03, 0x00,  /* 7 */
  0x1A, 0x25, 0x25, 0x1A, 0x00,  /* 8 */
  0x06, 0x29, 0x29, 0x1E, 0x00,  /* 9 */
  0x00, 0x36, 0x36, 0x00, 0x00,  /* : */
  0x40, 0x36, 0x16, 0x00, 0x00,  /* ; */
  0x00, 0x08, 0x14, 0x22, 0x00,  /* < */
  0x14, 0x14, 0x14, 0x14, 0x00,  /* = */
  0x00, 0x22, 0x14, 0x08, 0x00,  /* > */
  0x00, 0x02, 0x29, 0x06, 0x00,  /* ? */
  0x1E, 0x21, 0x2D, 0x0E, 0x00,  /* @ */
  0x3E, 0x09, 0x09, 0x3E, 0x00,  /* A */
  0x3F, 0x25, 0x25, 0x1A, 0x00,  /* B */
  0x1E, 0x21, 0x21, 0x12, 0x00,  /* C */
  0x3F, 0x21, 0x21, 0x1E, 0x00,  /* D */
  0x3F, 0x25, 0x25, 0x21, 0x00,  /* E */
  0x3F, 0x05, 0x05, 0x01, 0x00,  /* F */
  0x1E, 0x21, 0x29, 0x3A, 0x00,  /* G */
  0x3F, 0x04, 0x04, 0x3F, 0x00,  /* H */
  0x00, 0x21, 0x3F, 0x21, 0x00,  /* I */
  0x10, 0x20, 0x20, 0x1F, 0x00,  /* J */
  0x3F, 0x0C, 0x12, 0x21, 0x00,  /* K */
  0x3F, 0x20, 0x20, 0x20, 0x00,  /* L */
  0x3F, 0x06, 0x06, 0x3F, 0x00,  /* M */
  0x3F, 0x06, 0x18, 0x3F, 0x00,  /* N */
  0x1E, 0x21, 0x21, 0x1E, 0x00,  /* O */
  0x3F, 0x09, 0x09, 0x06, 0x00,  /* P */
  0x1E, 0x31, 0x21, 0x5E, 0x00,  /* Q */
  0x3F, 0x09, 0x19, 0x26, 0x00,  /* R */
  0x12, 0x25, 0x29, 0x12, 0x00,  /* S */
  0x00, 0x01, 0x3F, 0x01, 0x00,  /* T */
  0x1F, 0x20, 0x20, 0x1F, 0x00,  /* U */
  0x0F, 0x30, 0x30, 0x0F, 0x00,  /* V */
  0x3F, 0x18, 0x18, 0x3F, 0x00,  /* W */
  0x33, 0x0C, 0x0C, 0x33, 0x00,  /* X */
  0x00, 0x07, 0x38, 0x07, 0x00,  /* Y */
  0x31, 0x29, 0x25, 0x23, 0x00,  /* Z */
  0x00, 0x3F, 0x21, 0x21, 0x00,  /* [ */
  0x02, 0x04, 0x08, 0x10, 0x00,  /* barra */
  0x00, 0x21, 0x21, 0x3F, 0x00,  /* ] */
  0x00, 0x02, 0x01, 0x02, 0x00,  /* ^ */
  0x20, 0x20, 0x20, 0x20, 0x00,  /* _ */
  0x00, 0x01, 0x02, 0x00, 0x00,  /* ` */
  0x18, 0x24, 0x14, 0x3C, 0x00,  /* a */
  0x3F, 0x24, 0x24, 0x18, 0x00,  /* b */
  0x18, 0x24, 0x24, 0x00, 0x00,  /* c */
  0x18, 0x24, 0x24, 0x3F, 0x00,  /* d */
  0x18, 0x34, 0x2C, 0x08, 0x00,  /* e */
  0x08, 0x3E, 0x09, 0x02, 0x00,  /* f */
  0x28, 0x54, 0x54, 0x4C, 0x00,  /* g */
  0x3F, 0x04, 0x04, 0x38, 0x00,  /* h */
  0x00, 0x24, 0x3D, 0x20, 0x00,  /* i */
  0x00, 0x20, 0x40, 0x3D, 0x00,  /* j */
  0x3F, 0x08, 0x14, 0x20, 0x00,  /* k */
  0x00, 0x21, 0x3F, 0x20, 0x00,  /* l */
  0x3C, 0x08, 0x0C, 0x38, 0x00,  /* m */
  0x3C, 0x04, 0x04, 0x38, 0x00,  /* n */
  0x18, 0x24, 0x24, 0x18, 0x00,  /* o */
  0x7C, 0x24, 0x24, 0x18, 0x00,  /* p */
  0x18, 0x24, 0x24, 0x7C, 0x00,  /* q */
  0x3C, 0x04, 0x04, 0x08, 0x00,  /* r */
  0x28, 0x2C, 0x34, 0x14, 0x00,  /* s */
  0x04, 0x1F, 0x24, 0x20, 0x00,  /* t */
  0x1C, 0x20, 0x20, 0x3C, 0x00,  /* u */
  0x00, 0x1C, 0x20, 0x1C, 0x00,  /* v */
  0x3C, 0x30, 0x30, 0x3C, 0x00,  /* w */
  0x24, 0x18, 0x18, 0x24, 0x00,  /* x */
  0x0C, 0x50, 0x20, 0x1C, 0x00,  /* y */
  0x24, 0x34, 0x2C, 0x24, 0x00,  /* z */
  0x00, 0x04, 0x1E, 0x21, 0x00,  /* { */
  0x00, 0x00, 0x3F, 0x00, 0x00,  /* | */
  0x00, 0x21, 0x1E, 0x04, 0x00,  /* } */
  0x02, 0x01, 0x02, 0x01, 0x00,  /* ~ */
};

static void G_Limpiar(void)
{
  for (uint16_t i = 0; i < sizeof(oled_buf); i++) oled_buf[i] = 0;
}

static void G_Pixel(int x, int y, uint8_t on)
{
  if ((x < 0) || (x >= OLED_ANCHO) || (y < 0) || (y >= OLED_ALTO)) return;
  if (on) oled_buf[x + (y >> 3) * OLED_ANCHO] |=  (uint8_t)(1U << (y & 7));
  else    oled_buf[x + (y >> 3) * OLED_ANCHO] &= (uint8_t)~(1U << (y & 7));
}

static void G_Rect(int x, int y, int w, int h, uint8_t on)
{
  for (int j = y; j < y + h; j++)
    for (int i = x; i < x + w; i++)
      G_Pixel(i, j, on);
}

static void G_Linea(int x0, int y0, int x1, int y1, uint8_t on)
{
  int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  int dy = (y1 > y0) ? (y0 - y1) : (y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  while (1)
  {
    G_Pixel(x0, y0, on);
    if ((x0 == x1) && (y0 == y1)) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

/* Línea de 2 píxeles de grosor */
static void G_Linea2(int x0, int y0, int x1, int y1, uint8_t on)
{
  G_Linea(x0, y0, x1, y1, on);
  G_Linea(x0, y0 + 1, x1, y1 + 1, on);
}

/* Elipse rellena. tramado = 1 dibuja solo la mitad de los puntos
   (efecto de "rubor" en las mejillas)                                  */
static void G_Elipse(int cx, int cy, int rx, int ry, uint8_t on, uint8_t tramado)
{
  long rx2 = (long)rx * rx;
  long ry2 = (long)ry * ry;
  long lim = rx2 * ry2;

  for (int y = -ry; y <= ry; y++)
    for (int x = -rx; x <= rx; x++)
      if ((long)x * x * ry2 + (long)y * y * rx2 <= lim)
      {
        if (tramado && (((cx + x) + (cy + y)) & 1)) continue;
        G_Pixel(cx + x, cy + y, on);
      }
}

/* Arco de media elipse (abajo = 1: sonrisa ‿ ; abajo = 0: ⌒), grosor 2 */
static void G_Arco(int cx, int cy, int rx, int ry, uint8_t abajo, uint8_t on)
{
  for (int g = 0; g < 2; g++)
  {
    int r_y = ry - g;
    int r_x = rx - g;
    if ((r_x <= 0) || (r_y <= 0)) break;
    int px = -r_x, py = 0;
    for (int x = -r_x; x <= r_x; x++)
    {
      /* y = r_y * sqrt(1 - x²/r_x²), sin usar float */
      long t = (long)r_y * r_y * ((long)r_x * r_x - (long)x * x) / ((long)r_x * r_x);
      int y = 0;
      while ((long)(y + 1) * (y + 1) <= t) y++;
      if (!abajo) y = -y;
      if (x != -r_x) G_Linea(cx + px, cy + py, cx + x, cy + y, on);
      px = x; py = y;
    }
  }
}

static void G_Texto(int x, int y, const char *s, uint8_t on)
{
  while (*s)
  {
    char c = *s++;
    if ((c < 32) || (c > 126)) c = '?';
    const uint8_t *col = &fuente5x7[(c - 32) * 5];
    for (int i = 0; i < 5; i++)
      for (int j = 0; j < 8; j++)
        if (col[i] & (1U << j)) G_Pixel(x + i, y + j, on);
    x += 6;
  }
}

static int G_Ancho_Texto(const char *s)
{
  int n = 0;
  while (*s++) n++;
  return (n > 0) ? (n * 6 - 1) : 0;
}

static void G_Texto_Centrado(int y, const char *s, uint8_t on)
{
  G_Texto((OLED_ANCHO - G_Ancho_Texto(s)) / 2, y, s, on);
}

/* =====================================================================
   PERSONAJE "CHISPA" (diseño original)
   Carita con flequillo, ojos grandes con brillo, mejillas sonrojadas.
   ===================================================================== */

typedef enum
{
  CARA_NORMAL = 0,     /* quieta, despierta, mira a los lados        */
  CARA_FELIZ,          /* avanzando                                   */
  CARA_SORPRESA,       /* obstáculo                                   */
  CARA_TRISTE,         /* falla una rueda                             */
  CARA_DORMIDA,        /* mucho tiempo quieta                         */
  CARA_FIESTA          /* terminó la misión                           */
} Expresion;

#define OJO_IZQ_X   38
#define OJO_DER_X   90
#define OJO_Y       33
#define BOCA_X      64
#define BOCA_Y      47

/* Pelo: flequillo en ondas y mechones a los lados (enmarca la cara) */
static void Dibujar_Pelo(void)
{
  G_Rect(0, 0, OLED_ANCHO, 4, 1);
  for (int x = 4; x < OLED_ANCHO + 8; x += 16)
    G_Elipse(x, 2, 10, 10, 1, 0);

  /* Mechones laterales que bajan y se curvan hacia adentro */
  for (int y = 0; y < 58; y++)
  {
    int ancho = 10 - (y * y) / 360;
    if (ancho < 2) ancho = 2;
    G_Rect(0, y, ancho, 1, 1);
    G_Rect(OLED_ANCHO - ancho, y, ancho, 1, 1);
  }

  /* Líneas de textura del pelo */
  G_Linea(28, 3, 30, 9, 0);
  G_Linea(52, 4, 51, 10, 0);
  G_Linea(76, 4, 77, 10, 0);
}

/* Broche en forma de corazón, recortado sobre el flequillo */
static void Dibujar_Broche(int cx, int cy)
{
  G_Elipse(cx - 2, cy - 1, 2, 2, 0, 0);
  G_Elipse(cx + 2, cy - 1, 2, 2, 0, 0);
  for (int i = 0; i <= 4; i++)
    G_Linea(cx - 4 + i, cy + i, cx + 4 - i, cy + i, 0);
}

/* Pestañas en la esquina de afuera del ojo */
static void Dibujar_Pestanas(int cx, int cy, int rx, int ry, uint8_t derecho)
{
  int s = derecho ? 1 : -1;
  G_Linea2(cx + s * (rx + 1), cy - ry + 5, cx + s * (rx + 5), cy - ry + 1, 1);
  G_Linea2(cx + s * (rx + 2), cy - ry + 9, cx + s * (rx + 6), cy - ry + 7, 1);
}

/* Ojo abierto estilo anime:
   línea del párpado arriba, ojo claro, pupila grande con reflejo
   abajo y dos brillos. mx, my: hacia dónde mira.                       */
static void Dibujar_Ojo(int cx, int cy, int mx, int my, uint8_t grande, uint8_t derecho)
{
  int rx = grande ? 11 : 9;
  int ry = grande ? 13 : 11;

  /* Línea del párpado: elipse más grande, recortada a la parte de arriba */
  G_Elipse(cx, cy, rx + 3, ry + 3, 1, 0);
  G_Rect(cx - rx - 4, cy - ry / 3, 2 * rx + 9, ry + ry / 3 + 5, 0);
  G_Elipse(cx, cy, rx + 1, ry + 1, 0, 0);

  G_Elipse(cx, cy, rx, ry, 1, 0);                          /* ojo          */

  if (grande)                                              /* asustada     */
  {
    G_Elipse(cx + mx, cy + my, 4, 5, 0, 0);
    G_Pixel(cx + mx - 1, cy + my - 2, 1);
  }
  else
  {
    G_Elipse(cx + mx, cy + my + 1, 6, 8, 0, 0);            /* pupila       */
    G_Elipse(cx + mx, cy + my + 6, 3, 2, 1, 1);            /* reflejo abajo */
    G_Elipse(cx + mx - 2, cy + my - 3, 2, 2, 1, 0);        /* brillo grande */
    G_Rect(cx + mx + 2, cy + my + 1, 2, 2, 1);             /* brillo chico */
  }

  Dibujar_Pestanas(cx, cy, rx, ry, derecho);
}

/* Párpado triste: tapa la parte de arriba del ojo en diagonal */
static void Tapar_Triste(int cx, int cy, uint8_t derecho)
{
  for (int x = cx - 13; x <= cx + 13; x++)
  {
    int d = derecho ? (x - cx) : (cx - x);      /* más tapado hacia afuera */
    int y_lim = cy - 6 + d / 3;
    for (int y = cy - 16; y < y_lim; y++) G_Pixel(x, y, 0);
  }
}

/* Ojo cerrado (dormida o parpadeo): curva ‿ con pestañas */
static void Dibujar_Ojo_Cerrado(int cx, int cy, uint8_t derecho)
{
  int s = derecho ? 1 : -1;
  G_Arco(cx, cy + 1, 10, 5, 1, 1);
  G_Linea2(cx + s * 9, cy + 1, cx + s * 13, cy - 2, 1);
  G_Linea2(cx + s * 5, cy + 5, cx + s * 7, cy + 9, 1);
}

/* Ojo feliz: forma de ^ */
static void Dibujar_Ojo_Feliz(int cx, int cy)
{
  G_Arco(cx, cy + 4, 9, 8, 0, 1);
}

static void Dibujar_Mejillas(void)
{
  G_Elipse(OJO_IZQ_X - 13, 47, 6, 3, 1, 1);
  G_Elipse(OJO_DER_X + 13, 47, 6, 3, 1, 1);
}

/* Destellos para la fiesta */
static void Dibujar_Destello(int cx, int cy, int r)
{
  G_Linea(cx - r, cy, cx + r, cy, 1);
  G_Linea(cx, cy - r, cx, cy + r, 1);
  G_Pixel(cx - 1, cy - 1, 1); G_Pixel(cx + 1, cy - 1, 1);
  G_Pixel(cx - 1, cy + 1, 1); G_Pixel(cx + 1, cy + 1, 1);
}

/* Boca abierta sonriendo (media elipse con contorno y lengüita) */
static void Dibujar_Boca_Abierta(int rx, int ry)
{
  int y0 = BOCA_Y - 2;
  G_Elipse(BOCA_X, y0, rx, ry, 1, 0);
  G_Elipse(BOCA_X, y0, rx - 2, ry - 2, 0, 0);
  G_Rect(BOCA_X - rx - 1, y0 - ry - 1, 2 * rx + 3, ry + 1, 0);
  G_Rect(BOCA_X - rx, y0, 2 * rx + 1, 1, 1);
  G_Elipse(BOCA_X, y0 + ry - 3, rx / 2, 2, 1, 1);         /* lengüita */
}

/* Ícono de batería (0 a 4 rayitas), recortado sobre el pelo */
static void Dibujar_Bateria(int x, int y, uint8_t nivel)
{
  G_Rect(x - 1, y - 1, 18, 9, 0);
  G_Rect(x, y, 14, 7, 1);
  G_Rect(x + 1, y + 1, 12, 5, 0);
  G_Rect(x + 14, y + 2, 2, 3, 1);
  for (uint8_t i = 0; (i < nivel) && (i < 4); i++)
    G_Rect(x + 2 + i * 3, y + 2, 2, 3, 1);
}

/* Ícono WiFi: estado 1 = conectado, 2 = sin conexión */
static void Dibujar_WiFi(int x, int y, uint8_t estado)
{
  G_Rect(x - 1, y - 1, 15, 10, 0);
  G_Rect(x + 5, y + 6, 2, 2, 1);
  G_Arco(x + 6, y + 6, 3, 2, 0, 1);
  if (estado == 1)
  {
    G_Arco(x + 6, y + 6, 6, 5, 0, 1);
  }
  else
  {
    G_Linea(x + 9, y + 1, x + 13, y + 5, 1);               /* x pequeña */
    G_Linea(x + 13, y + 1, x + 9, y + 5, 1);
  }
}

/* Dibuja la cara completa en la memoria de imagen.
   mx, my: hacia dónde mira (-3..3)   t: tiempo en ms (animaciones)    */
static void Dibujar_Cara(Expresion e, int mx, int my, uint8_t parpadeo, uint32_t t)
{
  Dibujar_Pelo();
  Dibujar_Broche(104, 7);

  switch (e)
  {
    case CARA_FIESTA:
      Dibujar_Ojo_Feliz(OJO_IZQ_X, OJO_Y);
      Dibujar_Ojo_Feliz(OJO_DER_X, OJO_Y);
      Dibujar_Boca_Abierta(8, 7);
      if ((t / 300) & 1) { Dibujar_Destello(20, 22, 3); Dibujar_Destello(106, 30, 3); }
      else               { Dibujar_Destello(24, 30, 2); Dibujar_Destello(104, 21, 2); }
      break;

    case CARA_SORPRESA:
      Dibujar_Ojo(OJO_IZQ_X, OJO_Y + 1, 0, 0, 1, 0);
      Dibujar_Ojo(OJO_DER_X, OJO_Y + 1, 0, 0, 1, 1);
      G_Elipse(BOCA_X, BOCA_Y + 1, 4, 5, 1, 0);
      G_Elipse(BOCA_X, BOCA_Y + 1, 2, 3, 0, 0);
      if ((t / 250) & 1)
      {
        G_Rect(115, 22, 3, 9, 1);                         /* signo ! */
        G_Rect(115, 33, 3, 3, 1);
      }
      break;

    case CARA_TRISTE:
      Dibujar_Ojo(OJO_IZQ_X, OJO_Y + 1, 0, 2, 0, 0);
      Dibujar_Ojo(OJO_DER_X, OJO_Y + 1, 0, 2, 0, 1);
      Tapar_Triste(OJO_IZQ_X, OJO_Y + 1, 0);
      Tapar_Triste(OJO_DER_X, OJO_Y + 1, 1);
      G_Linea2(OJO_IZQ_X - 10, OJO_Y - 9, OJO_IZQ_X + 7, OJO_Y - 14, 1);   /* cejas */
      G_Linea2(OJO_DER_X - 7, OJO_Y - 14, OJO_DER_X + 10, OJO_Y - 9, 1);
      G_Arco(BOCA_X, BOCA_Y + 4, 6, 4, 0, 1);              /* boca ⌒       */
      {
        int gy = OJO_Y + 13 + (int)((t / 120) % 8);         /* lágrima      */
        G_Elipse(OJO_IZQ_X + 8, gy, 2, 3, 1, 0);
        G_Pixel(OJO_IZQ_X + 8, gy - 4, 1);
      }
      break;

    case CARA_DORMIDA:
      Dibujar_Ojo_Cerrado(OJO_IZQ_X, OJO_Y, 0);
      Dibujar_Ojo_Cerrado(OJO_DER_X, OJO_Y, 1);
      G_Elipse(BOCA_X, BOCA_Y + 1, 2, 2, 1, 0);
      {
        uint32_t f = (t / 400) % 4;                         /* z z Z subiendo */
        if (f >= 1) G_Texto(100, 26, "z", 1);
        if (f >= 2) G_Texto(106, 21, "z", 1);
        if (f >= 3) G_Texto(112, 15, "Z", 1);
      }
      break;

    case CARA_FELIZ:
    case CARA_NORMAL:
    default:
      if (parpadeo)
      {
        Dibujar_Ojo_Cerrado(OJO_IZQ_X, OJO_Y, 0);
        Dibujar_Ojo_Cerrado(OJO_DER_X, OJO_Y, 1);
      }
      else
      {
        Dibujar_Ojo(OJO_IZQ_X, OJO_Y, mx, my, 0, 0);
        Dibujar_Ojo(OJO_DER_X, OJO_Y, mx, my, 0, 1);
      }
      if (e == CARA_FELIZ) Dibujar_Boca_Abierta(6, 5);
      else                 G_Arco(BOCA_X, BOCA_Y - 2, 5, 3, 1, 1);
      break;
  }

  Dibujar_Mejillas();
}


/* Pasa toda la imagen a la pantalla (8 páginas de 8 filas) */
static void OLED_Enviar(void)
{
#if OLED_SH1106
  const uint8_t columna = 2;           /* El SH1106 tiene 132 columnas */
#else
  const uint8_t columna = 0;
#endif

  for (uint8_t pag = 0; pag < 8; pag++)
  {
    OLED_Comando((uint8_t)(0xB0 + pag));
    OLED_Comando((uint8_t)(0x00 | (columna & 0x0F)));
    OLED_Comando((uint8_t)(0x10 | (columna >> 4)));

    OLED_ALTO_PIN(OLED_DC);            /* DC = 1: datos */
    OLED_BAJO_PIN(OLED_CS);
    for (uint8_t x = 0; x < OLED_ANCHO; x++)
    {
      OLED_Byte(oled_buf[pag * OLED_ANCHO + x]);
    }
    OLED_ALTO_PIN(OLED_CS);
  }
}

/* Modo texto (para los modos de prueba 1, 5 y 7): dos renglones */
static uint8_t oled_modo_texto = 0;

static void Pantalla_Texto(uint8_t fila, const char *texto)
{
  if (!oled_modo_texto)
  {
    G_Limpiar();
    Dibujar_Pelo();                    /* Marco decorativo arriba */
    Dibujar_Broche(104, 7);
    oled_modo_texto = 1;
  }

  G_Rect(10, 24 + fila * 14, 108, 10, 0);
  G_Texto_Centrado(25 + fila * 14, texto, 1);
  OLED_Enviar();
}

/* Cara normal con un mensaje abajo (al encender, al iniciar Bluetooth) */
static void Pantalla_Saludo(const char *mensaje)
{
  G_Limpiar();
  Dibujar_Cara(CARA_NORMAL, 0, 0, 0, HAL_GetTick());
  G_Rect(0, 55, OLED_ANCHO, 9, 0);
  G_Texto_Centrado(56, mensaje, 1);
  OLED_Enviar();
  oled_modo_texto = 0;
}

/* =====================================================================
   ADC1 (configurado por registros, no necesita el módulo HAL ADC)
   ===================================================================== */

static void ADC_Iniciar(void)
{
  /* Reloj del ADC: 72 MHz / 6 = 12 MHz (máximo permitido: 14 MHz) */
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;
  __HAL_RCC_ADC1_CLK_ENABLE();

  ADC1->CR1   = 0;                              /* Modo independiente   */
  ADC1->SQR1  = 0;                              /* 1 conversión         */
  ADC1->SMPR2 = ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9; /* 239.5 ciclos (estable) */

  ADC1->CR2 = ADC_CR2_ADON;                     /* Encender el ADC      */
  HAL_Delay(2);

  /* Calibración interna */
  ADC1->CR2 |= ADC_CR2_RSTCAL;
  while (ADC1->CR2 & ADC_CR2_RSTCAL) { }
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 & ADC_CR2_CAL) { }

  /* Disparo por software (SWSTART) */
  ADC1->CR2 |= ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;
}

static uint16_t ADC_Leer(uint32_t canal)
{
  uint32_t t = HAL_GetTick();

  ADC1->SQR3 = canal;
  ADC1->SR   = 0;
  ADC1->CR2 |= ADC_CR2_SWSTART;

  while (!(ADC1->SR & ADC_SR_EOC))
  {
    if ((HAL_GetTick() - t) > 2) return 0;
  }

  return (uint16_t)(ADC1->DR & 0x0FFF);
}

/* Promedio de varias lecturas convertido a voltios en el pin */
static float ADC_Voltios(uint32_t canal)
{
  uint32_t suma = 0;

  for (uint32_t i = 0; i < MUESTRAS_ADC; i++)
  {
    suma += ADC_Leer(canal);
  }

  return ((float)suma / (float)MUESTRAS_ADC) * VDDA_V / 4095.0f;
}

/* =====================================================================
   TEXTO PARA LA PANTALLA (sin printf, para no necesitar soporte float)
   ===================================================================== */

static char *Agregar_Texto(char *p, const char *s)
{
  while (*s) *p++ = *s++;
  return p;
}

static char *Agregar_Numero(char *p, uint32_t n)
{
  char tmp[10];
  int  i = 0;

  do
  {
    tmp[i++] = (char)('0' + (n % 10));
    n /= 10;
  } while (n > 0);

  while (i > 0) *p++ = tmp[--i];
  return p;
}

/* valor viene multiplicado por 10 (1 decimal) o por 100 (2 decimales) */
static char *Agregar_Decimal(char *p, uint32_t valor, uint8_t decimales)
{
  uint32_t div  = (decimales == 2) ? 100 : 10;
  uint32_t frac = valor % div;

  p = Agregar_Numero(p, valor / div);
  *p++ = '.';
  if ((decimales == 2) && (frac < 10)) *p++ = '0';
  p = Agregar_Numero(p, frac);
  return p;
}

/* Distancia lineal recorrida: solo cuentan los lados (en los giros en
   su sitio el carro no avanza). Promedio de las dos ruedas.             */
static float Distancia_Recorrida_cm(void)
{
  float pulsos = ((float)acc_recta.real_A +
                  (float)acc_recta.real_B / RELACION_DERECHA) * 0.5f;

  if (en_recta)
  {
    pulsos += ((float)cuenta_A + (float)cuenta_B / RELACION_DERECHA) * 0.5f;
  }

  return pulsos * CM_POR_PULSO / FACTOR_DISTANCIA;
}

/* Línea 1: "B:7.42V T:25.3°C"
   Línea 2: "Dist: 123.4 cm"                                             */
/* Lee batería, temperatura y distancia y las deja en las variables
   globales (las usan la pantalla y el envío por WiFi).                  */
static void Leer_Sensores(void)
{
  float temp;

  bateria_v = ADC_Voltios(CANAL_BATERIA) * DIVISOR_BATERIA * FACTOR_BATERIA;

  temp = ADC_Voltios(CANAL_LM35) * 100.0f;     /* LM35: 10 mV por °C */
  if (temp < 0.0f) temp = 0.0f;
  temperatura_c = temp;

  distancia_cm = Distancia_Recorrida_cm();
}

/* Elige la expresión según lo que esté pasando y dibuja la carita.
   No muestra datos: solo íconos de batería y WiFi arriba.              */
static void Pantalla_Actualizar(void)
{
  static uint32_t t_ultimo_mov = 0;
  uint32_t   ahora = HAL_GetTick();
  Expresion  e;
  const char *mensaje = NULL;
  int        mx = 0;
  int        my = 0;
  uint8_t    parpadeo = 0;
  int        nivel;

  Leer_Sensores();                     /* Para la base de datos */

  if (movimiento != 'S') t_ultimo_mov = ahora;

  if (alerta_llantas)
  {
    e = CARA_TRISTE;
    if (alerta_llantas == 1)      mensaje = "Rueda izquierda!";
    else if (alerta_llantas == 2) mensaje = "Rueda derecha!";
    else                          mensaje = "Ambas ruedas!";
  }
  else if (hay_obstaculo)
  {
    e = CARA_SORPRESA;
    mensaje = "Cuidado!";
  }
  else if (mision_terminada)
  {
    e = CARA_FIESTA;
    mensaje = "Mision completa!";
  }
  else if (movimiento != 'S')
  {
    e = CARA_FELIZ;
    if (movimiento == 'L')      mx = -3;     /* Mira hacia donde gira */
    else if (movimiento == 'R') mx = 3;
    else if (movimiento == 'B') my = 2;
  }
  else if ((ahora - t_ultimo_mov) > TIEMPO_DORMIR_MS)
  {
    e = CARA_DORMIDA;
  }
  else
  {
    e = CARA_NORMAL;
    switch ((ahora / 1500) % 4)              /* Mira a los lados */
    {
      case 1:  mx = -2; break;
      case 3:  mx = 2;  break;
      default: mx = 0;  break;
    }
  }

  if ((e == CARA_NORMAL) || (e == CARA_FELIZ))
  {
    parpadeo = ((ahora % 3500) < 150) ? 1 : 0;
  }

  G_Limpiar();
  Dibujar_Cara(e, mx, my, parpadeo, ahora);

  /* Ícono de batería (0 a 4 rayitas) */
  nivel = (int)(((bateria_v - BATERIA_MIN_V) * 4.0f) / (BATERIA_MAX_V - BATERIA_MIN_V) + 0.5f);
  if (nivel < 0) nivel = 0;
  if (nivel > 4) nivel = 4;
  Dibujar_Bateria(4, 2, (uint8_t)nivel);

  /* Ícono WiFi (solo en los modos que lo usan) */
  if (wifi_en_uso) Dibujar_WiFi(20, 1, wifi_ok ? 1 : 2);

  if (mensaje != NULL)
  {
    G_Rect(0, 55, OLED_ANCHO, 9, 0);
    G_Texto_Centrado(56, mensaje, 1);
  }

  OLED_Enviar();
  oled_modo_texto = 0;
}

/* =====================================================================
   WIFI ESP8266 CON COMANDOS AT
   Funciona como una "máquina de estados" que NO bloquea: cada vez que
   se llama WiFi_Tarea() revisa si llegó la respuesta esperada y pasa
   al siguiente paso. Así el carro sigue moviéndose mientras envía.

   Pasos:  AT -> ATE0 -> CWMODE=1 -> CWJAP (conectar a la red)
           -> CIPMUX=0 -> LISTO
   Envío:  CIPSTART (abrir TCP) -> CIPSEND (largo) -> datos HTTP
           -> esperar respuesta de Node-RED -> cerrar -> LISTO
   ===================================================================== */

#if WIFI_ACTIVO

typedef enum
{
  W_AT = 0,
  W_ECO,
  W_MODO,
  W_RED,
  W_ESPERA_RED,
  W_MUX,
  W_LISTO,
  W_TCP,
  W_LARGO,
  W_DATOS,
  W_RESPUESTA,
  W_CIERRE
} EstadoWiFi;

#define CMD_RED   "AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_CLAVE "\"\r\n"
#define CMD_TCP   "AT+CIPSTART=\"TCP\",\"" SERVIDOR_IP "\"," SERVIDOR_PUERTO "\r\n"

static EstadoWiFi w_estado    = W_AT;
static uint32_t   w_t_estado  = 0;     /* Cuándo empezó el paso actual   */
static uint32_t   w_t_envio   = 0;     /* Cuándo fue el último envío     */
static uint8_t    w_http_ok   = 0;     /* Node-RED respondió 200         */
static uint8_t    w_intentos  = 0;
static char       w_resp[256];         /* Lo que ha respondido el ESP    */
static uint16_t   w_resp_len  = 0;
static char       w_peticion[512];     /* Petición HTTP completa         */
static uint16_t   w_pet_len   = 0;
static char       w_cmd[32];
static uint8_t    w_iniciado  = 0;     /* 1 cuando WiFi_Iniciar() ya corrió */

static void ESP_Enviar_N(const char *s, uint16_t n)
{
  for (uint16_t i = 0; i < n; i++)
  {
    while (!(USART2->SR & USART_SR_TXE)) { }
    USART2->DR = (uint8_t)s[i];
  }
}

static void ESP_Enviar(const char *s)
{
  while (*s)
  {
    while (!(USART2->SR & USART_SR_TXE)) { }
    USART2->DR = (uint8_t)*s++;
  }
}

/* Pasa lo que guardó la interrupción al texto de respuesta */
static void WiFi_Recibir(void)
{
  uint8_t dato;

  while (esp_cola != esp_cabeza)
  {
    dato = esp_buffer[esp_cola];
    esp_cola = (uint16_t)((esp_cola + 1) % ESP_BUFFER);

    if (dato == 0) continue;

    /* Si se llena, se descarta la mitad más vieja */
    if (w_resp_len >= sizeof(w_resp) - 1)
    {
      uint16_t mitad = (uint16_t)(sizeof(w_resp) / 2);
      for (uint16_t i = 0; i < w_resp_len - mitad; i++)
      {
        w_resp[i] = w_resp[i + mitad];
      }
      w_resp_len = (uint16_t)(w_resp_len - mitad);
    }

    w_resp[w_resp_len++] = (char)dato;
    w_resp[w_resp_len]   = '\0';

    if (strstr(w_resp, " 200 ") != NULL) w_http_ok = 1;
  }
}

static uint8_t Resp(const char *texto)
{
  return (strstr(w_resp, texto) != NULL) ? 1 : 0;
}

/* Cambia de paso: borra la respuesta y envía el comando (si hay) */
static void Pasar(EstadoWiFi nuevo, const char *comando)
{
  w_estado   = nuevo;
  w_t_estado = HAL_GetTick();
  w_resp_len = 0;
  w_resp[0]  = '\0';

  if (comando != NULL) ESP_Enviar(comando);
}

/* Arma el POST con los datos en JSON:
   {"modo":0,"bateria":7.42,"temperatura":25.3,"distancia":120.4,"obstaculo":0,
    "alerta":"ninguna","color":"rojo","estado":"buscar_pelota"}          */
static void Armar_Peticion(void)
{
  char  cuerpo[256];
  char *p;
  uint16_t largo_cuerpo;

  Leer_Sensores();

  p = cuerpo;
  p = Agregar_Texto(p, "{\"modo\":");
  p = Agregar_Numero(p, MODO);
  p = Agregar_Texto(p, ",\"bateria\":");
  p = Agregar_Decimal(p, (uint32_t)(bateria_v * 100.0f + 0.5f), 2);
  p = Agregar_Texto(p, ",\"temperatura\":");
  p = Agregar_Decimal(p, (uint32_t)(temperatura_c * 10.0f + 0.5f), 1);
  p = Agregar_Texto(p, ",\"distancia\":");
  p = Agregar_Decimal(p, (uint32_t)(distancia_cm * 10.0f + 0.5f), 1);
  p = Agregar_Texto(p, ",\"obstaculo\":");
  p = Agregar_Numero(p, hay_obstaculo ? 1 : 0);
  p = Agregar_Texto(p, ",\"alerta\":\"");
  p = Agregar_Texto(p, Texto_Alerta(alerta_llantas));
  p = Agregar_Texto(p, "\",\"color\":\"");
  p = Agregar_Texto(p, color_pelota);
  p = Agregar_Texto(p, "\",\"estado\":\"");
  p = Agregar_Texto(p, estado_carro);
  p = Agregar_Texto(p, "\"}");
  *p = '\0';
  largo_cuerpo = (uint16_t)(p - cuerpo);

  p = w_peticion;
  p = Agregar_Texto(p, "POST " SERVIDOR_RUTA " HTTP/1.1\r\n");
  p = Agregar_Texto(p, "Host: " SERVIDOR_IP ":" SERVIDOR_PUERTO "\r\n");
  p = Agregar_Texto(p, "Content-Type: application/json\r\n");
  p = Agregar_Texto(p, "Content-Length: ");
  p = Agregar_Numero(p, largo_cuerpo);
  p = Agregar_Texto(p, "\r\nConnection: close\r\n\r\n");
  p = Agregar_Texto(p, cuerpo);
  *p = '\0';
  w_pet_len = (uint16_t)(p - w_peticion);

  p = w_cmd;
  p = Agregar_Texto(p, "AT+CIPSEND=");
  p = Agregar_Numero(p, w_pet_len);
  p = Agregar_Texto(p, "\r\n");
  *p = '\0';
}

static void WiFi_Iniciar(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_USART2_CLK_ENABLE();

  /* PA2 = TX (función alterna), PA3 = RX (entrada) */
  g.Pin   = ESP_TX_PIN;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ESP_PORT, &g);

  g.Pin  = ESP_RX_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESP_PORT, &g);

  /* 8N1. USART2 usa el reloj APB1 (36 MHz) */
  USART2->CR1 = 0;
  USART2->CR2 = 0;
  USART2->CR3 = 0;
  USART2->BRR = (HAL_RCC_GetPCLK1Freq() + (ESP_BAUDIOS / 2)) / ESP_BAUDIOS;

  /* UART, transmisor, receptor e INTERRUPCIÓN de recepción */
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  wifi_texto = "WiFi: buscar ESP";
  w_t_envio  = HAL_GetTick();
  w_iniciado = 1;
  wifi_en_uso = 1;
  Pasar(W_AT, "AT\r\n");
}

/* Llamar seguido (cada pocos ms). Nunca se queda esperando. */
static void WiFi_Tarea(void)
{
  uint32_t ahora = HAL_GetTick();
  uint32_t t;

  if (!w_iniciado) return;            /* Modos sin WiFi (1, 2, 5) */

  WiFi_Recibir();
  t = ahora - w_t_estado;

  switch (w_estado)
  {
    /* ---------------- Configuración inicial ---------------- */
    case W_AT:
      if (Resp("OK"))
      {
        w_intentos = 0;
        Pasar(W_ECO, "ATE0\r\n");             /* Sin eco de comandos */
      }
      else if (t > 1000)
      {
        if (++w_intentos >= 3) wifi_texto = "ESP no responde";
        Pasar(W_AT, "AT\r\n");
      }
      break;

    case W_ECO:
      if (Resp("OK"))     Pasar(W_MODO, "AT+CWMODE=1\r\n");   /* Modo estación */
      else if (t > 2000)  Pasar(W_AT, "AT\r\n");
      break;

    case W_MODO:
      if (Resp("OK"))
      {
        wifi_texto = "WiFi: conectando";
        Pasar(W_RED, CMD_RED);
      }
      else if (t > 2000)  Pasar(W_AT, "AT\r\n");
      break;

    case W_RED:
      if (Resp("OK"))
      {
        Pasar(W_MUX, "AT+CIPMUX=0\r\n");      /* Una sola conexión */
      }
      else if (Resp("FAIL") || Resp("ERROR") || (t > 20000))
      {
        wifi_texto = "Red: error clave";
        Pasar(W_ESPERA_RED, NULL);
      }
      break;

    case W_ESPERA_RED:                        /* Reintentar en 5 s */
      if (t > 5000)
      {
        wifi_texto = "WiFi: conectando";
        Pasar(W_RED, CMD_RED);
      }
      break;

    case W_MUX:
      if (Resp("OK"))
      {
        wifi_texto = "WiFi: conectado";
        Pasar(W_LISTO, NULL);
      }
      else if (t > 2000)  Pasar(W_AT, "AT\r\n");
      break;

    /* ---------------- Envío periódico ---------------- */
    case W_LISTO:
      if (Resp("WIFI DISCONNECT"))
      {
        wifi_texto = "WiFi: conectando";
        Pasar(W_RED, CMD_RED);
      }
      else if ((ahora - w_t_envio) >= PERIODO_ENVIO_MS)
      {
        w_t_envio = ahora;
        w_http_ok = 0;
        Armar_Peticion();
        Pasar(W_TCP, CMD_TCP);                /* Abrir conexión con Node-RED */
      }
      else if (w_resp_len > 100)
      {
        w_resp_len = 0;                       /* Limpiar mensajes viejos */
        w_resp[0]  = '\0';
      }
      break;

    case W_TCP:
      if (Resp("CONNECT") || Resp("ALREADY"))
      {
        Pasar(W_LARGO, w_cmd);                /* AT+CIPSEND=largo */
      }
      else if (Resp("no ip") || Resp("WIFI DISCONNECT"))
      {
        envios_error++;
        wifi_ok = 0;
        wifi_texto = "WiFi: conectando";
        Pasar(W_RED, CMD_RED);
      }
      else if (Resp("ERROR") || Resp("CLOSED") || (t > 5000))
      {
        envios_error++;
        wifi_ok = 0;
        wifi_texto = "Sin Node-RED";          /* IP o puerto mal, o firewall */
        Pasar(W_LISTO, NULL);
      }
      break;

    case W_LARGO:
      if (Resp(">"))
      {
        Pasar(W_DATOS, NULL);
        ESP_Enviar_N(w_peticion, w_pet_len);  /* La petición HTTP */
      }
      else if (Resp("ERROR") || (t > 3000))
      {
        envios_error++;
        wifi_ok = 0;
        Pasar(W_CIERRE, "AT+CIPCLOSE\r\n");
      }
      break;

    case W_DATOS:
      if (Resp("SEND OK"))
      {
        Pasar(W_RESPUESTA, NULL);
      }
      else if (Resp("ERROR") || Resp("FAIL") || (t > 5000))
      {
        envios_error++;
        wifi_ok = 0;
        Pasar(W_CIERRE, "AT+CIPCLOSE\r\n");
      }
      break;

    case W_RESPUESTA:                         /* Esperar a Node-RED */
      if (w_http_ok && Resp("CLOSED"))
      {
        envios_ok++;
        wifi_ok = 1;
        wifi_texto = "Envio OK";
        Pasar(W_LISTO, NULL);
      }
      else if (Resp("CLOSED") || (t > 4000))
      {
        if (w_http_ok)
        {
          envios_ok++;
          wifi_ok = 1;
          wifi_texto = "Envio OK";
        }
        else
        {
          envios_error++;
          wifi_ok = 0;
          wifi_texto = "Node-RED no 200";
        }
        Pasar(W_CIERRE, "AT+CIPCLOSE\r\n");
      }
      break;

    case W_CIERRE:
      if (Resp("OK") || Resp("ERROR") || (t > 2000))
      {
        Pasar(W_LISTO, NULL);
      }
      break;

    default:
      Pasar(W_AT, "AT\r\n");
      break;
  }
}

#else  /* WIFI_ACTIVO == 0 */

static void WiFi_Iniciar(void) { }
static void WiFi_Tarea(void)   { }

#endif /* WIFI_ACTIVO */

/* =====================================================================
   SEÑALES DE ERROR (LED + carita triste en la OLED)
   1 parpadeo  = rueda IZQUIERDA
   2 parpadeos = rueda DERECHA
   3 parpadeos = las dos
   ===================================================================== */

static const char *Texto_Alerta(uint8_t n)
{
  switch (n)
  {
    case 1:  return "rueda izquierda";
    case 2:  return "rueda derecha";
    case 3:  return "ambas ruedas";
    default: return "ninguna";
  }
}

/* Espera "ms" milisegundos sin dejar de atender el WiFi */
static void Esperar_ms(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();

  uint32_t t_pant = 0;

  while ((HAL_GetTick() - t0) < ms)
  {
    if ((HAL_GetTick() - t_pant) >= PERIODO_PANTALLA_MS)
    {
      t_pant = HAL_GetTick();
      Pantalla_Actualizar();           /* Carita triste animada */
    }
    WiFi_Tarea();
    HAL_Delay(5);
  }
}

static void Senal_Error(uint8_t n)
{
  codigo_error   = n;
  alerta_llantas = n;          /* Se envía la alerta a la base de datos */

  movimiento     = 'S';

  /* El carro queda detenido, pero sigue enviando la alerta por WiFi */
  while (1)
  {
    for (uint8_t i = 0; i < n; i++)
    {
      LED_ON();
      Esperar_ms(250);
      LED_OFF();
      Esperar_ms(250);
    }
    Esperar_ms(1500);
  }
}

static void Falla(uint8_t n)
{
  tramo_activo = 0;
  MotorA_Velocidad(0);
  MotorB_Velocidad(0);
  MotorA_Freno();
  MotorB_Freno();
  HAL_Delay(300);
  MotorA_Libre();
  MotorB_Libre();
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
  Senal_Error(n);
}

/* =====================================================================
   MODO 1: PRUEBA DE HARDWARE
   ===================================================================== */

#if (MODO == 1)

static uint32_t Probar_Motor(uint8_t motor, Sentido s, uint32_t ms)
{
  uint32_t pulsos;

  __disable_irq();
  cuenta_A = 0;
  cuenta_B = 0;
  tramo_activo = 0;
  __enable_irq();

  led_encoder = motor;

  if (motor == 1)
  {
    MotorA_Velocidad(VEL_PRUEBA);
    MotorA_Direccion(s);
  }
  else
  {
    MotorB_Velocidad(VEL_PRUEBA);
    MotorB_Direccion(s);
  }

  HAL_Delay(ms);

  MotorA_Freno();
  MotorB_Freno();
  HAL_Delay(300);
  MotorA_Libre();
  MotorB_Libre();
  MotorA_Velocidad(0);
  MotorB_Velocidad(0);

  led_encoder = 0;
  LED_OFF();

  pulsos = (motor == 1) ? cuenta_A : cuenta_B;
  return pulsos;
}

static void Prueba_Hardware(void)
{
  uint8_t falla = 0;

  Pantalla_Texto(0, "MODO PRUEBA");
  Pantalla_Texto(1, "Motores y encoders");

  LED_ON();
  HAL_Delay(ESPERA_INICIAL_MS);
  LED_OFF();

  prueba_izq_adelante = Probar_Motor(1, ADELANTE, 1500);
  HAL_Delay(700);
  prueba_izq_atras    = Probar_Motor(1, ATRAS,    1500);
  HAL_Delay(1500);
  prueba_der_adelante = Probar_Motor(2, ADELANTE, 1500);
  HAL_Delay(700);
  prueba_der_atras    = Probar_Motor(2, ATRAS,    1500);

  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);

  if ((prueba_izq_adelante < PULSOS_MINIMOS_PRUEBA) ||
      (prueba_izq_atras    < PULSOS_MINIMOS_PRUEBA))
  {
    falla |= 1;
  }

  if ((prueba_der_adelante < PULSOS_MINIMOS_PRUEBA) ||
      (prueba_der_atras    < PULSOS_MINIMOS_PRUEBA))
  {
    falla |= 2;
  }

  if (falla)
  {
    Senal_Error(falla);
  }

  mision_terminada = 1;
  Pantalla_Actualizar();               /* Carita de fiesta */

  while (1)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    for (uint8_t i = 0; i < 10; i++)
    {
      Pantalla_Actualizar();
      HAL_Delay(100);
    }
  }
}

#endif /* MODO == 1 */

/* =====================================================================
   MODO 6: CONTROL POR BLUETOOTH (HC-05 en USART1)
   La INTERRUPCIÓN de la UART (USART1_IRQHandler, al final del archivo)
   guarda cada byte que llega en bt_buffer. El programa principal los
   va leyendo con BT_Leer() y mueve los motores.
   ===================================================================== */

#if (MODO == 6)

static char    pedido     = 'S';    /* Último movimiento pedido por el celular */
static char    mov_actual = 'S';    /* Movimiento aplicado a los motores       */
static int32_t vel_bt     = VEL_BT_INICIAL;

static void BT_Iniciar(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_USART1_CLK_ENABLE();

  /* PA9 = TX (función alterna), PA10 = RX (entrada) */
  g.Pin   = BT_TX_PIN;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BT_PORT, &g);

  g.Pin  = BT_RX_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT_PORT, &g);

  /* 8 bits, sin paridad, 1 bit de parada. USART1 usa el reloj APB2 (72 MHz) */
  USART1->CR1 = 0;
  USART1->CR2 = 0;
  USART1->CR3 = 0;
  USART1->BRR = (HAL_RCC_GetPCLK2Freq() + (BT_BAUDIOS / 2)) / BT_BAUDIOS;

  /* Activar UART, transmisor, receptor e INTERRUPCIÓN de recepción (RXNE) */
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* Devuelve el siguiente byte recibido, o -1 si no hay ninguno */
static int BT_Leer(void)
{
  uint8_t dato;

  if (bt_cola == bt_cabeza) return -1;

  dato = bt_buffer[bt_cola];
  bt_cola = (uint8_t)((bt_cola + 1) % BT_BUFFER);
  return dato;
}

/* Envía un texto al celular (aparece en la terminal de la app) */
static void BT_Enviar_Texto(const char *s)
{
  while (*s)
  {
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = (uint8_t)*s++;
  }
}

static uint8_t Es_Adelante(char m)
{
  return (m == 'F') || (m == 'Q') || (m == 'E');
}

/* Suma a la distancia total lo recorrido con el movimiento anterior */
static void BT_Guardar_Distancia(void)
{
  uint32_t a;
  uint32_t b;

  __disable_irq();
  a = cuenta_A;
  b = cuenta_B;
  cuenta_A = 0;
  cuenta_B = 0;
  __enable_irq();

  if (en_recta)
  {
    acc_recta.real_A += a;
    acc_recta.real_B += b;
  }
}

/* Pone los motores según el movimiento. En los movimientos que avanzan
   (no en los giros en su sitio) se cuenta la distancia.                 */
static void BT_Aplicar(char m)
{
  int32_t v     = vel_bt;
  int32_t lento = vel_bt / 2;        /* Rueda interior en las diagonales */

  BT_Guardar_Distancia();
  en_recta = 0;

  switch (m)
  {
    case 'F':                                      /* Adelante */
      MotorA_Velocidad(v);     MotorB_Velocidad(v);
      MotorA_Direccion(ADELANTE); MotorB_Direccion(ADELANTE);
      en_recta = 1;
      break;

    case 'B':                                      /* Atrás */
      MotorA_Velocidad(v);     MotorB_Velocidad(v);
      MotorA_Direccion(ATRAS);    MotorB_Direccion(ATRAS);
      en_recta = 1;
      break;

    case 'L':                                      /* Gira izquierda en su sitio */
      MotorA_Velocidad(v);     MotorB_Velocidad(v);
      MotorA_Direccion(ATRAS);    MotorB_Direccion(ADELANTE);
      break;

    case 'R':                                      /* Gira derecha en su sitio */
      MotorA_Velocidad(v);     MotorB_Velocidad(v);
      MotorA_Direccion(ADELANTE); MotorB_Direccion(ATRAS);
      break;

    case 'Q':                                      /* Adelante-izquierda */
      MotorA_Velocidad(lento); MotorB_Velocidad(v);
      MotorA_Direccion(ADELANTE); MotorB_Direccion(ADELANTE);
      en_recta = 1;
      break;

    case 'E':                                      /* Adelante-derecha */
      MotorA_Velocidad(v);     MotorB_Velocidad(lento);
      MotorA_Direccion(ADELANTE); MotorB_Direccion(ADELANTE);
      en_recta = 1;
      break;

    case 'Z':                                      /* Atrás-izquierda */
      MotorA_Velocidad(lento); MotorB_Velocidad(v);
      MotorA_Direccion(ATRAS);    MotorB_Direccion(ATRAS);
      en_recta = 1;
      break;

    case 'C':                                      /* Atrás-derecha */
      MotorA_Velocidad(v);     MotorB_Velocidad(lento);
      MotorA_Direccion(ATRAS);    MotorB_Direccion(ATRAS);
      en_recta = 1;
      break;

    default:                                       /* 'S': parar */
      m = 'S';
      MotorA_Freno();
      MotorB_Freno();
      break;
  }

  mov_actual = m;

  /* Para la carita: hacia dónde va */
  switch (m)
  {
    case 'F':                     movimiento = 'F'; break;
    case 'B':                     movimiento = 'B'; break;
    case 'L': case 'Q': case 'Z': movimiento = 'L'; break;
    case 'R': case 'E': case 'C': movimiento = 'R'; break;
    default:                      movimiento = 'S'; break;
  }
}

/* Interpreta un byte recibido. Devuelve 1 si cambió la velocidad. */
static uint8_t BT_Procesar(int c)
{
  switch (c)
  {
    case 'F': case 'B': case 'L': case 'R':
    case 'Q': case 'E': case 'Z': case 'C':
    case 'S':
      pedido = (char)c;
      break;

    case 'X':                                      /* Más velocidad */
      vel_bt += PASO_VEL_BT;
      if (vel_bt > 100) vel_bt = 100;
      return 1;

    case 'Y':                                      /* Menos velocidad */
      vel_bt -= PASO_VEL_BT;
      if (vel_bt < VEL_MINIMA) vel_bt = VEL_MINIMA;
      return 1;

    default:            /* Luces, servos (J/K con números), T, D: se ignoran */
      break;
  }

  return 0;
}

static void Modo_Bluetooth(void)
{
  uint32_t ahora;
  uint32_t t_medicion = 0;
  uint32_t t_pantalla = 0;
  uint32_t t_comando;
  uint32_t d;
  uint8_t  obstaculo = 0;
  uint8_t  forzar    = 1;
  int      c;
  char     efectivo;
  uint8_t  revisar_A;
  uint8_t  revisar_B;
  uint32_t prev_A = 0;
  uint32_t prev_B = 0;
  uint32_t t_mov_A;
  uint32_t t_mov_B;

  BT_Iniciar();
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
  BT_Aplicar('S');

  Pantalla_Saludo("Bluetooth listo!");
  BT_Enviar_Texto("Carro listo\r\n");
  HAL_Delay(1500);

  t_comando = HAL_GetTick();
  t_mov_A   = t_comando;
  t_mov_B   = t_comando;

  while (1)
  {
    ahora = HAL_GetTick();

    /* 1) Leer todo lo que guardó la interrupción */
    while ((c = BT_Leer()) >= 0)
    {
      t_comando = ahora;
      if (BT_Procesar(c)) forzar = 1;     /* Nueva velocidad: reaplicar */
    }

#if (TIMEOUT_BT_MS > 0)
    if ((ahora - t_comando) > TIMEOUT_BT_MS) pedido = 'S';
#else
    (void)t_comando;
#endif

    /* 2) Sensor ultrasónico: bloquea solo el avance hacia adelante */
    if ((ahora - t_medicion) >= PERIODO_MEDICION_MS)
    {
      t_medicion = ahora;
      d = Medir_Distancia_cm();

      if (!obstaculo && (d <= DISTANCIA_LIMITE))
      {
        obstaculo = 1;
        BT_Enviar_Texto("Obstaculo!\r\n");
      }
      else if (obstaculo && (d > DISTANCIA_REANUDAR))
      {
        obstaculo = 0;
      }

      hay_obstaculo = obstaculo;
    }

    /* 3) Mover los motores */
    efectivo = (obstaculo && Es_Adelante(pedido)) ? 'S' : pedido;

    if (forzar || (efectivo != mov_actual))
    {
      BT_Aplicar(efectivo);
      forzar = 0;
    }

    if (mov_actual == 'S') LED_OFF(); else LED_ON();

    /* 4) Alerta de llantas: si una rueda debería girar con fuerza y su
          encoder no da pulsos en TIMEOUT_ATASCO_MS, se marca la alerta.
          En las diagonales la rueda interior va lenta y no se revisa.  */
    revisar_A = 0;
    revisar_B = 0;
    switch (mov_actual)
    {
      case 'F': case 'B': case 'L': case 'R': revisar_A = 1; revisar_B = 1; break;
      case 'Q': case 'Z':                     revisar_B = 1;                break;
      case 'E': case 'C':                     revisar_A = 1;                break;
      default:                                                              break;
    }

    if ((cuenta_A != prev_A) || !revisar_A)
    {
      if (cuenta_A != prev_A) alerta_llantas &= (uint8_t)~1U;   /* Volvió a girar */
      prev_A  = cuenta_A;
      t_mov_A = ahora;
    }
    if ((cuenta_B != prev_B) || !revisar_B)
    {
      if (cuenta_B != prev_B) alerta_llantas &= (uint8_t)~2U;
      prev_B  = cuenta_B;
      t_mov_B = ahora;
    }

    if ((ahora - t_mov_A) > TIMEOUT_ATASCO_MS) alerta_llantas |= 1;
    if ((ahora - t_mov_B) > TIMEOUT_ATASCO_MS) alerta_llantas |= 2;

    /* 5) Pantalla */
    if ((ahora - t_pantalla) >= PERIODO_PANTALLA_MS)
    {
      t_pantalla = ahora;

      Pantalla_Actualizar();           /* La carita reacciona sola */
    }

    WiFi_Tarea();
    HAL_Delay(5);
  }
}

#endif /* MODO == 6 */

/* =====================================================================
   LÓGICA DE MOVIMIENTO
   ===================================================================== */

static void Iniciar_Acumulado(Acumulado *acc, float exceso_inicial)
{
  acc->ideal_A  = 0.0f;
  acc->ideal_B  = 0.0f;
  acc->real_A   = 0;
  acc->real_B   = 0;
  acc->exceso_A = exceso_inicial;
  acc->exceso_B = exceso_inicial;
}

static uint32_t Calcular_Objetivo(float ideal, uint32_t real, float exceso)
{
  float falta = ideal - (float)real - exceso;

  if (falta < 1.0f) falta = 1.0f;

  return (uint32_t)(falta + 0.5f);
}

static void Arrancar_Motores(Sentido sA, Sentido sB)
{
  __disable_irq();
  if (!fin_A) MotorA_Direccion(sA);
  if (!fin_B) MotorB_Direccion(sB);
  __enable_irq();
}

static void Ajustar_Velocidades(int32_t vel, uint32_t objA, uint32_t objB)
{
  int32_t a = (int32_t)cuenta_A;
  int32_t b = (int32_t)cuenta_B;
  /* > 0: la izquierda va adelantada (con la derecha escalada) */
  int32_t error = (int32_t)((float)a * RELACION_DERECHA + 0.5f) - b;
  int32_t corr;
  int32_t vA, vB;

  uint32_t restA = (objA > (uint32_t)a) ? (objA - (uint32_t)a) : 0;
  uint32_t restB = (objB > (uint32_t)b) ? (objB - (uint32_t)b) : 0;
  uint32_t restante = (restA < restB) ? restA : restB;

  int32_t base = vel;
  if ((restante <= PULSOS_DESACELERAR) && (VEL_LENTA < vel))
  {
    base = VEL_LENTA;
  }

  corr = KP_SINCRONIA * error;
  if (corr >  CORRECCION_MAX) corr =  CORRECCION_MAX;
  if (corr < -CORRECCION_MAX) corr = -CORRECCION_MAX;

  vA = base - corr;
  vB = base + corr;

  if (vA < VEL_MINIMA) vA = VEL_MINIMA;
  if (vB < VEL_MINIMA) vB = VEL_MINIMA;

  MotorA_Velocidad(vA);
  MotorB_Velocidad(vB);
}

static void Pausar_Por_Obstaculo(Sentido sA, Sentido sB)
{
  uint32_t d;

  MotorA_Freno();
  MotorB_Freno();
  hay_obstaculo = 1;

  do
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    Pantalla_Actualizar();
    for (uint32_t i = 0; i < PERIODO_MEDICION_MS; i += 5)
    {
      WiFi_Tarea();
      HAL_Delay(5);
    }
    d = Medir_Distancia_cm();
  } while (d <= DISTANCIA_REANUDAR);

  hay_obstaculo = 0;
  LED_ON();
  Arrancar_Motores(sA, sB);
}

static void Ejecutar_Tramo(Sentido sA, Sentido sB, float pulsos,
                           Acumulado *acc, int32_t vel,
                           uint8_t vigilar_obstaculo)
{
  uint32_t t_medicion = 0;
  uint32_t t_pantalla = 0;
  uint32_t prev_A = 0;
  uint32_t prev_B = 0;
  uint32_t t_mov_A;
  uint32_t t_mov_B;
  uint32_t ahora;
  uint32_t objA;
  uint32_t objB;
  uint32_t final_A;
  uint32_t final_B;
  float    medido;

  acc->ideal_A += pulsos;
  acc->ideal_B += pulsos * RELACION_DERECHA;
  objA = Calcular_Objetivo(acc->ideal_A, acc->real_A, acc->exceso_A);
  objB = Calcular_Objetivo(acc->ideal_B, acc->real_B, acc->exceso_B);

  __disable_irq();
  cuenta_A = 0;
  cuenta_B = 0;
  objetivo_A = objA;
  objetivo_B = objB;
  fin_A = 0;
  fin_B = 0;
  tramo_activo = 1;
  en_recta = (acc == &acc_recta) ? 1 : 0;
  if (sA == sB)            movimiento = (sA == ADELANTE) ? 'F' : 'B';
  else if (sA == ATRAS)    movimiento = 'L';
  else                     movimiento = 'R';
  __enable_irq();

  MotorA_Velocidad(vel);
  MotorB_Velocidad(vel);
  Arrancar_Motores(sA, sB);

  t_mov_A = HAL_GetTick();
  t_mov_B = t_mov_A;

  while (!(fin_A && fin_B))
  {
    ahora = HAL_GetTick();

    if (vigilar_obstaculo &&
        ((ahora - t_medicion) >= PERIODO_MEDICION_MS))
    {
      t_medicion = ahora;

      if (Medir_Distancia_cm() <= DISTANCIA_LIMITE)
      {
        Pausar_Por_Obstaculo(sA, sB);
        ahora      = HAL_GetTick();
        t_medicion = ahora;
        t_mov_A    = ahora;
        t_mov_B    = ahora;
      }
    }

    if ((ahora - t_pantalla) >= PERIODO_PANTALLA_MS)
    {
      t_pantalla = ahora;
      Pantalla_Actualizar();
    }

    if (cuenta_A != prev_A) { prev_A = cuenta_A; t_mov_A = ahora; }
    if (cuenta_B != prev_B) { prev_B = cuenta_B; t_mov_B = ahora; }

    if (!fin_A && ((ahora - t_mov_A) > TIMEOUT_ATASCO_MS)) Falla(1);
    if (!fin_B && ((ahora - t_mov_B) > TIMEOUT_ATASCO_MS)) Falla(2);

    Ajustar_Velocidades(vel, objA, objB);
    WiFi_Tarea();
    HAL_Delay(5);
  }

  tramo_activo = 0;

  HAL_Delay(PAUSA_ENTRE_TRAMOS_MS);

  __disable_irq();
  final_A = cuenta_A;
  final_B = cuenta_B;
  __enable_irq();

  MotorA_Libre();
  MotorB_Libre();

  acc->real_A += final_A;
  acc->real_B += final_B;
  en_recta = 0;
  movimiento = 'S';

  medido = (float)final_A - (float)objA;
  if (medido < 0.0f) medido = 0.0f;
  acc->exceso_A = 0.5f * acc->exceso_A + 0.5f * medido;

  medido = (float)final_B - (float)objB;
  if (medido < 0.0f) medido = 0.0f;
  acc->exceso_B = 0.5f * acc->exceso_B + 0.5f * medido;

  Pantalla_Actualizar();
}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  float pulsos_lado;
  float pulsos_giro;
  float angulo_exterior;
  float arco_giro_cm;
  uint32_t t_inicio;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  MotorA_Libre();
  MotorB_Libre();
  MotorA_Velocidad(0);
  MotorB_Velocidad(0);

  ADC_Iniciar();
  OLED_Iniciar();
#if (MODO == 0) || (MODO == 3) || (MODO == 4) || (MODO == 6) || (MODO == 7)
  WiFi_Iniciar();
#endif
  Pantalla_Saludo("Hola!");
  HAL_Delay(1200);

  Iniciar_Acumulado(&acc_recta, EXCESO_INICIAL_RECTA);
  Iniciar_Acumulado(&acc_giro,  EXCESO_INICIAL_GIRO);

#if (MODO == 4)

  /* ---------------- SOLO PANTALLA (sin motores) -------------- */
  while (1)
  {
    Pantalla_Actualizar();
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    for (uint32_t i = 0; i < PERIODO_PANTALLA_MS; i += 5)
    {
      WiFi_Tarea();
      HAL_Delay(5);
    }
  }

#endif

#if (MODO == 7)

  /* -------------- PRUEBA WIFI (sin motores) ------------------ */
  /* Línea 1: estado del ESP8266. Línea 2: envíos buenos y malos. */
  {
    char     linea[32];
    char    *p;
    uint32_t t_pant = 0;

    while (1)
    {
      WiFi_Tarea();

      if ((HAL_GetTick() - t_pant) >= PERIODO_PANTALLA_MS)
      {
        t_pant = HAL_GetTick();
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

        Pantalla_Texto(0, wifi_texto);

        p = linea;
        p = Agregar_Texto(p, "OK:");
        p = Agregar_Numero(p, envios_ok);
        p = Agregar_Texto(p, " Err:");
        p = Agregar_Numero(p, envios_error);
        *p = '\0';
        Pantalla_Texto(1, linea);
      }

      HAL_Delay(5);
    }
  }

#endif

#if (MODO == 5)

  /* ------ VER ENCODERS: VUELTAS_PRUEBA_ENC vueltas por rueda ---- */
  /* Con el carro LEVANTADO: cada motor gira a VEL_RECTA (la misma
     velocidad del recorrido) hasta que SU encoder cuente las vueltas.
     Si la rueda da MÁS vueltas de las indicadas, se pierden pulsos. */
  {
    char     linea[32];
    char    *p;
    uint32_t t0;

    HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);

    while (1)
    {
      for (uint8_t motor = 1; motor <= 2; motor++)
      {
        Pantalla_Texto(0, (motor == 1) ? "Girando: IZQ" : "Girando: DER");
        Pantalla_Texto(1, "Contar vueltas");
        HAL_Delay(1500);

        /* Preparar: solo la rueda elegida tiene objetivo */
        __disable_irq();
        cuenta_A = 0;
        cuenta_B = 0;
        objetivo_A = PULSOS_POR_VUELTA * VUELTAS_PRUEBA_ENC;
        objetivo_B = PULSOS_POR_VUELTA * VUELTAS_PRUEBA_ENC;
        fin_A = (motor == 1) ? 0 : 1;
        fin_B = (motor == 2) ? 0 : 1;
        tramo_activo = 1;
        __enable_irq();

        if (motor == 1)
        {
          MotorA_Velocidad(VEL_RECTA);
          MotorA_Direccion(ADELANTE);
        }
        else
        {
          MotorB_Velocidad(VEL_RECTA);
          MotorB_Direccion(ADELANTE);
        }

        /* Esperar a que el encoder de ESA rueda frene el motor
           (máximo 10 s por si el encoder no cuenta)             */
        t0 = HAL_GetTick();
        while (!(fin_A && fin_B) && ((HAL_GetTick() - t0) < 10000))
        {
          HAL_Delay(5);
        }

        tramo_activo = 0;
        MotorA_Freno();
        MotorB_Freno();
        HAL_Delay(400);
        MotorA_Libre();
        MotorB_Libre();
        MotorA_Velocidad(0);
        MotorB_Velocidad(0);

        /* Mostrar lo que contó cada encoder */
        p = linea;
        p = Agregar_Texto(p, "Izq (PB6): ");
        p = Agregar_Numero(p, cuenta_A);
        *p = '\0';
        Pantalla_Texto(0, linea);

        p = linea;
        p = Agregar_Texto(p, "Der (PB7): ");
        p = Agregar_Numero(p, cuenta_B);
        *p = '\0';
        Pantalla_Texto(1, linea);

        HAL_Delay(4000);    /* Tiempo para leer la pantalla */
      }
    }
  }

#endif

#if (MODO == 6)
  Modo_Bluetooth();       /* No regresa */
#endif

  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);

#if (MODO == 1)
  Prueba_Hardware();      /* No regresa */
#endif

  pulsos_lado = (LONGITUD_LADO_CM * FACTOR_DISTANCIA) / CM_POR_PULSO;

  angulo_exterior = 360.0f / (float)NUM_LADOS;
  arco_giro_cm    = (angulo_exterior / 360.0f) * PI_F * ANCHO_EJE_CM * FACTOR_GIRO;
  pulsos_giro     = arco_giro_cm / CM_POR_PULSO;
  (void)pulsos_lado;      /* Evita avisos en los modos que no los usan */
  (void)pulsos_giro;

  /* Espera inicial mostrando los valores */
  LED_ON();
  t_inicio = HAL_GetTick();
  while ((HAL_GetTick() - t_inicio) < ESPERA_INICIAL_MS)
  {
    Pantalla_Actualizar();
    for (uint32_t i = 0; i < PERIODO_PANTALLA_MS; i += 5)
    {
      WiFi_Tarea();
      HAL_Delay(5);
    }
  }

#if (MODO == 0)

  for (uint32_t vuelta = 0; vuelta < NUM_VUELTAS; vuelta++)
  {
    for (uint32_t lado = 0; lado < NUM_LADOS; lado++)
    {
      Ejecutar_Tramo(ADELANTE, ADELANTE, pulsos_lado, &acc_recta, VEL_RECTA, 1);
      Ejecutar_Tramo(ATRAS,    ADELANTE, pulsos_giro, &acc_giro,  VEL_GIRO,  0);
    }
  }

#elif (MODO == 2)

  for (uint32_t i = 0; i < NUM_LADOS; i++)
  {
    Ejecutar_Tramo(ATRAS, ADELANTE, pulsos_giro, &acc_giro, VEL_GIRO, 0);
  }

#elif (MODO == 3)

  Ejecutar_Tramo(ADELANTE, ADELANTE,
                 (DIST_CALIBRACION_CM * FACTOR_DISTANCIA) / CM_POR_PULSO,
                 &acc_recta, VEL_RECTA, 1);

#endif

  MotorA_Libre();
  MotorB_Libre();
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);

  movimiento       = 'S';
  mision_terminada = 1;                /* Carita de fiesta */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* Recorrido terminado: LED parpadea, la pantalla sigue actualizando
       y se siguen enviando los datos por WiFi                          */
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    for (uint32_t i = 0; i < 500; i += PERIODO_PANTALLA_MS)
    {
      Pantalla_Actualizar();
      for (uint32_t k = 0; k < PERIODO_PANTALLA_MS; k += 5)
      {
        WiFi_Tarea();
        HAL_Delay(5);
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration (HSE 8 MHz x 9 = 72 MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief TIM2: contador de 1 us (72 MHz / 72)
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_PERIODO - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief TIM3: PWM 20 kHz en CH1 (PA6 -> PWMA) y CH2 (PA7 -> PWMB)
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_PERIODO - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* Niveles iniciales */
  HAL_GPIO_WritePin(GPIOA, TRIG_PIN | BIN1_PIN | BIN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, AIN1_PIN | AIN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OLED_PORT, OLED_CLK | OLED_MOSI | OLED_DC, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OLED_PORT, OLED_CS | OLED_RES, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  /* PA0 -> ECHO */
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

  /* PA1 TRIGGER, PA4 BIN1, PA5 BIN2 */
  GPIO_InitStruct.Pin = TRIG_PIN | BIN1_PIN | BIN2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB10 AIN1, PB11 AIN2 */
  GPIO_InitStruct.Pin = AIN1_PIN | AIN2_PIN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PA8 -> STBY */
  GPIO_InitStruct.Pin = STBY_PIN;
  HAL_GPIO_Init(STBY_PORT, &GPIO_InitStruct);

  /* PB8 CS, PB12 RES, PB13 CLK, PB14 DC, PB15 MOSI -> OLED */
  GPIO_InitStruct.Pin = OLED_CS | OLED_RES | OLED_CLK | OLED_DC | OLED_MOSI;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  /* PC13 -> LED */
  GPIO_InitStruct.Pin = LED_PIN;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  /* PA6 -> PWMA, PA7 -> PWMB */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB0 -> Batería, PB1 -> LM35 (analógicas) */
  GPIO_InitStruct.Pin = BAT_PIN | LM35_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB6 -> Encoder izquierdo, PB7 -> Encoder derecho */
  GPIO_InitStruct.Pin = ENC_A_PIN | ENC_B_PIN;
#if (FLANCOS_POR_RANURA == 1)
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;           /* Solo subida   */
#else
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;   /* Subida y bajada */
#endif
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}


/* USER CODE BEGIN 4 */

/* =====================================================================
   INTERRUPCIÓN DE LOS ENCODERS (PB6 y PB7 comparten EXTI9_5)
   Si CubeMX genera este handler en stm32f1xx_it.c, BORRA este de aquí.
   ===================================================================== */

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ENC_A_PIN);
  HAL_GPIO_EXTI_IRQHandler(ENC_B_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t ahora = Micros();

  if (GPIO_Pin == ENC_A_PIN)
  {
    if ((ahora - ultimo_A_us) < DEBOUNCE_US) return;
    ultimo_A_us = ahora;

    cuenta_A++;
    if (led_encoder == 1) HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

    if (tramo_activo && !fin_A && (cuenta_A >= objetivo_A))
    {
      MotorA_Freno();
      fin_A = 1;
    }
  }
  else if (GPIO_Pin == ENC_B_PIN)
  {
    if ((ahora - ultimo_B_us) < DEBOUNCE_US) return;
    ultimo_B_us = ahora;

    cuenta_B++;
    if (led_encoder == 2) HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

    if (tramo_activo && !fin_B && (cuenta_B >= objetivo_B))
    {
      MotorB_Freno();
      fin_B = 1;
    }
  }
}

/* =====================================================================
   INTERRUPCIÓN DE LA UART1 (Bluetooth HC-05)
   Se ejecuta sola cada vez que llega un byte desde el celular.
   Lo guarda en un buffer circular para que main() lo procese.
   Si CubeMX genera este handler en stm32f1xx_it.c, BORRA este de aquí.
   ===================================================================== */

void USART1_IRQHandler(void)
{
  uint32_t sr = USART1->SR;
  uint8_t  dato;
  uint8_t  siguiente;

  if (sr & (USART_SR_RXNE | USART_SR_ORE))
  {
    dato = (uint8_t)USART1->DR;        /* Leer DR limpia la bandera */

    siguiente = (uint8_t)((bt_cabeza + 1) % BT_BUFFER);
    if (siguiente != bt_cola)          /* Si el buffer está lleno, se descarta */
    {
      bt_buffer[bt_cabeza] = dato;
      bt_cabeza = siguiente;
    }
  }
}

/* =====================================================================
   INTERRUPCIÓN DE LA UART2 (WiFi ESP8266)
   Guarda cada byte que responde el módulo WiFi.
   Si CubeMX genera este handler en stm32f1xx_it.c, BORRA este de aquí.
   ===================================================================== */

void USART2_IRQHandler(void)
{
  uint32_t sr = USART2->SR;
  uint8_t  dato;
  uint16_t siguiente;

  if (sr & (USART_SR_RXNE | USART_SR_ORE))
  {
    dato = (uint8_t)USART2->DR;        /* Leer DR limpia la bandera */

    siguiente = (uint16_t)((esp_cabeza + 1) % ESP_BUFFER);
    if (siguiente != esp_cola)
    {
      esp_buffer[esp_cabeza] = dato;
      esp_cabeza = siguiente;
    }
  }
}

/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
