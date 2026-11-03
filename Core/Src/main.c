/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Carro autónomo STM32F103 (Blue Pill)
  *                   - Puente H TB6612FNG (avance / reversa / freno)
  *                   - 2 encoders HC-020K (medición de distancia y giros)
  *                   - Sensor ultrasónico HC-SR04 (se detiene a <= 10 cm)
  *                   - Recorre un polígono regular N lados, L cm, V vueltas
  *                   - LCD 1602 en modo 4 bits:
  *                       Línea 1: voltaje de batería (ADC) y temperatura (LM35)
  *                       Línea 2: distancia recorrida
  *                   - Control por Bluetooth HC-05 (USART1 con interrupción)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
   ===================================================================== */

#define MODO                   6

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
#define PERIODO_PANTALLA_MS    300      /* Refresco del LCD               */
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
#define AIN1_PORT     GPIOA
#define AIN1_PIN      GPIO_PIN_2
#define AIN2_PORT     GPIOA
#define AIN2_PIN      GPIO_PIN_3
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

/* LCD 1602 en modo 4 bits (RW va a GND) */
#define LCD_PORT      GPIOB
#define LCD_RS        GPIO_PIN_10
#define LCD_E         GPIO_PIN_11
#define LCD_D4        GPIO_PIN_12
#define LCD_D5        GPIO_PIN_13
#define LCD_D6        GPIO_PIN_14
#define LCD_D7        GPIO_PIN_15

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

#if (MODO < 0) || (MODO > 6)
#error "MODO debe ser 0, 1, 2, 3, 4, 5 o 6"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */

static uint32_t Micros(void);
static void     Delay_us(uint32_t us);
static uint32_t Medir_Distancia_cm(void);

static void MotorA_Direccion(Sentido s);
static void MotorB_Direccion(Sentido s);
static void MotorA_Freno(void);
static void MotorB_Freno(void);
static void MotorA_Libre(void);
static void MotorB_Libre(void);
static void MotorA_Velocidad(int32_t pct);
static void MotorB_Velocidad(int32_t pct);

static void LCD_Pulso_E(void);
static void LCD_Nibble(uint8_t n);
static void LCD_Enviar(uint8_t valor, uint8_t es_dato);
static void LCD_Comando(uint8_t cmd);
static void LCD_Iniciar(void);
static void LCD_Linea(uint8_t fila, const char *texto);

static void     ADC_Iniciar(void);
static uint16_t ADC_Leer(uint32_t canal);
static float    ADC_Voltios(uint32_t canal);

static char *Agregar_Texto(char *p, const char *s);
static char *Agregar_Numero(char *p, uint32_t n);
static char *Agregar_Decimal(char *p, uint32_t valor, uint8_t decimales);
static float Distancia_Recorrida_cm(void);
static void  Pantalla_Actualizar(void);

static void Iniciar_Acumulado(Acumulado *acc, float exceso_inicial);
static uint32_t Calcular_Objetivo(float ideal, uint32_t real, float exceso);
static void Arrancar_Motores(Sentido sA, Sentido sB);
static void Ajustar_Velocidades(int32_t vel, uint32_t objA, uint32_t objB);
static void Pausar_Por_Obstaculo(Sentido sA, Sentido sB);
__attribute__((unused))
static void Ejecutar_Tramo(Sentido sA, Sentido sB, float pulsos,
                           Acumulado *acc, int32_t vel,
                           uint8_t vigilar_obstaculo);

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

/* Retardo en microsegundos con TIM2 (cuenta 1 us, de 0 a 59999) */
static void Delay_us(uint32_t us)
{
  uint32_t inicio = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t transcurrido = 0;
  uint32_t ahora;

  while (transcurrido < us)
  {
    ahora = __HAL_TIM_GET_COUNTER(&htim2);
    transcurrido = (ahora >= inicio) ? (ahora - inicio)
                                     : (ahora + TIM2_PERIODO - inicio);
  }
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
   LCD 1602 (HD44780) EN MODO 4 BITS
   Solo se usan D4..D7. Cada byte se envía en dos mitades (nibbles):
   primero la parte alta y luego la baja. RW siempre a GND (solo escritura).
   ===================================================================== */

static void LCD_Pulso_E(void)
{
  HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_SET);
  Delay_us(1);
  HAL_GPIO_WritePin(LCD_PORT, LCD_E, GPIO_PIN_RESET);
  Delay_us(50);                       /* La mayoría de comandos tarda 37 us */
}

static void LCD_Nibble(uint8_t n)
{
  HAL_GPIO_WritePin(LCD_PORT, LCD_D4, (n & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_PORT, LCD_D5, (n & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_PORT, LCD_D6, (n & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_PORT, LCD_D7, (n & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  LCD_Pulso_E();
}

/* es_dato = 1 para un carácter, 0 para un comando */
static void LCD_Enviar(uint8_t valor, uint8_t es_dato)
{
  HAL_GPIO_WritePin(LCD_PORT, LCD_RS, es_dato ? GPIO_PIN_SET : GPIO_PIN_RESET);
  LCD_Nibble(valor >> 4);             /* Parte alta */
  LCD_Nibble(valor & 0x0F);           /* Parte baja */
}

static void LCD_Comando(uint8_t cmd)
{
  LCD_Enviar(cmd, 0);
  if ((cmd == 0x01) || (cmd == 0x02))
  {
    HAL_Delay(2);                     /* Borrar / Home tardan ~1.5 ms */
  }
}

static void LCD_Iniciar(void)
{
  HAL_Delay(50);                      /* Esperar a que el LCD encienda */
  HAL_GPIO_WritePin(LCD_PORT, LCD_RS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_PORT, LCD_E,  GPIO_PIN_RESET);

  /* Secuencia de la hoja de datos para entrar a modo 4 bits */
  LCD_Nibble(0x03);
  HAL_Delay(5);
  LCD_Nibble(0x03);
  Delay_us(150);
  LCD_Nibble(0x03);
  Delay_us(150);
  LCD_Nibble(0x02);                   /* A partir de aquí: 4 bits */

  LCD_Comando(0x28);                  /* 4 bits, 2 líneas, 5x8 puntos  */
  LCD_Comando(0x0C);                  /* Pantalla ON, sin cursor       */
  LCD_Comando(0x06);                  /* Cursor avanza a la derecha    */
  LCD_Comando(0x01);                  /* Borrar pantalla               */
}

/* Escribe una línea completa (16 caracteres, rellena con espacios).
   Así no hace falta borrar la pantalla y no parpadea.                   */
static void LCD_Linea(uint8_t fila, const char *texto)
{
  uint8_t i = 0;

  LCD_Comando((fila == 0) ? 0x80 : 0xC0);

  while ((i < 16) && (texto[i] != '\0'))
  {
    LCD_Enviar((uint8_t)texto[i], 1);
    i++;
  }

  while (i < 16)
  {
    LCD_Enviar(' ', 1);
    i++;
  }
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
static void Pantalla_Actualizar(void)
{
  char  linea[32];
  char *p;
  float vbat;
  float temp;
  float dist;

  vbat = ADC_Voltios(CANAL_BATERIA) * DIVISOR_BATERIA * FACTOR_BATERIA;
  temp = ADC_Voltios(CANAL_LM35) * 100.0f;     /* LM35: 10 mV por °C */
  dist = Distancia_Recorrida_cm();

  if (temp < 0.0f) temp = 0.0f;

  bateria_v     = vbat;
  temperatura_c = temp;
  distancia_cm  = dist;

  p = linea;
  p = Agregar_Texto(p, "B:");
  p = Agregar_Decimal(p, (uint32_t)(vbat * 100.0f + 0.5f), 2);
  p = Agregar_Texto(p, "V T:");
  p = Agregar_Decimal(p, (uint32_t)(temp * 10.0f + 0.5f), 1);
  *p++ = (char)0xDF;                            /* Símbolo de grado */
  *p++ = 'C';
  *p   = '\0';
  LCD_Linea(0, linea);

  p = linea;
  p = Agregar_Texto(p, "Dist: ");
  p = Agregar_Decimal(p, (uint32_t)(dist * 10.0f + 0.5f), 1);
  p = Agregar_Texto(p, " cm");
  *p = '\0';
  LCD_Linea(1, linea);
}

/* =====================================================================
   SEÑALES DE ERROR (LED + LCD)
   1 parpadeo  = rueda IZQUIERDA
   2 parpadeos = rueda DERECHA
   3 parpadeos = las dos
   ===================================================================== */

static void Senal_Error(uint8_t n)
{
  codigo_error = n;

  LCD_Linea(0, "ERROR: RUEDA");
  if (n == 1)      LCD_Linea(1, "IZQUIERDA");
  else if (n == 2) LCD_Linea(1, "DERECHA");
  else             LCD_Linea(1, "AMBAS");

  while (1)
  {
    for (uint8_t i = 0; i < n; i++)
    {
      LED_ON();
      HAL_Delay(250);
      LED_OFF();
      HAL_Delay(250);
    }
    HAL_Delay(1500);
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

  LCD_Linea(0, "MODO PRUEBA");
  LCD_Linea(1, "Motores/encoder");

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

  LCD_Linea(0, "PRUEBA OK");
  LCD_Linea(1, "");

  while (1)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(1000);
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

  BT_Iniciar();
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
  BT_Aplicar('S');

  LCD_Linea(0, "MODO BLUETOOTH");
  LCD_Linea(1, "Conecta el cel");
  BT_Enviar_Texto("Carro listo\r\n");
  HAL_Delay(1500);

  t_comando = HAL_GetTick();

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
    }

    /* 3) Mover los motores */
    efectivo = (obstaculo && Es_Adelante(pedido)) ? 'S' : pedido;

    if (forzar || (efectivo != mov_actual))
    {
      BT_Aplicar(efectivo);
      forzar = 0;
    }

    if (mov_actual == 'S') LED_OFF(); else LED_ON();

    /* 4) Pantalla */
    if ((ahora - t_pantalla) >= PERIODO_PANTALLA_MS)
    {
      t_pantalla = ahora;

      if (obstaculo && Es_Adelante(pedido))
      {
        LCD_Linea(0, "OBSTACULO <10cm");
        LCD_Linea(1, "Ve hacia atras");
      }
      else
      {
        Pantalla_Actualizar();
      }
    }

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

  do
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    Pantalla_Actualizar();
    HAL_Delay(PERIODO_MEDICION_MS);
    d = Medir_Distancia_cm();
  } while (d <= DISTANCIA_REANUDAR);

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
  LCD_Iniciar();
  LCD_Linea(0, "Carro autonomo");
  LCD_Linea(1, "Iniciando...");
  HAL_Delay(800);

  Iniciar_Acumulado(&acc_recta, EXCESO_INICIAL_RECTA);
  Iniciar_Acumulado(&acc_giro,  EXCESO_INICIAL_GIRO);

#if (MODO == 4)

  /* ---------------- SOLO PANTALLA (sin motores) -------------- */
  while (1)
  {
    Pantalla_Actualizar();
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(PERIODO_PANTALLA_MS);
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
        LCD_Linea(0, (motor == 1) ? "Girando: IZQ" : "Girando: DER");
        LCD_Linea(1, "Contar vueltas");
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
        LCD_Linea(0, linea);

        p = linea;
        p = Agregar_Texto(p, "Der (PB7): ");
        p = Agregar_Numero(p, cuenta_B);
        *p = '\0';
        LCD_Linea(1, linea);

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
    HAL_Delay(PERIODO_PANTALLA_MS);
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* Recorrido terminado: LED parpadea y la pantalla sigue actualizando */
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    Pantalla_Actualizar();
    HAL_Delay(500);

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
  HAL_GPIO_WritePin(GPIOA, TRIG_PIN | AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_PORT, LCD_RS | LCD_E | LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  /* PA0 -> ECHO */
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

  /* PA1 TRIGGER, PA2 AIN1, PA3 AIN2, PA4 BIN1, PA5 BIN2 */
  GPIO_InitStruct.Pin = TRIG_PIN | AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA8 -> STBY */
  GPIO_InitStruct.Pin = STBY_PIN;
  HAL_GPIO_Init(STBY_PORT, &GPIO_InitStruct);

  /* PB10..PB15 -> LCD (RS, E, D4, D5, D6, D7) */
  GPIO_InitStruct.Pin = LCD_RS | LCD_E | LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7;
  HAL_GPIO_Init(LCD_PORT, &GPIO_InitStruct);

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
