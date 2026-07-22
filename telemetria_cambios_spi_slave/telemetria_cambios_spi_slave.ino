//====================================================
//                    LIBRERÍAS
//====================================================
#include <SPI.h>
#include <libmaple/spi.h>
#include <libmaple/gpio.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>
//#include <libmaple/afio.h>


//====================================================
//           ENVÍO DE DATOS (SPI SLAVE)
//
//  Estructura del paquete (9 bytes):
//  [0]      → 0xAA        (sync byte)
//  [1..4]   → cambio      (int32_t, 4 bytes)
//  [5..8]   → tiempo_us   (uint32_t, 4 bytes)
//====================================================
#define MAX_DATOS 9   // 1 sync + 1×4 int + 1×4 uint32_t

volatile uint8_t bufferActivo    = 0;
         uint8_t bufferEscritura = 1;
uint8_t misDatos[2][MAX_DATOS];

volatile int i_dato = 0;


//====================================================
//                      CAMBIOS
//====================================================

#define TOLERANCIA 70
#define NUM_CONFIRMACIONES 6

int t1      = 3420;   // Primera
int t2      = 3070;   // Segunda
int t3      = 2090;   // Tercera
int t4      = 3959;   // Cuarta
int t5      = 3860;   // Quinta
int t6      = 3700;   // Sexta
int neutro  = 4095;   // Neutro
int pin_cambios = PA4;
int tolerancia = 5;

int estado_actual = -1;         // último cambio CONFIRMADO
int candidato = -2;             // valor que se está evaluando
int contador_confirmacion = 0;  // cuántas veces seguidas se repitió el candidato


//====================================================
//                      SETUP
//====================================================
void setup() {
  Serial.begin(115200);

  configurarSPISlave();
  Serial.println("[OK] SPI Slave configurado.");

  memset(misDatos, 0, sizeof(misDatos));
  Serial.println("[OK] Setup completo. Iniciando adquisicion...");
}


//====================================================
//                      LOOP
//====================================================
void loop() {

  //--- Lectura de cambios ---------------------------
  uint16_t cambio_adc = analogRead(pin_cambios);
  int cambio_confirmado = leer_cambio_confirmado(cambio_adc);

  //--- Armar y publicar paquete SPI -----------------
  uint32_t tiempo_actual = micros();
  armarPaquete(cambio_confirmado, tiempo_actual);
  bufferActivo    = bufferEscritura;
  bufferEscritura = 1 - bufferActivo;

  //--- Debug serial ---------------------------------
  Serial.print("cambio=");
  Serial.println(cambio_confirmado);
}


//====================================================
//         CONFIGURACIÓN SPI SLAVE
//====================================================
void configurarSPISlave() {
  // 1. Habilitar relojes
  rcc_clk_enable(RCC_SPI1);
  rcc_clk_enable(RCC_GPIOA);
  rcc_clk_enable(RCC_GPIOB);
  rcc_clk_enable(RCC_AFIO);

  // 2. Liberar pines JTAG (PA15, PB3, PB4) para que funcionen como SPI
  // Esto es CRÍTICO para que el micro no ignore estos pines
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  // 3. Activar el Remap usando la constante correcta de libmaple
  afio_remap(AFIO_REMAP_SPI1);

  // 4. Configurar los pines según el Remap de SPI1
  // NSS: PA15 | SCK: PB3 | MISO: PB4 | MOSI: PB5
  gpio_set_mode(GPIOA, 15, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOB, 3, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOB, 5, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOB, 4, GPIO_AF_OUTPUT_PP);

  // 5. Configuración de registros SPI
  SPI1->regs->CR1  = 0;
  SPI1->regs->CR2 |= SPI_CR2_RXNEIE;
  SPI1->regs->CR1 |= SPI_CR1_SPE;

  // Cargar primer byte
  SPI1->regs->DR = misDatos[bufferActivo][0];
  nvic_irq_enable(NVIC_SPI1);
}


//====================================================
//         ISR — SPI1
//====================================================
extern "C" void __irq_spi1() {
  if (SPI1->regs->SR & SPI_SR_RXNE) {
    (void)SPI1->regs->DR;
    i_dato++;
    if (i_dato >= MAX_DATOS) i_dato = 0;
    SPI1->regs->DR = misDatos[bufferActivo][i_dato];
  }
}


//====================================================
//              FUNCIONES AUXILIARES
//====================================================

void armarPaquete(int cambio, uint32_t t) {
  uint8_t* buf = misDatos[bufferEscritura];
  int pos = 0;
  buf[pos++] = 0xAA;
  memcpy(&buf[pos], &cambio, 4); pos += 4;
  memcpy(&buf[pos], &t,      4); pos += 4; // pos = 9
}

int clasificar_lectura(int valor) {
  if (valor >= (t1 - TOLERANCIA) && valor <= (t1 + TOLERANCIA)) return 1;
  if (valor >= (t2 - TOLERANCIA) && valor <= (t2 + TOLERANCIA)) return 2;
  if (valor >= (t3 - TOLERANCIA) && valor <= (t3 + TOLERANCIA)) return 3;
  if (valor >= (t4 - TOLERANCIA) && valor <= (t4 + TOLERANCIA)) return 4;
  if (valor >= (t5 - TOLERANCIA) && valor <= (t5 + TOLERANCIA)) return 5;
  if (valor >= (t6 - TOLERANCIA) && valor <= (t6 + TOLERANCIA)) return 6;
  if (valor >= (neutro - TOLERANCIA) && valor <= (neutro + TOLERANCIA)) return 0;
  return -1; // en tránsito, no matchea ninguna
}

int leer_cambio_confirmado(int valor_adc) {
  int lectura = clasificar_lectura(valor_adc);

  if (lectura == -1) {
    // en tránsito: se resetea el conteo, no se confirma nada
    contador_confirmacion = 0;
    candidato = -2;
    return estado_actual;
  }

  if (lectura == candidato) {
    contador_confirmacion++;
  } else {
    candidato = lectura;
    contador_confirmacion = 1;
  }

  if (contador_confirmacion >= NUM_CONFIRMACIONES) {
    estado_actual = candidato; // se confirma (aunque sea el mismo que ya tenía)
  }

  return estado_actual;
}
