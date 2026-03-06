#include <libmaple/spi.h>
#include <libmaple/gpio.h>
#include <libmaple/rcc.h>

// Datos simples para enviar
uint8_t misDatos[] = {10, 20, 30, 40, 50}; 
int i = 0;

void setup() {
  Serial.begin(115200);
  rcc_clk_enable(RCC_SPI1);
  
  rcc_clk_enable(RCC_GPIOA);

  gpio_set_mode(GPIOA, 4, GPIO_INPUT_FLOATING); // NSS
  gpio_set_mode(GPIOA, 5, GPIO_INPUT_FLOATING); // SCK
  gpio_set_mode(GPIOA, 7, GPIO_INPUT_FLOATING); // MOSI
  gpio_set_mode(GPIOA, 6, GPIO_AF_OUTPUT_PP);    // MISO

  SPI1->regs->CR1 = 0;
  SPI1->regs->CR1 |= SPI_CR1_SPE; // Habilitar SPI
  
  // Cargar el primer dato manualmente para que esté listo
  SPI1->regs->DR = misDatos[0]; 
}

void loop() {
  // ¿Llegó un byte de la Raspi? (RXNE = Receive buffer Not Empty)
  if (SPI1->regs->SR & SPI_SR_RXNE) {
    
    // 1. Leer lo que mandó la Raspi (esto limpia el flag para recibir el siguiente)
    uint8_t recibido = SPI1->regs->DR; 

    // 2. Preparar el SIGUIENTE dato para la Raspi
    i++;
    if (i >= 5) i = 0; // Reiniciar array si llegamos al final
    
    SPI1->regs->DR = misDatos[i];

    // Opcional: ver qué mandó la Raspi
    Serial.print("Recibido: ");
    Serial.println(recibido);
  }
}
