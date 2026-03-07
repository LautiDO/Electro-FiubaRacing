#include <libmaple/spi.h>//control spi
#include <libmaple/gpio.h> //control registros
#include <libmaple/rcc.h> //control del clock

// Datos simples para enviar
uint8_t misDatos[] = {10, 20, 30, 40, 50}; 
int i = 0;

void setup() {
  Serial.begin(115200);
  rcc_clk_enable(RCC_SPI1); //habilito el clock del spi
  
  rcc_clk_enable(RCC_GPIOA); //activo el clock del puerto a, osea de pa4 a pa7

  gpio_set_mode(GPIOA, 4, GPIO_INPUT_FLOATING); // NSS
  gpio_set_mode(GPIOA, 5, GPIO_INPUT_FLOATING); // SCK
  gpio_set_mode(GPIOA, 7, GPIO_INPUT_FLOATING); // MOSI
  gpio_set_mode(GPIOA, 6, GPIO_AF_OUTPUT_PP);    // MISO

  SPI1->regs->CR1 = 0; //spi slave, 8 bits, msb
  SPI1->regs->CR1 |= SPI_CR1_SPE; // Habilitar SPI
  //dr = data register
  //sr = status register
  SPI1->regs->DR = misDatos[0]; //seteo en el primer valor para que no tenga basura
}

void loop() {
  // ¿Llegó un byte de la Raspi? (RXNE = Receive buffer Not Empty)
  if (SPI1->regs->SR & SPI_SR_RXNE) {

    uint8_t recibido = SPI1->regs->DR;  //leo el data register  y limpias el rxne para que no entre si no tiene data nueva

    // 2. Preparar el SIGUIENTE dato para la Raspi
    i++;
    if (i >= 5) i = 0; // Reiniciar array si llegamos al final
    
    SPI1->regs->DR = misDatos[i];

    // Opcional: ver qué mandó la Raspi
    Serial.print("Recibido: ");
    Serial.println(recibido);
  }
}
