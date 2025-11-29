#include "nrf52810.h"
#include "nrf_gpio.h"
#include "led.h"

void EN_Init(void)
{
  //ÅäÖÃ·äÃùÆ÷Çı¶¯GPIO
  nrf_gpio_cfg_output(8);
	nrf_gpio_pin_clear(8);	
}




