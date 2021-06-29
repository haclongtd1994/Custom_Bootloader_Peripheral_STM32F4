#include <stdint.h>
#include "Board_LED.h"

void delay(uint32_t time){
	for (volatile uint32_t i=0; i < time;i++){
		for (volatile uint32_t j=0; j < 50000; j++);
	}
}

int main(void) {
	while(1) {
	  LED_Initialize();
		LED_On(0);
		LED_On(1);
		LED_On(2);
		LED_On(3);
		delay(50);
		LED_Off(0);
		LED_Off(1);
		LED_Off(2);
		LED_Off(3);
		delay(50);
	}
}

