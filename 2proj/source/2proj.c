#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"

void delay_ms(unsigned short delay_t)
{
	SIM->SCGC6 |= (1 << 24);
	SIM->SOPT2 |= (0x2 << 24);
	TPM0->CONF |= (0x1 << 17);
	TPM0->SC = (1 << 7) | (0x7);

	TPM0->MOD = delay_t*62 + delay_t/2 -1;

	TPM0->SC |= 0x01 << 3;

	while(!(TPM0->SC & 0x80)){}
	return;

}

int main(void) {


    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif


    SIM->SCGC6 |= (1 << 26); // initializes TPM2
    SIM->SOPT2 |= (0x2 << 24); //src clock = oscerclk
    setup_LED1();
    setup_SW1_interrupt();


    setPortB();
    setPortC();
	setPortB();
	setPortC();
	PTB_0();
	PTB_1();
	//PTB_2();
	//Enables ptb2 as pwm
	PORTB->PCR[2] &= ~(0x700); //clear mux
	PORTB->PCR[2] |= 0x300;
	//set up channel 1
	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4); //EDGE
		TPM2->MOD = 7999;
	TPM2->SC |= 0x01 << 3; //starts clock
	//Enable portb3 as pwm
	PORTB->PCR[3] &= ~(0x700); //clear mux
	PORTB->PCR[3] |= 0x300;
	//set up channel 1
		TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4); //EDGE
	TPM2->MOD = 7999;
	//PTB_3();
	PTC_1();
	PTC_2();
	TPM2->SC |= 0x01 << 3; //starts clock





    volatile static int i = 0 ;

    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}


void setup_LED1() {
SIM->SCGC5 |= (1<<12);
// Setup port for GPIO
PORTD->PCR[5] &= ~0x700; // Clear First
PORTD->PCR[5] |= 0x700 & (1 << 8); // Set MUX bits
GPIOD->PDDR |= (1 << 5); // Setup Pin 5 as output
}

void PORTC_PORTD_IRQHandler(void){
PORTC->PCR[3] |= (1<<24); // Clears interrupt flag

for (int i = 0; i < 10; i++){
	delay_ms(500);
}

square();

for (int i = 0; i < 10; i++){
	delay_ms(500);
}

circle();

for (int i = 0; i < 10; i++){
	delay_ms(500);
}



}

// Setup SW1 with Interrupt enabled
void setup_SW1_interrupt() {
SIM->SCGC5 |= (1<<11);  // Enable Port C Clock
PORTC->PCR[3] &= 0xF0703; // Clear First
PORTC->PCR[3] |= 0xF0703 & ((0xA << 16) | (1 << 8) | 0x3 ); // Set MUX bits,enable pullups, interrupt on falling edge
GPIOC->PDDR &= ~(1 << 3); // Setup Pin 3 Port C as input
// TODO: Leave as priority 0
// TODO: Call Core API to Enable IRQ
NVIC_EnableIRQ(31);
}

void left()
{
	        GPIOB->PDOR |=   (1 << 0);
	        GPIOB->PDOR |=   (1 << 1);
	        //GPIOB->PDOR |=  (1 << 2);
	        TPM2->CONTROLS[0].CnV = 5000;

	        //GPIOB->PDOR |=  (1 << 3);
	        TPM2->CONTROLS[1].CnV = 5000;
	        GPIOC->PDOR &= ~(1 << 2);
	        GPIOC->PDOR |=  (1 << 1);

	        //delay_ms(500);

	        //stop();




}

void stop()
{

				GPIOB->PDOR |=   (1 << 0);
		        GPIOB->PDOR |=   (1 << 1);
		        //GPIOB->PDOR |=  (1 << 2);
		        TPM2->CONTROLS[0].CnV = 5000;

		        //GPIOB->PDOR |=  (1 << 3);
		        TPM2->CONTROLS[1].CnV = 5000;
		        GPIOC->PDOR |=  (1 << 1);
		        GPIOC->PDOR |=  (1 << 2);


}

void forward()
{
	 GPIOB->PDOR &=  ~(1 << 0);
	 GPIOB->PDOR |=   (1 << 1);
     //GPIOB->PDOR |=  (1 << 2);
	 TPM2->CONTROLS[0].CnV = 5000;

	 //GPIOB->PDOR |=  (1 << 3);
	 TPM2->CONTROLS[1].CnV = 5000;
     GPIOC->PDOR &= ~(1 << 2);
	 GPIOC->PDOR |=  (1 << 1);

}

void forward_l()
{
	 GPIOB->PDOR &=  ~(1 << 0);
	 GPIOB->PDOR |=   (1 << 1);
     //GPIOB->PDOR |=  (1 << 2);
	 TPM2->CONTROLS[0].CnV = 5000;

	 //GPIOB->PDOR |=  (1 << 3);
	 TPM2->CONTROLS[1].CnV = 7500;
     GPIOC->PDOR &= ~(1 << 2);
	 GPIOC->PDOR |=  (1 << 1);

}

void forward_r()
{
	 GPIOB->PDOR &=  ~(1 << 0);
	 GPIOB->PDOR |=   (1 << 1);
     //GPIOB->PDOR |=  (1 << 2);
	 TPM2->CONTROLS[0].CnV = 7500;

	 //GPIOB->PDOR |=  (1 << 3);
	 TPM2->CONTROLS[1].CnV = 5000;
     GPIOC->PDOR &= ~(1 << 2);
	 GPIOC->PDOR |=  (1 << 1);

}
void right()
{
	        GPIOB->PDOR &=   ~(1 << 0);
	        GPIOB->PDOR |=   (1 << 1);
	        //GPIOB->PDOR |=  (1 << 2);
	        TPM2->CONTROLS[0].CnV = 5000;

	        //GPIOB->PDOR |=  (1 << 3);
	        TPM2->CONTROLS[1].CnV = 5000;
	        GPIOC->PDOR |=  (1 << 1);
	        GPIOC->PDOR |=  (1 << 2);

	        //delay_ms(500);

	        //stop();




}

void setPortB()
{
	SIM->SCGC5 |= 1<<10;
}


void setPortC()
{
	SIM->SCGC5 |= 1<<11;
}

void PTB_0()
{
	PORTB->PCR[0] &= ~(0x700 << 8);
	PORTB->PCR[0] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 0);
}

void PTB_1()
{
	PORTB->PCR[1] &= ~(0x700 << 8);
	PORTB->PCR[1] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 1);
}

void PTB_2()
{
	PORTB->PCR[2] &= ~(0x700 << 8);
	PORTB->PCR[2] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 2);
}

void PTB_3()
{
	PORTB->PCR[3] &= ~(0x700 << 8);
	PORTB->PCR[3] |= 0x700 & (1 << 8);
	GPIOB->PDDR |= (1 << 3);
}

void PTC_1()
{
	PORTC->PCR[1] &= ~(0x700 << 8);
	PORTC->PCR[1] |= 0x700 & (1 << 8);
	GPIOC->PDDR |= (1 << 1);
}

void PTC_2()
{
	PORTC->PCR[2] &= ~(0x700 << 8);
	PORTC->PCR[2] |= 0x700 & (1 << 8);
	GPIOC->PDDR |= (1 << 2);
}

void square()
{

	// move forward half
	forward();
	delay_ms(900);
	stop();

	//left turn
	left();
	delay_ms(450);
	delay_ms(450);
	delay_ms(450);
	stop();

	//move forward full
	forward();
	delay_ms(600);
	delay_ms(600);
	delay_ms(600);
	stop();
	//left turn
	left();
	delay_ms(450);
	delay_ms(450);
	delay_ms(450);
	stop();

	//MVF
	forward();
	delay_ms(600);
	delay_ms(600);
	delay_ms(600);
	stop();

	//LT
	left();
	delay_ms(450);
	delay_ms(450);
	delay_ms(450);
	stop();

	//MVF
	forward();
	delay_ms(600);
	delay_ms(600);
	delay_ms(600);
	stop();
	//LT
	left();
	delay_ms(450);
	delay_ms(450);
	delay_ms(450);
	stop();

	//MVF
	forward();
	delay_ms(600);
	delay_ms(600);
	delay_ms(600);
	stop();
	//RT
	right();
	delay_ms(450);
	delay_ms(450);
	delay_ms(450);
	stop();
	//MVF
	forward();
	delay_ms(600);
	delay_ms(600);
	delay_ms(600);
	stop();
	//RT
	right();
	delay_ms(450);
	delay_ms(450);
	delay_ms(450);
	stop();
	//MVF
	forward();
	delay_ms(600);
	delay_ms(600);
	delay_ms(600);
	stop();
	//RT
	right();
	delay_ms(450);
	delay_ms(450);
	delay_ms(450);
	stop();
	//MVF
	forward();
	delay_ms(600);
	delay_ms(600);
	delay_ms(600);
	stop();
	//RT
	right();
	delay_ms(450);
	delay_ms(450);
	delay_ms(450);
	stop();
	//Move forward half
	forward();
	delay_ms(900);
	stop();

	// done

}

void circle()
{

	//left turn
	forward_l();
	delay_ms(725);
	delay_ms(725);
	delay_ms(725);
	stop();
	delay_ms(100);

	//left turn
	forward_l();
	delay_ms(725);
	delay_ms(725);
	delay_ms(725);
	stop();
	delay_ms(100);

	//left turn
	forward_l();
	delay_ms(725);
	delay_ms(725);
	delay_ms(725);
	stop();
	delay_ms(100);

	//right turn
	forward_r();
	delay_ms(725);
	delay_ms(725);
	delay_ms(725);
	stop();
	delay_ms(100);

	//right turn
	forward_r();
	delay_ms(725);
	delay_ms(725);
	delay_ms(725);
	stop();
	delay_ms(100);

	//right turn
	forward_r();
	delay_ms(725);
	delay_ms(725);
	delay_ms(725);
	stop();
	delay_ms(100);

	//right turn
	forward_r();
	delay_ms(725);
	delay_ms(725);
	delay_ms(725);
	stop();
	delay_ms(100);

	//left turn
	forward_l();
	delay_ms(725);
	delay_ms(725);
	delay_ms(725);
	stop();


	//done



}
