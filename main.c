#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "qfplib-m3.h"

//origin on left motor, 0 deg mark at 9 o'clock(12 being perpendicular to the line between motor axles, pointing to the draw-area)
//sizes in mm
//rads?


#define pimul2 (float)pi*2
#define pi (float)3.14159265358979323846
#define pidiv2 (float)pi/2

#define updatespermm (uint8_t)2
#define size (float)1//hard bake this?,or will this get optimized even if it gets passed around
#define axleoffset (float) 10
#define stepdelay (uint32_t)1000
#define stepsperrotation (float)4096
#define stepsperrad (float)stepsperrotation/pimul2

#define l_highright (float)10
#define l_lowright (float)10
#define l_highleft (float)10
#define l_lowleft (float)10

#define l_highright2 (float)l_highright*l_highright
#define l_lowright2 (float)l_lowright*l_lowright
#define l_highleft2 (float)l_highleft*l_highleft
#define l_lowleft2 (float)l_lowleft*l_lowleft

const uint32_t pinsetarray[]={0x00000001,0x00000003,0x00000002,0x00000006,0x00000004,0x0000000c,0x00000008,0x00000009};
uint16_t stepcounter[]={1024,1024};


void main(){
	// Enalbe periph clocks, does port b clock need to be enabled for adc, and port a?(if needed for rgisters can it be disabled after registers are set)
	RCC->APB2ENR = RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA;

	// pin a9 as altfunc output 50mhz, pina10 as floatin ,rest as floating
	GPIOA->CRH = 0b01000100010001000100010010110100;

	// enable usart, enable rxne, enable tx and rx
	USART1->CR1 = 0b0010000000101100;
	//set baud to 9600 (72000000/16/(468+12/16))
	USART1->BRR = 0b0001110101001100;

	//enable usart interrupt
	NVIC_EnableIRQ(USART1_IRQn);

	while(1){

	}
}

void USART1_IRQHandler(void){
	//check RXNE handler
	if((USART1->SR & 0b0000000000100000) != 0){
		//check character
		if((char)(USART1->DR)=='c'){
			USART1->DR = '!';
			// Wait until Tx data register is empty
			while((USART1->SR & USART_FLAG_TXE) == 0)
			{
			}
		}
	}
	USART1->SR = 0;// needed?
}

void sequencedigits(){
	//switch
}

void lift(){

}

void drawarc(){

}

void drawline(){

}

void setxy(float x, float y){
	//argument boundries?
	float l = qfp_fsqrt(qfp_fmul(x,x)+qfp_fmul(y,y));
	float l2 =qfp_fmul(l,l);
	movemotorto(0,qfp_fadd(qfp_fdiv(qfp_fsub(pidiv2,qfp_fatan2(qfp_fsub(qfp_fadd(l2,l_lowright2),l_highright2),1)),qfp_fmul(qfp_fmul(2,l2),l_lowright2)),qfp_fatan2(y,x)));
}

void movemotorto(uint8_t motor,float angle){
	uint32_t targetcounter=qfp_fmul(stepsperrad,angle);
	if(stepcounter[motor]-targetcounter<0){

	}
	//wraparound?
	while(targetcounter!=stepcounter[motor]){
		stepcounter+=dir;//uint+ int?
		GPIOA->ODR=pinsetarray[stepcounter[motor]%3];
		wait(stepdelay);
	}
	return;
}

void wait(uint32_t counts){
	while(counts!=0){
		counts--;
	}
	return;
}
