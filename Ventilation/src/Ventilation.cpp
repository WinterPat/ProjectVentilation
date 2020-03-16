/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

/* I2C clock is set to 1.8MHz */
#define I2C_CLK_DIVIDER         (40)
/* 100KHz I2C bit-rate */
#define I2C_BITRATE         (50000)
#define TICKRATE_HZ (1000)
/* Standard I2C mode */
#define I2C_MODE    (0)
#include <cr_section_macros.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here
#define  DevAddr  (0x40)

#include <cstring>
#include <cstdio>
#include "ITM_write.h"
//#include "DigitalIoPin.h"
#include "LiquidCrystal.h"
#include "ModbusMaster.h"
#include "ModbusRegister.h"
#include "LpcUart.h"
#include "BarGraph.h"
#include "SimpleMenu.h"
#include "MenuItem.h"
#include "PropertyEdit.h"
#include "IntegerEdit.h"
#include "I2C.h"
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/


/*****************************************************************************
 * Public functions
 ****************************************************************************/
static volatile int counter;
static volatile int counter2;
static volatile uint32_t systicks;

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
class Printer{
public:
	Printer(){
		ITM_init();
	}
	void print(const std::string str){
		ITM_write(str.c_str());
	}
private:

};
void SysTick_Handler(void)
{
	systicks++;
	counter2++;
	if(counter > 0) counter--;
}
#ifdef __cplusplus
}
#endif

void Sleep(int ms)
{
	counter = ms;
	while(counter > 0) {
		__WFI();
	}
}
static void Init_I2C_PinMux(void)
{
#if defined(BOARD_KEIL_MCB1500)||defined(BOARD_NXP_LPCXPRESSO_1549)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
#else
#error "No I2C Pin Muxing defined for this example"
#endif
}
// TODO: insert other include files here
static void setupI2CMaster()
{
	/* Enable I2C clock and reset I2C peripheral - the boot ROM does not
	   do this */
	Chip_I2C_Init(LPC_I2C0);

	/* Setup clock rate for I2C */
	Chip_I2C_SetClockDiv(LPC_I2C0, I2C_CLK_DIVIDER);

	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C0, I2C_BITRATE);

	/* Enable Master Mode */
	Chip_I2CM_Enable(LPC_I2C0);
}



// TODO: insert other definitions and declarations here

void ADC_StartCalibration(LPC_ADC_T *pADC)
{
	// clock divider is the lowest 8 bits of the control register
	/* Setup ADC for about 500KHz (per UM) */
	uint32_t ctl = (Chip_Clock_GetSystemClockRate() / 500000) - 1;
	/* Set calibration mode */
	ctl |= ADC_CR_CALMODEBIT;
	pADC->CTRL = ctl;
	/* Calibration is only complete when ADC_CR_CALMODEBIT bit has cleared */
	while(pADC->CTRL & ADC_CR_CALMODEBIT) { };
}
bool enter_critical(void)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	return (pm & 1) == 0;
}
// restore interrupt enable state
void leave_critical(bool enable)
{
	if(enable) __enable_irq();
}
/* this function is required by the modbus library */
uint32_t millis() {
	return systicks;
}

#if 0
void printRegister(ModbusMaster& node, uint16_t reg)
{
	uint8_t result;
	// slave: read 16-bit registers starting at reg to RX buffer
	result = node.readHoldingRegisters(reg, 1);

	// do something with data if read is successful
	if (result == node.ku8MBSuccess)
	{
		printf("R%d=%04X\n", reg, node.getResponseBuffer(0));
	}
	else {
		printf("R%d=???\n", reg);
	}
}

bool setFrequency(ModbusMaster& node, uint16_t freq)
{
	uint8_t result;
	int ctr;
	bool atSetpoint;
	const int delay = 500;

	node.writeSingleRegister(1, freq); // set motor frequency

	printf("Set freq = %d\n", freq/40); // for debugging

	// wait until we reach set point or timeout occurs
	ctr = 0;
	atSetpoint = false;
	do {
		Sleep(delay);
		// read status word
		result = node.readHoldingRegisters(3, 1);
		// check if we are at setpoint
		if (result == node.ku8MBSuccess) {
			if(node.getResponseBuffer(0) & 0x0100) atSetpoint = true;
		}
		ctr++;
	} while(ctr < 20 && !atSetpoint);

	printf("Elapsed: %d\n", ctr * delay); // for debugging

	return atSetpoint;
}


void abbModbusTest()
{




	ModbusMaster node(2); // Create modbus object that connects to slave id 2
	node.begin(9600); // set transmission rate - other parameters are set inside the object and can't be changed here

	printRegister(node, 3); // for debugging

	node.writeSingleRegister(0, 0x0406); // prepare for starting

	printRegister(node, 3); // for debugging

	Sleep(1000); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well

	printRegister(node, 3); // for debugging

	node.writeSingleRegister(0, 0x047F); // set drive to start mode

	printRegister(node, 3); // for debugging

	Sleep(1000); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well

	printRegister(node, 3); // for debugging

	int i = 0;
	int j = 0;
	const uint16_t fa[20] = { 1000, 2000, 3000, 3500, 4000, 5000, 7000, 8000, 10000, 15000, 20000, 9000, 8000, 7000, 6000, 5000, 4000, 3000, 2000, 1000 };

	while (1) {
		uint8_t result;

		// slave: read (2) 16-bit registers starting at register 102 to RX buffer
		j = 0;
		do {
			result = node.readHoldingRegisters(102, 2);
			j++;
		} while(j < 3 && result != node.ku8MBSuccess);
		// note: sometimes we don't succeed on first read so we try up to threee times
		// if read is successful print frequency and current (scaled values)
		if (result == node.ku8MBSuccess) {
			printf("F=%4d, I=%4d  (ctr=%d)\n", node.getResponseBuffer(0), node.getResponseBuffer(1),j);
		}
		else {
			printf("ctr=%d\n",j);
		}

		Sleep(3000);
		i++;
		if(i >= 20) {
			i=0;
		}
		// frequency is scaled:
		// 20000 = 50 Hz, 0 = 0 Hz, linear scale 400 units/Hz
		setFrequency(node, fa[i]);
	}
}
#else
bool setFrequency(ModbusMaster& node, uint16_t freq)
{
	int result;
	int ctr;
	bool atSetpoint;
	const int delay = 500;

	ModbusRegister Frequency(&node, 1); // reference 1
	ModbusRegister StatusWord(&node, 3);

	Frequency = freq; // set motor frequency

	printf("Set freq = %d\n", freq/40); // for debugging

	// wait until we reach set point or timeout occurs
	ctr = 0;
	atSetpoint = false;
	do {
		Sleep(delay);
		// read status word
		result = StatusWord;
		// check if we are at setpoint
		if (result >= 0 && (result & 0x0100)) atSetpoint = true;
		ctr++;
	} while(ctr < 20 && !atSetpoint);

	printf("Elapsed: %d\n", ctr * delay); // for debugging

	return atSetpoint;
}


ModbusMaster abbModbusTest()
{
	ModbusMaster node(2); // Create modbus object that connects to slave id 2
	node.begin(9600); // set transmission rate - other parameters are set inside the object and can't be changed here

	ModbusRegister ControlWord(&node, 0);
	ModbusRegister StatusWord(&node, 3);
	ModbusRegister OutputFrequency(&node, 102);
	ModbusRegister Current(&node, 103);



	ControlWord = 0x0406; // prepare for starting


	Sleep(10); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes wel

	ControlWord = 0x047F; // set drive to start mode

	Sleep(10); // give converter some time to set up
	// note: we should have a startup state machine that check converter status and acts per current status
	//       but we take the easy way out and just wait a while and hope that everything goes well



	return node;
}
#endif


void modbusTest()
{
	ModbusMaster node(2); // Create modbus object that connects to slave id 2


	while (1) {
		static uint32_t i;
		uint8_t j, result;
		uint16_t data[6];

		for(j = 0; j < 6; j++) {
			i++;
			// set word(j) of TX buffer to least-significant word of counter (bits 15..0)
			node.setTransmitBuffer(j, i & 0xFFFF);
		}
		// slave: write TX buffer to (6) 16-bit registers starting at register 0
		result = node.writeMultipleRegisters(0, j);

		// slave: read (6) 16-bit registers starting at register 2 to RX buffer
		result = node.readHoldingRegisters(2, 6);

		// do something with data if read is successful
		if (result == node.ku8MBSuccess)
		{
			for (j = 0; j < 6; j++)
			{
				data[j] = node.getResponseBuffer(j);
			}
			printf("%6d, %6d, %6d, %6d, %6d, %6d\n", data[0], data[1], data[2], data[3], data[4], data[5]);
		}
		Sleep(1000);
	}
}



/**
 * @brief	Main UART program body
 * @return	Always returns 1
 */
int main(void)
{

#if defined (__USE_LPCOPEN)
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
	// Set up and initialize all required blocks and
	// functions related to the board hardware
	Board_Init();
	SysTick_Config(SystemCoreClock / 1000);
	Chip_RIT_Init(LPC_RITIMER);

#endif
#endif
	Chip_ADC_Init(LPC_ADC0, 0);
	/* Setup for ADC clock rate */
	Chip_ADC_SetClockRate(LPC_ADC0, 500000);
	/* For ADC0, sequencer A will be used without threshold events.
 It will be triggered manually, convert CH8 and CH10 in the sequence */
	Chip_ADC_SetupSequencer(LPC_ADC0, ADC_SEQA_IDX, (ADC_SEQ_CTRL_CHANSEL(8) |
			ADC_SEQ_CTRL_CHANSEL(10) | ADC_SEQ_CTRL_MODE_EOS));
	// fix this and check if this is needed
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 0, (IOCON_MODE_INACT | IOCON_ADMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 0, (IOCON_MODE_INACT | IOCON_ADMODE_EN));
	/* For ADC0, select analog input pin for channel 0 on ADC0 */
	Chip_ADC_SetADC0Input(LPC_ADC0, 0);
	/* Use higher voltage trim for both ADC */
	Chip_ADC_SetTrim(LPC_ADC0, ADC_TRIM_VRANGE_HIGHV);
	/* Assign ADC0_8 to PIO1_0 via SWM (fixed pin) and ADC0_10 to PIO0_0 */
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_8);
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_10);
	/* Need to do a calibration after initialization and trim */
	//while (!(Chip_ADC_IsCalibrationDone(LPC_ADC0))); // The NXP library function violates their own access rules given in data sheet so we can't use it
	ADC_StartCalibration(LPC_ADC0);
	/* Set maximum clock rate for ADC */
	/* Our CPU clock rate is 72 MHz and ADC clock needs to be 50 MHz or less
	 * so the divider must be at least two. The real divider used is the value below + 1
	 */
	Chip_ADC_SetDivider(LPC_ADC0, 1);
	/* Chip_ADC_SetClockRate set the divider but due to rounding error it sets the divider too low
	 * which results in a clock rate that is out of allowed range
	 */
	//Chip_ADC_SetClockRate(LPC_ADC0, 500000); // does not work with 72 MHz clock when we want maximum frequency
	/* Clear all pending interrupts and status flags */
	Chip_ADC_ClearFlags(LPC_ADC0, Chip_ADC_GetFlags(LPC_ADC0));
	/* Enable sequence A completion interrupts for ADC0 */
	Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE);
	/* We don't enable the corresponding interrupt in NVIC so the flag is set but no interrupt is
triggered */
	/* Enable sequencer */
	Chip_ADC_EnableSequencer(LPC_ADC0, ADC_SEQA_IDX);
	/* Configure systick timer */

	SysTick_Config(Chip_Clock_GetSysTickClockRate() / TICKRATE_HZ);
	Init_I2C_PinMux();

	setupI2CMaster();

	NVIC_DisableIRQ(I2C0_IRQn);

	LpcPinMap none = {-1, -1}; // unused pin has negative values in it
	LpcPinMap txpin = { 0, 18 }; // transmit pin that goes to debugger's UART->USB converter
	LpcPinMap rxpin = { 0, 13 }; // receive pin that goes to debugger's UART->USB converter
	LpcUartConfig cfg = { LPC_USART0, 115200, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1, false, txpin, rxpin, none, none };
	LpcUart dbgu(cfg);

	/* Set up SWO to PIO1_2 */
	Chip_SWM_MovablePortPinAssign(SWM_SWO_O, 1, 2); // Needed for SWO printf

	/* Enable and setup SysTick Timer at a periodic rate */
	SysTick_Config(SystemCoreClock / 1000);

	DigitalIoPin rs(0,8,false,false,false);
	DigitalIoPin en(1,6,false,false,false);
	DigitalIoPin d4(1,8,false,false,false);
	DigitalIoPin d5(0,5,false,false,false);
	DigitalIoPin d6(0,6,false,false,false);
	DigitalIoPin d7(0,7,false,false,false);

	LiquidCrystal *lcd = new LiquidCrystal(&rs, &en, &d4, &d5, &d6, &d7);

	DigitalIoPin sw1(0,9,true, true, false);
	DigitalIoPin sw2(0,10,true, true, false);
	DigitalIoPin sw3(1,3,true, true, false);
	Printer pri;

	I2C_config conf;
	I2C i2c(conf);

	SimpleMenu menu;

	IntegerEdit *frequency = new IntegerEdit(lcd, std::string("M Freq "),100 ,0 ,10);

	IntegerEdit *pressure = new IntegerEdit(lcd, std::string("A Pres "),125 ,0 , 25);

	menu.addItem(new MenuItem(frequency));
	menu.addItem(new MenuItem(pressure));

	frequency->setValue(50.0);
	pressure->setValue(25);



	ModbusMaster node(2);
	menu.event(MenuItem::show);
	node = abbModbusTest();

	bool state=false;
	bool clear = false;
	bool automode = true;

	uint16_t sel_freq;//selected freq
	uint16_t sel_pr;//selected pressure
	uint16_t freq = 0;//freq
	uint16_t prev_freq = 0;//previous freq
	int prev_pr;//previous pressure
	float mul_freq = 0.0;//multiplier for freq



	while(1){
		uint8_t val[3];
		uint8_t read = (0xF1);
		prev_pr = pressure->getValue();
		lcd->clear();
		menu.event(MenuItem::show);
		i2c.transaction(DevAddr,&read,1,val,3);
		int16_t pres = ((int16_t)val[0] << 8) | val[1];
		int16_t psa;
		psa = pres/240*0.95f;




		if(!sw1.read()){
			menu.event(MenuItem::ok);
			if(state==false){
				state=true;
				if(automode == false){

				}
				else if(automode == true){

				}
				lcd->setCursor(0, 1);
				lcd->print("  Pres  ");
				lcd->print(std::to_string(psa));
			}
			else{
				state=false;
				if(automode == false){

				}
				else if(automode == true){

					counter2 = 0;
				}
				lcd->setCursor(0, 1);
				lcd->print("  Pres  ");
				lcd->print(std::to_string(psa));
			}

			counter2=0;
			while(!sw1.read());
		}
		if(!sw2.read()){
			menu.event(MenuItem::up);


			if(state==false){
				if(automode == false){
					automode = true;

				}
				else if(automode == true){
					automode = false;

					counter2 = 0;
				}
				lcd->setCursor(0, 1);
				lcd->print("  Pres  ");
				lcd->print(std::to_string(psa));
			}
			while(!sw2.read());
		}
		if(!sw3.read()){
			menu.event(MenuItem::down);
			if(state==false){
				if(automode == false){
					automode = true;
				}
				else if(automode == true){
					automode = false;
				}
				lcd->setCursor(0, 1);
				lcd->print("  Pres  ");
				lcd->print(std::to_string(psa));
			}
			while(!sw3.read());
		}
		if(counter2>=10000){
			menu.event(MenuItem::back);
			state=false;
		}
		sel_freq = frequency->getValue()*200;
		sel_pr = pressure->getValue();

		if(automode == true){
			freq = sel_freq;
		}
		if(automode == false){
			if(sel_pr==psa && psa != 0);
			else if(sel_pr>psa){

				mul_freq = (sel_pr*10)/(psa);
				freq = freq + (mul_freq*10);

			}
			else if(sel_pr<psa){
				mul_freq = (psa*10)/(sel_pr);
				freq = prev_freq - (mul_freq*10);
				if(sel_pr <= 0){
					freq = 0;
				}
			}
			if(freq >= 20000){
				freq = 20000;
			}
			if(freq <= 0){
				freq = 0;
			}
		}
		lcd->setCursor(0, 1);
		lcd->print("  Pres  ");
		lcd->print(std::to_string(psa));

		if(sel_pr != prev_pr){
			counter2 = 0;
		}
		if(automode == false){
			if(counter2 >= 5000 && psa != sel_pr)
			{
				lcd->clear();
				lcd->setCursor(0, 0);
				lcd->print("PRESSURE NOT");
				lcd->setCursor(0, 1);
				lcd->print("REACHED!");
				clear = true;
				counter2 = 0;
			}else if(psa == sel_pr){
				counter2 = 0;
			}
			else if(clear == true){
				menu.event(MenuItem::show);
				clear = false;
			}
		}
		setFrequency(node, freq);
		prev_freq = freq;

		Sleep(1);
	}
	return 1;
}

