/*
 * DigitalIoPin.h
 *
 *  Created on: 7.2.2020
 *      Author: emilb
 */

#ifndef DIGITALIOPIN_H_
#define DIGITALIOPIN_H_





#endif /* DIGITALIOPIN_H_ */

class DigitalIoPin{
public:
	DigitalIoPin(int port, int pin, bool input = true, bool pullup = true, bool Invert = true){
		this->pin = pin;
		this->port = port;
		this->invert = Invert;
		if(Invert)Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, (IOCON_MODE_PULLUP | IOCON_DIGMODE_EN | IOCON_INV_EN));
		else(Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, (IOCON_MODE_PULLUP | IOCON_DIGMODE_EN )));
		if(pullup) Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, (IOCON_MODE_PULLUP | IOCON_DIGMODE_EN));
		else Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, (IOCON_MODE_PULLDOWN | IOCON_DIGMODE_EN));
		if(input) Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
		else Chip_GPIO_SetPinDIROutput(LPC_GPIO, port, pin);
	}
	virtual ~DigitalIoPin(){};
	bool read(){
		return Chip_GPIO_GetPinState(LPC_GPIO, port, pin);
	}
	void write(bool value){
		if(invert){
			Chip_GPIO_SetPinState(LPC_GPIO, port, pin, !value);
		}else{
			Chip_GPIO_SetPinState(LPC_GPIO, port, pin, value);
		}
	}
	void operator=(const bool b){
		write(b);
	}
	void flip(){
		LPC_GPIO->B[port][pin] = !LPC_GPIO->B[port][pin];
	}
private:
	int port;
	int pin;
	bool invert;
};
