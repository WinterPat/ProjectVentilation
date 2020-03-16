/*
 * IntegerEdit.h
 *
 *  Created on: 2.2.2016
 *      Author: krl
 */

#ifndef INTEGEREDIT_H_
#define INTEGEREDIT_H_

#include "PropertyEdit.h"
#include "LiquidCrystal.h"
#include <string>

class IntegerEdit: public PropertyEdit {
public:
	IntegerEdit(LiquidCrystal *lcd_, std::string editTitle,double upper,double lower,double step);
	virtual ~IntegerEdit();
	void increment();
	void decrement();
	void accept();
	void cancel();
	void setFocus(bool focus);
	bool getFocus();
	void display();
	double getValue();
	void setValue(double value);

private:
	void save();
	void displayEditValue();
	LiquidCrystal *lcd;
	std::string title;
	/*int value;
	int edit;
	int upper;
	int lower;*/
	double value;
	double edit;
	double upper;
	double lower;
	double step;
	bool focus;
};

#endif /* INTEGEREDIT_H_ */
