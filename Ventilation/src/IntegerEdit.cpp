/*
 * IntegerEdit.cpp
 *
 *  Created on: 2.2.2016
 *      Author: krl
 */

#include "IntegerEdit.h"
#include <cstdio>
#include <stdio.h>
#include <math.h>

IntegerEdit::IntegerEdit(LiquidCrystal *lcd_, std::string editTitle,double _upper,double _lower,double _step): lcd(lcd_), title(editTitle) {


	upper = _upper;
	lower = _lower;
	step = _step;
	value = 0.0;
	edit = 0.0;
	focus = false;

}
IntegerEdit::~IntegerEdit() {
}

void IntegerEdit::increment() {

	if(edit<upper){
		//edit++;
		edit = edit + step;

	}


}

void IntegerEdit::decrement() {



	if(edit >= lower+step ){
		edit = edit - step;
	}
}

void IntegerEdit::accept() {
	save();
}

void IntegerEdit::cancel() {
	edit = value;
}


void IntegerEdit::setFocus(bool focus) {
	this->focus = focus;
}

bool IntegerEdit::getFocus() {
	return this->focus;
}

void IntegerEdit::display() {
	lcd->clear();
	lcd->setCursor(0,0);
	lcd->print(title);
	//lcd->setCursor(0,1);
	char s[17];
	if(focus) {
		snprintf(s, 17, "[%.f]     ", edit);
	}
	else {
		snprintf(s, 17, " %.f      ", edit);
	}
	lcd->print(s);
}


void IntegerEdit::save() {
	// set current value to be same as edit value
	value = edit;
	// todo: save current value for example to EEPROM for permanent storage
}


double IntegerEdit::getValue() {
	return value;
}
void IntegerEdit::setValue(double value) {
	edit = value;
	save();
}
