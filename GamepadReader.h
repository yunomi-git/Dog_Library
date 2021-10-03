#ifndef __JOYSTICKREADER__
#define __JOYSTICKREADER__

#include "SoftwareSerial.h"

#define XBee Serial3  // TODO: make this configurable

struct JoystickValues {
    float rx;
    float ry;
    float lx;
    float ly;
};

class GamepadReader {
private:
	JoystickValues joystick;
	bool new_serial_was_read;
	String buttons_read;
	String last_buttons_read;


public:
	GamepadReader() {
		new_serial_was_read = false;
		buttons_read = "";
		last_buttons_read = "";
		XBee.begin(19200);
	}

	void operate() {
		if (XBee.available()) {
			new_serial_was_read = true;
			joystick.ry = XBee.parseFloat();
	        joystick.rx = XBee.parseFloat();
	        joystick.ly = XBee.parseFloat();
	        joystick.lx = XBee.parseFloat();
	        last_buttons_read = buttons_read;
	        buttons_read = XBee.readStringUntil('\n');

		} else {
			new_serial_was_read = false;
			last_buttons_read = buttons_read;
			flushInput();
		}
	}

	JoystickValues getJoystickValues() {
		return joystick;
	}

	bool buttonWasSinglePressed(String button) {
		bool button_is_pressed = buttonWasReadInBuffer(buttons_read, button);
		bool button_was_pressed = buttonWasReadInBuffer(last_buttons_read, button);
		return (button_is_pressed && !button_was_pressed);
	}

	bool buttonIsBeingPressed(String button) {
		bool button_is_pressed = buttonWasReadInBuffer(buttons_read, button);
		return button_is_pressed;
	}
	
private:
	bool buttonWasReadInBuffer(String buffer, String button) {
		return (buffer.indexOf(button) > 0);
	}

public:
	bool newSerialWasRead() {
		return new_serial_was_read;
	}

	void flushInput() {
		buttons_read = "";
		joystick = JoystickValues();
	}

};


#endif
