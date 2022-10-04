#pragma config(Sensor, S1,     ColorSensor,    sensorEV3_Color, modeEV3Color_Color)
#pragma config(Sensor, S4,     USonicSensor,   sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          lWheel,        tmotorEV3_Large, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorD,          rWheel,        tmotorEV3_Large, PIDControl, driveRight, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


const int topSpeed = 0;

void greenAction(float &dist, int &beeped);
void blueAction(int &r);

// assumes distance is already less than 10cm
void greenAction(float &dist, int &beeped) {
	// push the object out of the way

	if (dist <= 10 && beeped == 0) {
		//beep
		setMotorSpeed(lWheel, 0);
		setMotorSpeed(rWheel, 0);
		playSound(soundBeepBeep);

		sleep(2000);

		beeped = 1;
		} else {
		// rotate
		motor[lWheel] = 20;
		motor[rWheel] = -20;
		sleep(2000);

		// move it forward
		motor[lWheel] = 30;
		motor[rWheel] = 30;
		sleep(2000);

		// move backwards
		motor[lWheel] = -30;
		motor[rWheel] = -30;
		sleep(2000);

		// rotate back
		motor[lWheel] = -20;
		motor[rWheel] = 20;
		sleep(2000);
		}

}

void blueAction(int &r) {
	// turn around and go back to the beginning
	motor[lWheel] = 30;
	motor[rWheel] = -30;
	sleep(3000);

	r = 1;
}

task main()
{
	long redValue;
	long greenValue;
	long blueValue;

	float distance;
	int rotatedAlready = 0;
	int beeped = 0;

	//Keep looping forever
	while(true)
	{
		//If the color sensor senses the line (a dark value
		//lower than the calculated threshold of 50):

		getColorRGB(ColorSensor, redValue, greenValue, blueValue);
		long hue = getColorHue(ColorSensor);
		eraseDisplay();
		displayStringAt(0,15, "red: %d", redValue);
		displayStringAt(0,30, "blue: %d", blueValue);
		displayStringAt(0,45, "green: %d", greenValue);
		displayStringAt(0,60, "hue: %d", hue);
		long intens = (redValue + blueValue + greenValue) / 3;

		distance = getUSDistance(USonicSensor);

		/*ON THE TABLE -------------------------------------------- */
		if (redValue >= 51 && redValue <= 56 &&
			blueValue >= 56 && blueValue <= 60 &&
		greenValue >= 61 && greenValue <= 63 && intens >= 56) {
			displayStringAt(0, 75, "ON: Table or White");
			setMotorSpeed(lWheel, 0);
			setMotorSpeed(rWheel, topSpeed);

			/*GREEN EDGE LINE -------------------------------------------- */
		} else if (redValue >= 13 && redValue <= 38 &&
			blueValue >= 27 && blueValue <= 44 &&
		greenValue >= 37 && greenValue <= 51) {
			displayStringAt(0, 75, "ON: Green Edge");
			setMotorSpeed(lWheel, topSpeed);
			setMotorSpeed(rWheel, topSpeed);
			greenAction(distance, beeped);

			/*GREEN DARK -------------------------------------------- */
		} else if (redValue >= 4 && redValue <= 11 &&
			blueValue >= 17 && blueValue <= 27 &&
		greenValue >= 25 && greenValue <= 42) {
			displayStringAt(0, 75, "ON: Green Dark");
			setMotorSpeed(lWheel, 0);
			setMotorSpeed(rWheel, 0);
			greenAction(distance, beeped);

			/* BLUE DARK -------------------------------------------- */
		} else if (redValue >= 4 && redValue <= 12 &&
			blueValue >= 21 && blueValue <= 31 &&
		greenValue >= 10 && greenValue <= 15) {
			displayStringAt(0, 75, "ON: Blue Dark");
			// if haven't rotated yet rotate
			if (rotatedAlready == 0) {
				blueAction(rotatedAlready);
				} else {
				// rotate right to follow the line
				setMotorSpeed(lWheel, 0);
				setMotorSpeed(rWheel, 0);
			}
			/* BLUE EDGE LINE --------------------------------------------*/
		} else if (redValue >= 14 && redValue <= 33 &&
			blueValue >= 28 && blueValue <= 35 &&
		greenValue >= 19 && greenValue <= 33) {
			displayStringAt(0, 75, "ON: Blue Edge");
			// if haven't rotated yet rotate
			if (rotatedAlready == 0) {
				blueAction(rotatedAlready);
				} else {
				setMotorSpeed(lWheel, topSpeed);
				setMotorSpeed(rWheel, topSpeed);
			}
			} else if (intens < 20) {
			setMotorSpeed(lWheel, topSpeed);
			setMotorSpeed(rWheel, 0);
			} else {

		}
	}
}

