/******************************************************************************/
/* Copyright Â© 2024, Spencer Hiscox, Eitan Tesciuba, Batool Noweir, Dennis Lee.
	 All rights reserved.


	 	VERSION:
	 	DATE OF LAST UPDATE: July 12, 2024 5:15 PM



						THIS FILE IS INTENDED TO SERVE AS THE CUBIK'S RUBE?
						SPECIFIC SOFTWARE LIBRARY

		(all function definitions of functions called in the main program file
			are contained here)




UNIVERSITY OF WATERLOO
SOFTWARE WRITTEN AS PART OF ME101 ROBOTICS DESIGN PROJECT
SPRING TERM 2024 |

PROJECT TEAM:
SPENCER HISCOX		20959364
DENNIS LEE				xxxxxxxx
BATOOL NOWEIR			xxxxxxxx
EITAN TESCIUBA		xxxxxxxx

DATE OF FINAL SUBMISSION: XX/XX/2024
SUBMITTED TO:
		DR. PETER TEERTSTRA
		DR. CAROL HULLS
*/
/******************************************************************************/

//TEMPORARY FOR TESTING
const bool _PMC_ACTIVE = false;
const unsigned int _PMC_DECAY_FACTOR = 700;						// 700 <= b <= 1500, 0.001 is a ~linear decay (use desmos for testing / adjustment)
const unsigned int _FAILSAFE_TIMEOUT = 2100;







//SENSOR PORTS
const unsigned int 	_BASE_GYRO	= S1,
										_ARM_GYRO		= S2,
										_SCAN_GYRO	= S3,
										_LIGHT	 		= S4,
										_ENCODER 		= 253,
										_TIME				= 352,

//MOTOR PORTS
										_BASE_MOTOR					= motorA,
										_SCANNING_ARM_MOTOR	= motorB,
										_ARMATURE_MOTOR			= motorD,

//SENSOR MODES
										_RATE 				= modeEV3Gyro_Rate,
										_ANGLE 				= modeEV3Gyro_Angle,
										_RATEandANGLE = modeEV3Gyro_RateAndAngle,
										_COLOUR 			= modeEV3Color_Color,
										_AMBIENT 			= modeEV3Color_Ambient,
										_REFLECTED 		= modeEV3Color_Reflected;

//DIRECTIONAL CONSTANTS
const short int	_LEFT  = -1,
								_RIGHT = 	1,
								_CCW	 = -1,
								_CW		 =	1,
								_FWD	 =	1,
								_REV	 = -1,
								_DOWN	 =  1,
								_UP		 = -1,

//UNIVERSAL CONSTANTS
								_UNSPECIFIED = -1,
								_UNITY			 = 1,
								_NULL				 = 0;

//MOTOR SPEEDS BY OPERATION
const unsigned short int 	_ORIENT_ROT_SPD			= 10,	//motor pwr used to rotate base when reorienting cube
													_FACE_ROT_SPD				= 10,	//motor pwr used to rotate base when holding cube and rotating only the bottom face
													_SCAN_ROT_SPD				= 10,	//motor pwr used to rotate base when scanning the colours of the squares on cube faces
													_SCAN_ARM_MOVE_SPD	= -30,	//motor pwr used to move colour sensor armature into/out of position for scanning
													_ARMATURE_MOVE_SPD	= 35,	//motor pwr used to move primary armature up/down onto/off of cube (for flipping or holding)
													_ARMATURE_FLIP_SPD	= 35;	//motor pwr used to pull back with primary armature to execute cube flipping

//MOTOR ENCODER POSITION VALUES
const int _ARM_ENCODER_DOWN_POS		= 100,	 //motor encoder value when primary armature is in full DOWN position (holding cube)
					_ARM_ENCODER_UP_POS			= 0,	 //motor encoder value when primary armature is in full UP position (off of cube)
					//_ARM_ENCODER_NORMAL_POS	= 0,	 //motor encoder value when primary armature is in NORMAL position (can be moved on/off cube)
					_ARM_ENCODER_FLIP_POS		= 185,//motor encoder value when primary armature is fully RETRACTED (move motor to this position to flip cube)
					_SCANNING_ARM_RET_POS		= 0,	 //motor encoder value when colour sensor armature fully RETRACTED (clear of cube)
					_SCANNING_ARM_EXT_POS		= -780, //motor encoder value when colour sensor armature fully EXTENDED (in position for scanning)
		//FURTHER TESTING REQUIRED
					_BASE_UNIT_ANGLE				= 256;	//motor encoder value to rotate 90° accounting for gear reduction between motor and cube base (theo: 270, gear reduction 3:1)

//CUBE VIRTUAL MODEL UPDATE OPERATIONS
enum operation {
	_FLIP,																//these are used to tell updateCubeModel() what physical operations the cube has undergone
	_ROTATE_CUBE_RIGHT,
	_ROTATE_CUBE_LEFT,
	_ROTATE_FACE_RIGHT,
	_ROTATE_FACE_LEFT
};

//CUBE SQUARE COLOUR ENCODING
enum cubeColour {
	_RED,																	//used as return values by findColour() to specify which colour is being detected by sensor
	_ORANGE,
	_YELLOW,
	_GREEN,
	_BLUE,
	_WHITE
};


/******************************************************************************/
//														FUNCTION PROTOTYPES
/******************************************************************************/

//BASIC
bool setup(void);																																//sets up all sensor ports and modes
void wait(const float time);																				//waits <time> seconds
void waitButton(const unsigned short int btn,																		//waits for a button press or time elapsed
								const unsigned long int time=_NULL);
long int readSensor(const unsigned short int sensor,														//reads from any sensor (including encoders)
										const unsigned short int mode=_UNSPECIFIED,
										unsigned long int& r=0,
										unsigned long int& g=0,
										unsigned long int& b=0,
										const bool is_motor = false);
unsigned short int findColour(void);																						//maps RGB values from colour sensor to encoding


			//MENUS
			void dispPosInstructions(void);																						//displays instructions to position cube properly
			void three21(void);																												//animates a 3...2...1...GO! countdown on display
			unsigned short int menuNav(const unsigned short int menu);								//indicates what menu line the user is on &
																																											//handles the user's choice

//MECHANICAL MOVEMENT
bool pmc_throttle(const unsigned short int curr_pwr, 														//pmc_throttle() = "Precision Motor Controller Throttle"
									const unsigned short int tgt_motor, 														//this function controls down-throttling of motor
									const unsigned int enc_target,																	//power as encoder val approaches target
									const short int dir=_UNITY);
bool movColour(const short int dir);																		//extends / retracts the colour sensor armature
bool pullArmature(void);																												//retracts the armature in order to flip cube
bool rotateArmature(const unsigned short int dir);															//raises / lowers the primary armature (holds / releases cube)
bool flipRecover(const unsigned short int dir);																	//moves primary armature to _UP or _DOWN position (<dir>)
bool rotateCube(const unsigned short int dir,																		//rotates the base 90Â°, <num> times in <dir> direction
								const unsigned short int num=_UNITY);																	//ensuring primary armature is not holding cube
bool flipCube(const unsigned short int num=_UNITY);															//flips cube (primary arm) drops P-Arm IF REQ'D
/*
bool rotateFace(const unsigned short int dir,																		//executes face rotation on cube, drops P-Arm IF REQ'D
								const unsigned short int num=_UNITY);
*/


//SCANNING
bool scanFace(unsigned short int* pattern_map);																	//rotates the base and scans the square colours into DS
bool checkSum(const unsigned short int* pattern_map);														//checks that the scan executed correctly (DS good vals)
bool scanCube(unsigned short int* pattern_map);																	//scans all faces on the entire cube

//LOGISTICS
bool updateCubeModel(unsigned short int* pattern_map,														//updates the datastructure holding square colours
										 const unsigned short int* operation);														//to match physical moves executed on cube
bool findShortestPath(const unsigned short int* pattern_map,										//determines least number of moves to get cube in correct
											const unsigned short int* targetDnR,														//orientation for next face manipulation
											unsigned short int* instructions);
bool executeMove(const unsigned short int* pattern_map,													//executes series of instructions to perform physical
								 const unsigned short int* targetDnR,																//manipulations on orientation or faces
								 const unsigned short int* faceRotate);

//STATIC ALGORITHM / SOLVER
/*
bool staticSolveCube(void);																											//hard-coded algorithm to solve a specific cube configuration.
*/

//DYNAMIC ALGORITHMS / SOLVER
bool solveCubeVirtual(const unsigned short int* pattern_map,										//determines all the moves necessary (or next move ~)
											unsigned short int* solutionSteps);															//required to solve cube and translates to instructions.
bool solveCubePhysical(void);																										//actually executes all steps necessary to solve physical
																																											//cube


/******************************************************************************/
//										 PRELIMINARY TASK MAIN() [TESTING]
/******************************************************************************/

task main() {

displayString(2,"Please select the level of difficulty for solving the static cube");
displayString(4,"Press the up button for the high level solve (fast speed solve)");
displayString(6,"Press the centre button for the medium level solve (medium speed solve)");
displayString(8,"Press the down button for the easy level solve (slow speed solve)");

if (getButtonPress(buttonUp))
{}
else if (getButtonPress(buttonEnter))
{}
else if (getButtonPress(buttonDown))
{}
else
{displayString(2,"Please select a valid button");
displayString(4,"Press the up button for the high level solve (fast speed solve)");
displayString(6,"Press the centre button for the medium level solve (medium speed solve)");
displayString(8,"Press the down button for the easy level solve (slow speed solve)");
	}
}








/******************************************************************************/
//														FUNCTION DEFINITIONS
/******************************************************************************/

/*sets up all sensor ports (types) and modes (colour / gyro)
Returns TRUE if setup completed successfully, FALSE otherwise*/
bool setup(void) {
	//SENSORS

/*
	SensorType[_BASE_GYRO] = sensorEV3_Gyro;
	wait(0.05);
	SensorMode[_BASE_GYRO] = _CALIBRATION;
	wait(0.1);
	SensorMode[_BASE_GYRO] = _RATEandANGLE;
	wait(0.05);

	SensorType[_ARM_GYRO] = sensorEV3_Gyro;
	wait(0.05);
	SensorMode[_ARM_GYRO] = _CALIBRATION;
	wait(0.1);
	SensorMode[_ARM_GYRO] = _RATEandANGLE;
	wait(0.05);

	SensorType[_SCAN_GYRO] = sensorEV3_Gyro;
	wait(0.05);
	SensorMode[_SCAN_GYRO] = _CALIBRATION;
	wait(0.1);
	SensorMode[_SCAN_GYRO] = _RATEandANGLE;
	wait(0.05);
*/
	SensorType[_LIGHT] = sensorEV3_Color;
	SensorMode[_LIGHT] = _COLOUR;

																														//SH - ADDED:
	resetMotorEncoder(_BASE_MOTOR);
	resetMotorEncoder(_SCANNING_ARM_MOTOR);
	resetMotorEncoder(_ARMATURE_MOTOR);

	//ENCODER and TIME?
	time1[T1] = 0																							//SH - I DON'T THINK THIS IS NECESSARY SINCE ANY TIME WE WANT TO USE A TIMER WILL PROBABLY HAVE TO RESET RIGHT THEN.

	return TRUE;

}

/*Pauses program execution for <time> seconds.*/
void wait(const float time) {
	wait1Msec((int)(time * 1000));
	return;
}

/*waits for a user to press (and release) <btn>
<time> is an optional argument.
if <time> is specified, will wait EITHER for the user to press <btn>
OR for <time> seconds (whichever comes first), then exits.*/
void waitButton(const unsigned short int btn,
								const unsigned long int time) {

	time1[T1] = 0;

	if (time==_NULL)
		{
			while(!getButtonPress(btn))
			{}
			while(getButtonPress(btn))
			{}
		}
	else
		{
		while(getButtonPress(btn)||time1[T1]<time)
		{}
		}

}

/*This function should return the current sensor reading of ANY sensor. This
includes the colour sensor, gyro and motor encoders, depending on whether
_TOUCH, _DISTANCE, _COLOUR, _GYRO, _BASE_MOTOR, _SCANNING_ARM_MOTOR or
_ARMATURE_MOTOR are passed as the first argument. The second argument is
optional, if the function is called on _GYRO or _COLOUR without specifying
the mode, the gyro should return the current _ANGLE reading and the colour
sensor should return the whatever mode reading we are using to scan the cube.*/
long int readSensor(const unsigned short int sensor,
										const unsigned short int mode,													//SH - THIS IS MY FAULT, BUT I'M JUST NOW (07/12) REALIZING THIS FUNCTION MAY NOT WORK
										unsigned long int& r,																						//MAINLY BECAUSE THE VALUES FOR motorA/B/C/D ARE PROBABLY 0/1/2/3 -- WHICH ARE THE SAME
										unsigned long int& g,																						//AS s1/s2/s3/s4 -- SO THERE NEEDS TO BE A BOOLEAN FLAG IN THE PARAMETERS THAT ALLOWS US
										unsigned long int& b,
										const bool is_motor) {																			//TO SPECIFY THAT WE'RE ASKING FOR A SENSOR OR A MOTOR ENCODER VALUE (AGAIN, MY FAULT)

	if (is_motor) {
		return nMotorEncoder[sensor];
	}
	else {
		/*
		if (sensor == _GYRO)
		{
			if mode == _RATE
			{
				return getGyroRate(sensor);
			}
			else
			{
				return getGyroDegrees(sensor);
			}
		}
		*/
		if (sensor ==_LIGHT)
		{
			if (mode ==_AMBIENT)
			{
				SensorMode[_LIGHT] = modeEV3Color_Ambient;
				return getColorAmbient(sensor);
			}
			else if (mode ==_REFLECTED)
			{
				return getColorReflected(sensor);
			}
			else if (mode == _COLOUR) {
				SensorMode[_LIGHT] = modeEV3Color_Color;
				return SensorValue[_LIGHT];
			}
			else
			{
				SensorMode[_LIGHT] = modeEV3Color_Color; //might be supposed to be RBGraw mode from autocomplete
				return getColorRGB(_Light, r, g, b);
			}
		}
		/*
		else if (sensor == _TOUCH)
		{
			return SensorValue(sensor);
		}
		else if (sensor == _DISTANCE)
		{
			return getDistanceValue(sensor);
		}
		*/
		else {
			eraseDisplay();
			displayBigTextLine(5, "   Unhandled   ");
			displayBigTextLine(7, "   Exception   ");
			displayBigTextLine(9, "->readSensor()");
	    wait(4);
	    return -1;
		}
	}
}

/*This function reads from the colour sensor (probably in RGB mode) and
determines what colour the sensor is seeing based on the detected RGB values
the sensor reports seeing.
Returns the encoded colour value which was detected. (_RED, _ORANGE, _YELLOW,
_GREEN, _BLUE or _WHITE)*/
unsigned short int findColour(void) {
    long int r=0, g=0, b=0;
    getColorRGB(_LIGHT, r, g, b);

    // Thresholds for different colors
    if (r > 200 && g < 100 && b < 100) {
        return _RED;
    } else if (r > 200 && g > 100 && g < 200 && b < 100) {
        return _ORANGE;
    } else if (r > 200 && g > 200 && b < 100) {
        return _YELLOW;
    } else if (r < 100 && g > 200 && b < 100) {
        return _GREEN;
    } else if (r < 100 && g < 100 && b > 200) {
        return _BLUE;
    } else if (r > 200 && g > 200 && b > 200) {
        return _WHITE;
    } else {
        // Return a default value if no match is found
        return _WHITE;
    }
}


//POLISH / MENU TYPE STUFF -- SKIP INITIALLY
		/*function which displays instructions to position cube correctly
		(either for scanning or for single configuration solving)*/
		void dispPosInstructions(void) {

		}

		/*function displays 3.. 2.. 1.. GO! On the display (to start racing user)*/
		void three21(void) {

		}

		/*This function displays something which indicates to the user which LINE
		(menu item) they are on. Must allow them to move up and down menu items
		(UP/DOWN buttons) and select the currently highlighted menu item by pressing
		ENTER.
		Returns the number corresponding to the lineNo the user had highlighted at
		the time they pressed ENTER.*/
		unsigned short int menuNav(const unsigned short int menu) {										//***Parameter may need to be changed***

		}





/*This function controls the motor throttle of any motor as a function of how
close the motor encoder value is to the target value, throttling back using
an exponential decay algorithm as a close approach to the target angle of
rotation of the motor is obtained. The intent is to allow us to set the motor
powers as high as possible (so the robot solves the cube as fast as possible)
without sacrificing accuracy / precision of the movement.*/
/******************************************************************************/
bool pmc_throttle(const unsigned short int curr_pwr,
									const unsigned short int tgt_motor,
									const unsigned int enc_target,
									const short int dir) {
	static unsigned short int PWR;
	static unsigned int TGT, b;
	static float exp_term1, exp_term2, coF_prime, final_term1, normalizer;
	static bool output = false;

	static const float EXP09 = exp(0.09);

	if (PWR != curr_pwr) {
		b = _PMC_DECAY_FACTOR;        //***INSERT FUNCTION ADJUSTMENT TO DECAY FACTOR AS FUNCTION OF MOTOR PWR HERE***
		exp_term1 = pow(EXP09, b);
		exp_term2 = exp((float)b / 1000);
		coF_prime = 100 / (exp_term1 - 1);
		final_term1 = coF_prime * exp_term1;
		PWR = curr_pwr;
	}
	if (TGT != enc_target) {
		if (enc_target != 90) {
			normalizer = (float)90 / enc_target;
		}
		else {
			normalizer = _UNITY;
		}
		TGT = enc_target;
	}
	output = (bool)motor[tgt_motor] = dir * round(final_term1 - pow(exp_term2,
																			normalizer * nMotorEncoder[tgt_motor]));
	return output;
}






/******************************************************************************/
/*This function extends the colour sensor into or out of position (extends or
retracts) depending on whether _FWD or _REV is passed as the argument.
(NOTE: _FWD should extend the sensor to scanning position)
Returns TRUE if the sensor moved to the correct position, FALSE otherwise.*/
bool movColour(const short int dir) {
	//DEMO FUNCTION
	long int pitch = 0;
	clearTimer(T4);
	if (_PMC_ACTIVE) {
		while (pmc_throttle(_SCAN_ARM_MOVE_SPD, _SCANNING_ARM_MOTOR,
												_SCANNING_ARM_EXT_POS, dir)
					&& time1[T4] < _FAILSAFE_TIMEOUT);																																															//SH - PRETTY SURE I WAS THINKING ABOUT THIS AND DECIDED I MESSED SOMETHING UP
	}
	else {
		motor[_SCANNING_ARM_MOTOR] = dir * _SCAN_ARM_MOVE_SPD;
		while (readSensor(_SCANNING_ARM_MOTOR, _UNSPECIFIED, pitch, pitch, pitch,
					 true) > _SCANNING_ARM_EXT_POS
					&& time1[T4] < _FAILSAFE_TIMEOUT) {}
		motor[_SCANNING_ARM_MOTOR] = 0;
	}																																	//   - NEED TO COME BACK AND LOOK AT THIS LATER TO MAKE SURE.
	if (abs(readSensor(_SCANNING_ARM_MOTOR, _UNSPECIFIED, pitch, pitch, pitch,
					true) - _SCANNING_ARM_EXT_POS) > 3) {																//   - MAYBE ESTABLISHING CONSTANT FOR TIMEOUT VALUE? CAN'T REMEMBER.
		eraseDisplay();
		displayBigTextLine(5, "   Unhandled   ");
		displayBigTextLine(7, "   Exception   ");
		displayBigTextLine(9, "->movColour()");
          wait(4);
		return false;
	}
	return true;
}

/*This function simply pulls the armature backward in order to effect
flipping the cube. It assumes the primary armature is already in position
to perform the flip. Should execute at _ARMATURE_FLIP_SPD.*/
/******************************************************************************/
bool pullArmature(void) {
	if (_PMC_ACTIVE) {
		clearTimer(T1);
		while(pmc_throttle(_ARMATURE_FLIP_SPD, _ARMATURE_MOTOR,
											 _ARM_ENCODER_FLIP_POS)
				&& time1[T1] < _FAILSAFE_TIMEOUT);
	}
	else {
		motor[_ARMATURE_MOTOR] = _ARMATURE_FLIP_SPD;
		while(nMotorEncoder[_ARMATURE_MOTOR] < _ARM_ENCODER_FLIP_POS) {}						//SH - THIS SHOULD BE readSensor() not nMotorEncoder[] directly.
	}																																						//SH - ADDED:
	if (_PMC_ACTIVE) {
		clearTimer(T1);
		while(pmc_throttle(_ARMATURE_FLIP_SPD, _ARMATURE_MOTOR,
											 _ARM_ENCODER_DOWN_POS, _REV)
				&& time1[T1] < _FAILSAFE_TIMEOUT);
	}
	else {
		motor[_ARMATURE_MOTOR] = -_ARMATURE_FLIP_SPD;
		while(nMotorEncoder[_ARMATURE_MOTOR] > _ARM_ENCODER_DOWN_POS) {}						//SH - THIS SHOULD BE readSensor() not nMotorEncoder[] directly.
	}
	motor[_ARMATURE_MOTOR] = 0;
	long int pitch = 0;
	return abs(readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch,
												true)
					- _ARM_ENCODER_FLIP_POS) < 4;
}

/*Like movColour(), this function lowers the "flipping arm" or raises
the "flipping arm" depending on whether _DOWN or _UP is passed in as the
argument. (NOTE: _DOWN should lower the arm onto the cube). The function
assumes the armature is in the neutral, raised position and not in the raised
and pulled back position where the armature ends up after running
pullArmature(). Should execute at _ARMATURE_MOVE_SPD.
Returns TRUE if the armature moved to the correct position, FALSE otherwise.*/
/******************************************************************************/
bool rotateArmature(const unsigned short int dir) {
  if (_PMC_ACTIVE && (dir == _DOWN || dir == _UP)) {
  	clearTimer(T1);
		while(pmc_throttle(_ARMATURE_MOVE_SPD, _ARMATURE_MOTOR,
				  						 _ARM_ENCODER_DOWN_POS, dir)
				&& time1[T1] < _FAILSAFE_TIMEOUT);
	}
	else {
		if (dir == _DOWN)
	  {
	    // Lower the armature
    	motor[_ARMATURE_MOTOR]=_ARMATURE_MOVE_SPD;
			while(nMotorEncoder[_ARMATURE_MOTOR] < _ARM_ENCODER_DOWN_POS)
			{}
	  }
	  else if (dir == _UP)
		{
	    // Raise the armature
			motor[_ARMATURE_MOTOR] = -_ARMATURE_MOVE_SPD;
			while(nMotorEncoder[_ARMATURE_MOTOR] > _ARM_ENCODER_UP_POS)
			{}
	  }
	  else {
	    // Invalid direction
	    return false;
	  }
	}
	motor[_ARMATURE_MOTOR] = 0;
  return true;
}



/*Function which recovers the armature to either a neutral (_UP) position after
it executes a flip (pullArmature()) of the cube (from the position the armature
is in after the flip completes) or an extended (holding the cube (_DOWN))
position (from the position the armature is in after the flip completes),
depending on whether _DOWN or _UP is passed as the <dir> argument.
Returns TRUE if the armature moved to the correct position, FALSE otherwise.*/
/******************************************************************************/
bool flipRecover(const unsigned short int dir)
{
	if (_PMC_ACTIVE) {
		clearTimer(T1);
		while(pmc_throttle(_ARMATURE_MOVE_SPD, _ARMATURE_MOTOR,
											 _ARM_ENCODER_DOWN_POS, _REV)
				&& time1[T1] < _FAILSAFE_TIMEOUT);
	}
	else {
		motor[_ARMATURE_MOTOR] = -(_ARMATURE_MOVE_SPD);
		while(nMotorEncoder[_ARMATURE_MOTOR] > _ARM_ENCODER_DOWN_POS) {}
		motor[_ARMATURE_MOTOR] = 0;
	}
	if (dir == _UP)
	{
		if(!rotateArmature(_UP))																												//SH - MISSING CLOSING PARENTHESIS
		return FALSE;
		/*THIS IS FINE BUT REDUNDANT, I WOULD WRITE:
		return rotateArmature(_UP);
		*/
	}
	else if (dir == _DOWN) {																													//SH - MISSING BRACES TO SCOPE OUTER ELSE IF
		//motor[_ARMATURE_MOTOR] = 0;																											<--- I WOULD TURN THE MOTOR OFF BEFORE THE CHECK SINCE rotateArmature() WILL TURN IT BACK ON
		long int pitch = 0;																															//unneccessary scope removed
		if (readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch, true)
			!= _ARM_ENCODER_DOWN_POS)	//IF THE DIRECTION IS UP AND WE WANT THE MOTOR TO TURN OFF AS SOON AS THE WHILE CONDITION IS MET.
		//MIGHT WANT TO MAKE THIS CONDITION A RANGE LESS THAN 3 DEGREES
		//eg. if (abs(readSensor(_ARMATURE_MOTOR) - _ARM_ENCODER_DOWN_POR) > 3)
		return FALSE;

	}
	return TRUE;
	/*SH - I WOULD WRITE THIS WHOLE FUNCTION LIKE THIS:
	if (abs(readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, 0, 0, 0, true) - _ARM_ENCODER_FLIP_POS) > 3) {
		motor[_ARMATURE_MOTOR] = -_ARMATURE_MOVE_SPD;
		while (readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, 0, 0, 0, true) > _ARM_ENCODER_DOWN_POS) {}
		motor[_ARMATURE_MOTOR] = 0;
		if (dir == _UP) { return rotateArmature(_UP); }
		else if (abs(readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, 0, 0, 0, true)- _ARM_ENCODER_DOWN_POS) > 3) {
			return false;
		}
		else {
			return true;
		}
	}
	else {
		return rotateArmature(dir);
	}
	*/
}

/*This function checks the position of the primary armature and calls functions
like flipRecover() or rotateArmature() IF REQUIRED to make sure the armature
is not in position to hold the cube (which would rotate a face). The function
then rotates the "holder" (base) on which the cube is resting 90Â° <num> times
in <dir> direction. If _RIGHT is passed as <dir>, the cube should rotate
clockwise (_CW can also be used). If _LEFT is passed as <dir>, it should rotate
counter-clockwise (_CCW can also be used). The rotation should occur at
_ORIENT_ROT_SPD.
Returns TRUE if the rotation executed successfully, FALSE otherwise.*/
/******************************************************************************/
bool rotateCube(const unsigned short int dir,
								const unsigned short int num) {
	/*
	REMOVED BECAUSE ARMATURE CAN'T HOLD RETRACTED POSITION AFTER FLIPPING

	if(readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, 0, 0, 0, true) == _ARM_ENCODER_FLIP_POS)
	{
		if(!flipRecover(_UP))
		return FALSE;
	}
	*/
	long int pitch = 0;
	if(abs(readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch, true)
		 - _ARM_ENCODER_DOWN_POS) < 4)
	{
		if(!rotateArmature(_UP))
		return FALSE;
	//could just be "return rotateArmature(_UP);"
	}
	//readSensor(_BASE_MOTOR, _UNSPECIFIED, 0, 0, 0, true) = 0;								<--- ????

	if (_PMC_ACTIVE) {
		clearTimer(T1);
		while(pmc_throttle(_ORIENT_ROT_SPD, _BASE_MOTOR, _BASE_UNIT_ANGLE * num,
											 dir) && time1[T1] < _FAILSAFE_TIMEOUT);
	}
	else {
		if(dir == _RIGHT){																																				//SH - RIGHT AND LEFT IS FINE, IT'S WHAT I PUT THEM IN THE CONSTANTS FOR
			motor[_BASE_MOTOR] = _CW*_ORIENT_ROT_SPD;																										//   - BUT IT MIGHT BE LESS AMBIGUOUS TO READ IF WE JUST USE _CW AND _CCW
			while(readSensor(_BASE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch, true)
						< (_CW*_BASE_UNIT_ANGLE*num))
			{}
		}
		else if(dir == _LEFT){
			motor[_BASE_MOTOR] = _CCW*_ORIENT_ROT_SPD;
			while(readSensor(_BASE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch, true)
						> (_CCW*_BASE_UNIT_ANGLE*num))
			{}
		}
	}
	motor[_BASE_MOTOR] = 0;
	/*THIS WHOLE THING (THE TWO ELSE-IFS ABOVE) COULD JUST BE:
	motor[_BASE_MOTOR] = dir * _ORIENT_ROT_SPD;
	while (abs(readSensor(_BASE_MOTOR, _UNSPECIFIED, 0, 0, 0, true) - num * _BASE_UNIT_ANGLE) > 3) {}
	motor[_BASE_MOTOR] = 0;
	(there needs to be a check afterwards to make sure rotation resulted in the base being in the
	correct position -- which determines if this function returns true or false, but otherwise...)
	*/
	return TRUE;
}



/*This function checks the current position of the primary armature and
executes functions like flipRecover() or rotateArmature() to move the
primary armature into the correct position as required, it then will use
pullArmature() to flip the cube.
It repositions itself onto the cube and flips the cube a second time if
<num> == 2.
NOTE: It does NOT automatically recover after the final flip execution.
Returns TRUE if all operations completed successfully, FALSE otherwise.*/
bool flipCube(const unsigned short int num) {
	long int pitch = 0;
	if(readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch, true) 												//SH - I REWROTE THIS FUNCTION BECAUSE OF THE CHANGE WITH NOT USING flipRecover()
		== _ARM_ENCODER_UP_POS){
		if(!rotateArmature(_DOWN))
		return FALSE;
	}
	for (unsigned int i = 0; i < num; i++) {
		if (!pullArmature())
		return FALSE;
	}
	return TRUE;
}


/*Checks the current position of the primary armature and executes
functions like flipRecover() or rotateArmature() to ensure the primary
armature is in the correct position, then rotates the bottom row of the
cube 90Â° or 180Â° (depending on if <num> == 1 or <num> == 2) to the _LEFT or
_RIGHT (<dir>) at _FACE_ROT_SPD. Does NOT move armature back to nominal
(retracted / _UP) position once the rotation has completed.
Returns TRUE if all operations completed successfully, FALSE otherwise.*/

bool rotateFace(const unsigned short int dir,
								const unsigned short int num) {
	long int pitch = 0;
	if(readSensor(_ARMATURE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch, true) 											//SH - REWROTE THIS FUNCTION FOR TIME CONSTRAINT REASONS
		!= _ARM_ENCODER_DOWN_POS) {
		if (!rotateArmature(_DOWN))
	  return FALSE;
	}

	//SH - I EXPECT AN ADJUSTMENT ALGORITHM (IN CONJUNCTION WITH A GLOBAL ADJUSTMENT FOR TAKING UP THE SLOP) TO NEED TO BE INTRODUCED HERE


	if (_PMC_ACTIVE) {
		clearTimer(T1);
		while (pmc_throttle(_FACE_ROT_SPD, _BASE_MOTOR, _BASE_UNIT_ANGLE * num,
					 							dir) && time1[T1] < _FAILSAFE_TIMEOUT);
	}
	else {
		motor[_BASE_MOTOR] = dir * _FACE_ROT_SPD;
		while (abs(readSensor(_BASE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch,
							 						true) - num * _BASE_UNIT_ANGLE) > 3) {}
	}
	motor[_BASE_MOTOR] = 0;

	//SH - SAME ALGORITHMIC INSERTION, BUT REVERSING THE ADJUSTMENT SHOULD BE INSERTED HERE


	//this may need to be adjusted
	if (abs(readSensor(_BASE_MOTOR, _UNSPECIFIED, pitch, pitch, pitch, true)
					- num * _BASE_UNIT_ANGLE) > 3)
	return FALSE;
	return TRUE;
}


		//THESE FUNCTIONS ALL PERTAIN TO DYNAMIC SOLVING (ANY CUBE CONFIGURATION) -- SKIP FOR NOW
				/*This function coordinates the rotation of the cube 360Â° with the scanning
				of individual coloured squares on the cube face, encodes the read values
				and writes the appropriate value to the appropriate index in the <pattern_map>
				matrix.
				Returns TRUE if all operations completed successfully, FALSE otherwise.*/
				bool scanFace(unsigned short int* pattern_map) {

				}

				/*The function adds up the encoded colour values contained in <pattern_map>
				and checks they add to the correct amount.*/
				bool checkSum(const unsigned short int* pattern_map) {

				}

				/*this function needs to follow the full cube-scanning algorithm:
				1.	Extends colour sensor
				2.	Rotates cube 360Â° to scan corners / sides of top face
				3.	Retracts Colour sensor
				4.	Flips cube 1x
				5.	LOOP TO 1 (REPEAT 4x)

				-	Rotate Cube 90Â° (right, let's say)
				-	Flip 1x
				-	Extend colour sensor
				-	Rotate 360Â° (scan)
				-	Retract colour sensor
				-	Flip 2x
				-	Extend Colour sensor
				-	Rotate 360Â° (scan)
				-	Retract Colour sensor
				-	Flip 1x
						"	Rotate Cube 90Â° (left, if the first step was right ' this should put
						cube back in "default" orientation)
				Making sure to scan each square into the appropriate array index as it does.
				Verifies data contents of <pattern_map> after scanning is complete (should
				contain 9 of each colour square). Should call scanFace() repeatedly to
				actually execute scanning the cube.
				Returns TRUE if ALL operations completed successfully, FALSE otherwise.*/
				bool scanCube(unsigned short int* pattern_map) {

				}

				/*Needs to update the cube's virtual <pattern_map> to keep track of the cube's
				orientation. <operation> can be any of _FLIP, _ROTATE_CUBE_RIGHT,
				_ROTATE_CUBE_LEFT, _ROTATE_FACE_RIGHT or _ROTATE_FACE_LEFT. <operation> is
				passed as an array of operations, in the order in which they are being / will
				be performed.
				Should check to ensure data in <pattern_map> is at least still valid if nothing
				else.
				Returns TRUE if operation completed successfully, FALSE otherwise.*/
				bool updateCubeModel(unsigned short int* pattern_map,
														 const unsigned short int* operation) {											//this may not be necessary (READ INDEX ADJUSTMENT)
																																													//could use this to update read offsets (above)
				}

				/*This function should accept the current orientation (<pattern_map>) of the
				cube, along with a 1x2 array <targetDnR> of the form: [<DOWN>, <RIGHT>], where
				<DOWN> and <RIGHT> are the centre square colours of the faces which should be
				facing down and right after the cube has been physically manipulated. The
				function will determine the shortest number of moves to achieve that
				orientation from the current position and then encode those movements into
				an external array passed by reference into the function as <instructions>.
				//EXTERNAL ARRAY SHOULD HAVE THE FORM [<DIR_ROT1>, <NUM_ROTs1>, <NUM_FLIPS>]
				Where <DIR_ROT1> specifies the direction of cube rotation prior to flipping
				(_LEFT, _RIGHT or _NULL. _NULL IS USED IF NO ROTATION PRIOR TO FLIPPING REQd)
				<NUM_ROTs1> specifies 1 or 2 rotations before flipping.
				<NUM_FLIPS> specifies 1 or 2 flips to execute.
				Returns TRUE if all operations completed successfully, FALSE otherwise.*/
				bool findShortestPath(const unsigned short int* pattern_map,
															const unsigned short int* targetDnR,
															unsigned short int* instructions) {

				}

				/*This function should accept a matrix of the current cube orientation
				(<pattern_map>), as well as the target cube orientation <targetDnR> in the
				form of a 1x2 array [<DOWN>, <RIGHT>], where <DOWN> and <RIGHT> are the
				centre square colours of the faces which should be facing down and right
				after the cube has been physically manipulated. The final argument
				<faceRotate> is a 1x2 array of the form [<DIR>, <NUM>] where <DIR>
				specifies the direction of the face rotation (_LEFT or _RIGHT) and <NUM>
				specifies the number of 90Â° rotations to perform in that direction.
				The function will declare an array called "instructions" which MUST HAVE
				THE FORM [<DIR_ROT1>, <NUM_ROTs1>, <NUM_FLIPS>]
				Where <DIR_ROT1> specifies the direction of cube rotation prior to flipping
				(_LEFT, _RIGHT or _NULL. _NULL IS USED IF NO ROTATION PRIOR TO FLIPPING REQd)
				<NUM_ROTs1> specifies 1 or 2 rotations before flipping.
				<NUM_FLIPS> specifies 1 or 2 flips to execute.
				This function calls findShortestPath() with the appropriate arguments
				to determine what moves to execute and then executes the orienting moves
				(from "instructions" variable) then performs the rotation specified by
				<faceRotate> to complete the manipulation step to the cube.
				Returns TRUE if all operations completed successfully, FALSE otherwise.*/
				bool executeMove(const unsigned short int* pattern_map,
												 const unsigned short int* targetDnR,
												 const unsigned short int* faceRotate) {

				}

/*This function is a container hard-coding the (static) steps to solve
a cube in a specific configuration. It cannot solve any other configurations
and, as such, requires no scanning to determine / map the current
configuration of the cube. This function should only call the following
functions: flipCube(), rotateFace() and rotateCube().
Returns TRUE if all operations complete successfully, FALSE otherwise.*/
bool staticSolveCube(void) {
	//SH - NOTE: I NONE OF US CAN REALLY "CHECK" THIS FUNCTION, SINCE WE DON'T KNOW THE "KNOWN" CONFIGURATION YOU (EITAN) ARE USING TO TRY TO WRITE A SINGLE SOLVE ALGORITHM. AS SUCH I HAVE ONLY NOTED GENERAL COMMENTS / LOGICAL ERRORS ETC.
  bool outPut = TRUE;																					//SH - I CHANGED ALL NUMERIC VALUES USED FOR THE BOOLEAN "outPut" TO "TRUE" OR "FALSE" FOR READABILITY.

  for(int i = 0; i < 4; i++)
  {
    if(!rotateFace(_LEFT, 180))
    outPut = FALSE;
		if(flipCube(1))																					//think you meant if(!flipCube(1))					SH - NOTE: I THINK THERE IS A LOGICAL ERROR HERE, IF THE ROBOT FAILED TO rotateFace() IN THE PREVIOUS STEP, WE PROBABLY DON'T WANT IT TRYING TO FLIP THE CUBE
	  outPut = FALSE;																					//think this should probably be TRUE										(SHOULD PROBABLY JUST BE "RETURN FALSE" INSTEAD OF TRACKING "OUTPUT" IN A VARIABLE, YOU CAN DO IT YOUR WAY, BUT THEN YOU NEED TO CHECK if(outPut)
  	//could just write this as:																																					//						BEFORE EACH STEP AFTER THE FIRST (TO MAKE SURE IT ONLY ATTEMPTS THAT STEP IF THE PREVIOUS STEPS DIDN'T FAIL.
    //outPut = rotateFace(_LEFT, 180);
    //if (outPut) { outPut = flipCube(1); }
  }

  if(rotateCube(_RIGHT, 1))																		//think you meant if(!rotateCube(_RIGHT, 1))
    outPut = FALSE;

  for(int i = 0; i < 4; i++)
  {
	if(flipCube(1))																							//same thing again
  outPut = FALSE;
  if(rotateFace(_LEFT, 180))																	//same thing again
  outPut = FALSE;
  }

  if(rotateCube(_RIGHT, 2))																		//same thing
    outPut = FALSE;
  if(flipCube(2))
    outPut = FALSE;
  if(rotateFace(_RIGHT,90))																		//same thing
    outPut = FALSE;

  if(outPut == FALSE)
    return FALSE;
  else
    return TRUE;
  /*SH - THIS COULD JUST BE: "return outPut;"
  				AS WRITTEN, I WOULD JUST WRITE:
  				if (outPut)
  				return TRUE;
  				return FALSE;
  */
}

		//MORE FUNCTIONS PERTAINING TO DYNAMIC CUBE SOLVING (SKIP ON FIRST PASS)
					/*This function translates the CFOP speed cubing algorithm into a series
					of instructions to the various actuators on the device to execute the
					appropriate steps required to solve the cube, and generates those steps
					dynamically depending on the cube configuration described by <pattern_map>.
					The function either iterates after each executed move to find the new
					"best move" or works with the pattern_map and dynamically generates an
					array of fixed width but unknown length (each row represents the move
					instructions for a single step of the solution) and points <solutionSteps>
					to the memory address of this array.
					Returns TRUE if all operations executed successfully, FALSE otherwise.*/
					bool solveCubeVirtual(const unsigned short int* pattern_map,									//may be able to completely solve the cube in advance
																unsigned short int* solutionSteps) {										//may have to simply identify "best move" and move -- then
																																												//re-run this function at each step until solved
					}																																							//(will have to run concurrently with solveCubePhysical())

					/*Function creates a 6x3x3 array called "pattern_map" which can store
					the scanned colour values of each square on the cube during the scanCube()
					function execution. The purpose of this function is to completely solve
					the cube by executing the instructions determined by solveCubeVirtual().
					Returns TRUE if all operations executed successfully, FALSE otherwise.*/
					bool solveCubePhysical(void) {

					}


/******************************************************************************/
