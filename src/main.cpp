#include "main.h"
#include <cmath>

// Catapult Variables
int shootMode = 1;
int cataRotations = 0;
float cataPos = cata.get_position();
float cataTension = cata.get_torque();

// Drive pid
float kP = 20;
float kI = 0;
float kD = 10;

double LPos;
double RPos;

float Lerror = 0;
float Rerror = 0;

float LtotalError = 0;
float RtotalError = 0;

float Lderivative = 0;
float Rderivative = 0;

float LprevError = 0;
float RprevError = 0;

float integralBound = 20;

int counts = 0;
float LdesiredValue = 0;
float RdesiredValue = 0;

void drivePID(float Ldistance, float Rdistance){
	LdesiredValue = Ldistance/12.56*360;
	RdesiredValue = Rdistance/12.56*360;
	DLF.tare_position();
	DLB.tare_position();
	DRF.tare_position();
	DRB.tare_position();

	while(true){
		// Gets position of both motors
		LPos = (DLF.get_position() + DLB.get_position())/2;
		RPos = (DRF.get_position() + DRB.get_position())/2;

		// Proportional
		Lerror = LdesiredValue-LPos;
		Rerror = RdesiredValue-RPos;
		// Integral
		if(abs(Lerror) < integralBound){
			LtotalError += Lerror;
		}
		if(abs(Rerror) < integralBound){
			RtotalError += Rerror;
		}
		// Derivative
		Lderivative = Lerror - LprevError;
		Rderivative = Rerror - RprevError;
		
		// Gain
		double leftGain = Lerror * kP + Lderivative * kD + LtotalError*kI;
		double rightGain = Rerror * kP + Rderivative * kD + RtotalError*kI;

		DL.move_voltage(leftGain);
		DR.move_voltage(rightGain);

		if(round(LPos) == round(LdesiredValue) && round(RPos) == round(RdesiredValue)){
			counts++;
			if(counts == 25){
				break;
			}
		}
		else{
			counts = 0;
		}

		// Other stuff
		LprevError = Lerror;
		RprevError = Rerror;
		pros::delay(20);
	}
}




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void waitUntil(bool arg){
    while(arg == false){
        pros::delay(5);
    }
}

bool cataActivated = false;
int cataVelocity = 60;

void catapult(){
	cata.tare_position();
	while(true){
		
		if(Master.get_digital(DIGITAL_A)){
			shootMode *= 1;
			waitUntil(!Master.get_digital(DIGITAL_A));
		}

		if(cataActivated){
			cata.move_velocity(cataVelocity);
		}
		else{
			if(shootMode == 1){
				if(cataPos>=(cataRotations)*360+215){
					cata.brake();
				}
				else{
					cata.move_velocity(30);
				}
			}
			else{
				if(cataPos>=(cataRotations)*360+200){
					cata.brake();
				}
				else{
					cata.move_velocity(30);
				}
			}
		}
		pros::delay(50);
	}
}
void display(){
	while(true){
		pros::lcd::set_text(1,"LDesired: " + std::to_string(LdesiredValue));
		pros::lcd::set_text(2,"RDesired: " + std::to_string(RdesiredValue));

		pros::lcd::set_text(3,"Left: " + std::to_string(LPos));
		pros::lcd::set_text(4,"Right: " + std::to_string(RPos));
		pros::lcd::set_text(5,"Counts: " + std::to_string(counts));
		if(shootMode == 1){
			Master.set_text(0,0,"Mode: Shoot");
		}
		if(shootMode == -1){
			Master.set_text(1,0,"Mode: Capture");
		}
		pros::delay(50);
	}
}

void initialize() {
	pros::lcd::initialize();
	cata.set_brake_mode(MOTOR_BRAKE_HOLD);
	cata.set_encoder_units(MOTOR_ENCODER_DEGREES);
	DLF.set_encoder_units(MOTOR_ENCODER_DEGREES);
	DLB.set_encoder_units(MOTOR_ENCODER_DEGREES);
	DRF.set_encoder_units(MOTOR_ENCODER_DEGREES);
	DRB.set_encoder_units(MOTOR_ENCODER_DEGREES);

	pros::Task braindisplay(display);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// PID TESTING
	drivePID(20,20);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	pros::Task taskCata(catapult);
	while(true){
		int x = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int y = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		
		int leftPower = x/2 + y;
		int rightPower = -x/2 + y;
		DL.move_velocity(leftPower*6);
		DR.move_velocity(rightPower*6);
		
		cataPos = int(round(cata.get_position()));
		cataRotations = floor(cata.get_position()/360);
		cataTension = cata.get_torque();

		if(Master.get_digital(DIGITAL_R2)){
			cataActivated = true;
		}
		else{
			cataActivated = false;
		}

		if(Master.get_digital(DIGITAL_L2)){
			intake.move_voltage(12000);
		}
		else if(Master.get_digital(DIGITAL_L1)){
			intake.move_voltage(-12000);
		}
		else{
			intake.move_voltage(0);
		}

		pros::delay(20);
	}
}
