#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

#include "pros/serial.h"
#include "pros/serial.hpp"
#include <sstream>	        //Include sstream for serial parsing
#include <vector>

//Prototypes for hidden vex functions to bypass PROS bug
extern "C" int32_t vexGenericSerialReceive( uint32_t index, uint8_t *buffer, int32_t length );
extern "C" void vexGenericSerialEnable(  uint32_t index, uint32_t nu );
extern "C" void vexGenericSerialBaudrate(  uint32_t index, uint32_t rate );
extern "C" int32_t vexGenericSerialTransmit( uint32_t index, uint8_t *buffer, int32_t length );

double headingValue = 0;
double distX = 0;
double distY = 0;
char msg[1] = {'R'};
char msg2[1] = {'T'};
char msg3[2] = {'W', 'B'};
bool readSerial = true;
bool imu_reset = false;

void serialRead(void* params) {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    // Start serial on desired port
    vexGenericSerialEnable(SERIALPORT - 1, 0);
    
    // Set BAUD rate
    vexGenericSerialBaudrate(SERIALPORT - 1, 115200);
    
    // Let VEX OS configure port
    pros::delay(10);
    pros::screen::set_pen(COLOR_BLUE);
    
    // Serial message format:
    // D[LIDAR DIST]I[IR DATA]A[GYRO ANGLE]E
    // Example Message:
    // D50.2I128A12.32E
    bool toggle = false;
    //pros::screen::print(TEXT_MEDIUM, 6, "TEST");
    
    imu_reset = true;
    while (true) {
        
        // Buffer to store serial data
        uint8_t buffer[256];
        int len = 256;
        
        // Get serial data
        int32_t nRead = vexGenericSerialReceive(SERIALPORT - 1, buffer, len);
        //master.print(0,0,"%d",nRead);
        // Now parse the data
        if(master.get_digital(DIGITAL_Y) || imu_reset == true){
            vexGenericSerialTransmit(SERIALPORT - 1, (uint8_t *)msg3, 2);
            distX = 0;
            distY = 0;
            headingValue = 0;
            readSerial = true;
            imu_reset = false;
            pros::delay(70);
        }
        
        if (nRead >= 0 && readSerial) {
            
            // Stream to put the characters in
            std::stringstream myStream("");
            std::stringstream myStream2("");
            std::stringstream myStream3("");
            bool recordAngle = false;
            bool recordOpticalX = false;
            bool recordOpticalY = false;
            
            // Go through characters
            for (int i = 0; i < nRead; i++) {
                // Get current char
                char thisDigit = (char)buffer[i];
                
                // If its special, then don't record the value
                if (thisDigit == 'D' || thisDigit == 'I' || thisDigit == 'A' || thisDigit == 'X' || thisDigit == 'C' || thisDigit == 'Y' || thisDigit == 'D'){
                    recordAngle = false;
                    recordOpticalX = false;
                    recordOpticalY = false;
                }
                
                // Finished recieving angle, so put into variable
                if (thisDigit == 'E') {
                    recordAngle = false;
                    myStream >> headingValue;
                    pros::screen::print(TEXT_MEDIUM, 1, "[External IMU Heading]");
                    pros::screen::print(TEXT_MEDIUM, 2, "Degrees: %.2lf", headingValue);
                    //headingValue = 0;
                    //master.print(0,0,"%f",headingValue);
                    //pros::screen::print(TEXT_MEDIUM, 3, "%f", headingValue);
                }

                if (thisDigit == 'C'){
                    recordOpticalX = false;
                    myStream2 >> distX;
                    //pros::screen::print(TEXT_LARGE, 3, "[Optical Flow]");
                    pros::screen::print(TEXT_MEDIUM, 6, "distX: %.2lf", distX);
                }

                if (thisDigit == 'D'){
                    recordOpticalY = false;
                    myStream3 >> distY;
                    pros::screen::print(TEXT_MEDIUM, 5, "[Optical Flow]");
                    pros::screen::print(TEXT_MEDIUM, 7, "distY: %.2lf", (-distY));
                }
                
                // If we want the digits, put them into stream
                if (recordAngle)
                    myStream << (char)buffer[i];
                
                if (recordOpticalX)
                    myStream2 << (char)buffer[i];

                if (recordOpticalY)
                    myStream3 << (char)buffer[i];
                
                // If the digit is 'A', then the following data is the angle
                if (thisDigit == 'A'){
                    recordAngle = true;
                }
                
                if (thisDigit == 'X')
                    recordOpticalX = true;
                
                if (thisDigit == 'Y')
                    recordOpticalY = true;
                
                //myStream >> headingValue;
                //master.print(0,0,"%f",headingValue);
                    
                
            }
            
        }
    
        // Delay to let serial data arrive
        pros::delay(10);
        //master.print(0, 6, "%.2lf", headingValue);
        pros::Task::delay(15);
        /*if(toggle){
            vexGenericSerialTransmit(SERIALPORT - 1, (uint8_t *)msg, 1);
            toggle = !toggle;
        }
        else if(!toggle){
            vexGenericSerialTransmit(SERIALPORT - 1, (uint8_t *)msg2, 1);
            toggle = !toggle;
        }     */
    }
    
}


double swerve_power, turn, turnR;

void swerve_ctrl() {
	// swerve motor
	pros::Motor luA(Left_UpperA_motor);
	pros::Motor luB(Left_UpperB_motor);
	pros::Motor llA(Left_LowerA_motor);
	pros::Motor llB(Left_LowerB_motor);
	pros::Motor ruA(Right_UpperA_motor);
	pros::Motor ruB(Right_UpperB_motor);
	pros::Motor rlA(Right_LowerA_motor);
	pros::Motor rlB(Right_LowerB_motor);
	
	pros::Rotation swerve_rotL(Swerve_Rot_L);
	pros::Rotation swerve_rotR(Swerve_Rot_R);

	double upperL, lowerL;
	double upperR, lowerR;
    double turnL;
	double headingL, headingR;
    double heading_errorL;
    double total_heading_errorL;
    double prev_heading_errorL;
    double delta_heading_errorL;


    // ensure that left swerve is following right swerve

	while(true) {
		headingL = swerve_rotL.get_angle();
		headingR = swerve_rotR.get_angle();

        if ((headingL - headingR) > 180) {
            headingL -= 360;
        }
        else if ((headingL - headingR) < -180) {
            headingL += 360;
        }

        heading_errorL = headingR - headingL;
        total_heading_errorL += heading_errorL;
        delta_heading_errorL = heading_errorL - prev_heading_errorL;
        prev_heading_errorL = heading_errorL;
        
        if (fabs(headingL - headingR) > 2) {
            turnL = (swerve_kp * heading_errorL) + (swerve_ki * total_heading_errorL) + (swerve_kd * delta_heading_errorL);
        }
		upperL = turnL - swerve_power;
		lowerL = turnL + swerve_power;
		upperR = turnR - swerve_power;
		lowerR = turnR + swerve_power;
		
        

		luA.move(upperL);
		luB.move(upperL);
		llA.move(lowerL);
		llB.move(lowerL);
		ruA.move(upperR);
		ruB.move(upperR);
		rlA.move(lowerR);
		rlB.move(lowerR);

		pros::delay(2);
	}
}

void initialize() {
	pros::Motor luA(Left_UpperA_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor luB(Left_UpperB_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor llA(Left_LowerA_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor llB(Left_LowerB_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor ruA(Right_UpperA_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor ruB(Right_UpperB_motor, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rlA(Right_LowerA_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rlB(Right_LowerB_motor, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

	pros::Serial serial(SERIALPORT);
    int32_t serial_enable(SERIALPORT);
    serial.set_baudrate(BAUDERATE);
	// pros::Task gyroTask(serialRead);
	pros::Task swerve(swerve_ctrl);
}

void disabled() {}
void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor luA(Left_UpperA_motor);
	pros::Motor luB(Left_UpperB_motor);
	pros::Motor llA(Left_LowerA_motor);
	pros::Motor llB(Left_LowerB_motor);
	pros::Motor ruA(Right_UpperA_motor);
	pros::Motor ruB(Right_UpperB_motor);
	pros::Motor rlA(Right_LowerA_motor);
	pros::Motor rlB(Right_LowerB_motor);
    double turnR;
    
    while (true) {
		turn = master.get_analog(ANALOG_RIGHT_X);
		swerve_power = master.get_analog(ANALOG_LEFT_Y);
		// turnL = master.get_analog(ANALOG_LEFT_X);
        turnR = master.get_analog(ANALOG_LEFT_X);
        

		pros::delay(2);
	}
}