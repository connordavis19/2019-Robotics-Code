/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

// Create constants for the drive motors- change the values to the motor controller ID
public static int FRONT_LEFT_CHANNEL = 3;
public static int FRONT_RIGHT_CHANNEL = 5;
public static int REAR_RIGHT_CHANNEL = 4;
public static int REAR_LEFT_CHANNEL = 2;

// Create constants for the elevator motor and limits
public static int ELEVATOR_MOTOR_CHANNEL = 11;
public static int UPPER_ELEVATOR_LIMIT_CHANNEL= 1;
public static int LOWER_ELEVATOR_LIMIT_CHANNEL= 0;

//Create constants for lifters and lifter limits
public static int FRONT_LIFTER_CH = 10;
public static int REAR_LIFTER_CH = 12;
public static int LIFTER_DRIVE_CH = 9;
public static int FRONT_LIMIT_TOP_CH = 9;
public static int FRONT_LIMIT_BOTTOM_CH = 2;
public static int REAR_LIMIT_TOP_CH = 8;
public static int REAR_LIMIT_BOTTOM_CH = 6;

//Create constants for the drive solenoid channels- these should stay the same
public static int DRIVE_SOL_FORWARD_CH = 0;
public static int DRIVE_SOL_REVERSE_CH = 1;

//Create the constants for the driver joystick
public static int DRIVE_STICK_CH = 0;
public static int SHIFT_BUTTON_CH = 1;

//Create the constants for the elevator joystick
public static int ELEVATOR_STICK_CH = 1;

//Create constant for deadband
public static double DEADBAND_CH = 0.05;

//Create constant for herder potentiometer
public static int HERDER_POT_CH = 0;





}
