/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.LifterCommands.StopLifterMotorsCom;

/**
 * Add your docs here.
 */
public class LifterSub extends Subsystem {
  // Initialize all of the objects and values for the
  // subsystem-----------------------------------------------------------------------

  // front lifter
  private WPI_VictorSPX frontLifterMotor;
  private Encoder frontLifterEncoder;
  private DigitalInput frontTopLiftLimit;
  private DigitalInput frontBottomLiftLimit;
  private PIDController frontLiftPID;

  // rear lifter
  private WPI_VictorSPX rearLifterMotor;
  private Encoder rearLifterEncoder;
  private DigitalInput rearTopLiftLimit;
  private DigitalInput rearBottomLiftLimit;
  private PIDController rearLiftPID;

  // lift drive motor
  private WPI_VictorSPX liftDriveMotor;

  // pid coefficients
  private double kP, kI, kD, kMaxOutput, kMinOutput;

  // The constructor for the
  // subsystem---------------------------------------------------------------------------
  public LifterSub() {

    // Instatiate the motor and encoder objects and assign values for the subsystem

    // Front lifter
    frontLifterMotor = new WPI_VictorSPX(RobotMap.FRONT_LIFT_MOTOR_CH);
    frontLifterEncoder = new Encoder(RobotMap.FRONT_LIFT_ENC_CHANNELS[0], RobotMap.FRONT_LIFT_ENC_CHANNELS[1]);
    frontTopLiftLimit = new DigitalInput(RobotMap.FRONT_TOP_LIFT_LIMIT_CH);
    frontBottomLiftLimit = new DigitalInput(RobotMap.FRONT_BOTTOM_LIFT_LIMIT_CH);

    // Rear lifter
    rearLifterMotor = new WPI_VictorSPX(RobotMap.REAR_LIFT_MOTOR_CH);
    rearLifterEncoder = new Encoder(RobotMap.REAR_LIFT_ENC_CHANNELS[0], RobotMap.REAR_LIFT_ENC_CHANNELS[1]);
    rearTopLiftLimit = new DigitalInput(RobotMap.REAR_TOP_LIFT_LIMIT_CH);
    rearBottomLiftLimit = new DigitalInput(RobotMap.REAR_BOTTOM_LIFT_LIMIT_CH);

    // Lift driver motor
    liftDriveMotor = new WPI_VictorSPX(RobotMap.LIFT_DRIVE_MOTOR_CH);

    // create pidcontroller
    frontLiftPID = new PIDController(0, 0, 0, frontLifterEncoder, frontLifterMotor);
    rearLiftPID = new PIDController(0, 0, 0, rearLifterEncoder, rearLifterMotor);

    // create PID coefficients
    kP = 0.01;
    kI = 0;
    kD = 0;
    kMaxOutput = 0.5;
    kMinOutput = -0.5;

    // set PID coefficients
    // Front Lifter PID
    frontLiftPID.setP(kP);
    frontLiftPID.setI(kI);
    frontLiftPID.setD(kD);
    frontLiftPID.setOutputRange(kMinOutput, kMaxOutput);
    // Rear Lifter PID
    rearLiftPID.setP(kP);
    rearLiftPID.setI(kI);
    rearLiftPID.setD(kD);
    rearLiftPID.setOutputRange(kMinOutput, kMaxOutput);
  }

  // end of constructor--------------------------------------------------------

  // methods for printing values to SmartDashboard-----------------------------

  public void getAllLiftSensors() {
    getfrontLifterEncoder();
    getRearLiftEncoder();
    getFrontLiftTopLimit();
    getFrontLiftTopLimit();
    getRearLiftTopLimit();
    getRearLiftBottomLimit();
  }

  public double getfrontLifterEncoder() {
    SmartDashboard.putNumber("Front Lift Encoder Voltage", frontLifterEncoder.getDistance());
    return frontLifterEncoder.getDistance();
  }

  public double getRearLiftEncoder() {
    SmartDashboard.putNumber("Rear Lift Encoder Voltage", rearLifterEncoder.getDistance());
    return rearLifterEncoder.getDistance();
  }

  public boolean getFrontLiftTopLimit() {
    SmartDashboard.putBoolean("Front Top Lift Limit", frontTopLiftLimit.get());
    return frontTopLiftLimit.get();
  }

  public boolean getFrontLiftBottomLimit() {
    SmartDashboard.putBoolean("Front Bottom Lift Limit", frontBottomLiftLimit.get());
    return frontBottomLiftLimit.get();
  }

  public boolean getRearLiftTopLimit() {
    SmartDashboard.putBoolean("Rear Top Lift Limit", rearTopLiftLimit.get());
    return rearTopLiftLimit.get();
  }

  public boolean getRearLiftBottomLimit() {
    SmartDashboard.putBoolean("Rear Bottom Lift Limit", rearBottomLiftLimit.get());
    return rearBottomLiftLimit.get();
  }

  // Methods for moving lifters up
  // ------------------------------------------------------------------

  // Front lifter up
  public void frontLifterUp() {
    double frontLifterEncoderValue = getfrontLifterEncoder();
    boolean frontTopLimitValue = getFrontLiftTopLimit();

    if (frontLifterEncoderValue < 3.6 && frontTopLimitValue == true) {
      frontLifterMotor.set(-.5);
    }

    else {
      frontLifterMotor.set(0);
    }

  }

  // Rear lifter up
  public void rearLifterUp() {

    double rearLiftEncoderValue = getRearLiftEncoder();
    boolean rearTopLimitValue = getRearLiftTopLimit();

    if (rearLiftEncoderValue < 3.6 && rearTopLimitValue == true) {
      rearLifterMotor.set(-.5);
    }

    else {
      frontLifterMotor.set(0);
    }

  }

  // Both lifters down--------------------------------------------------

  public void bothLiftersDown() {
    double frontLifterEncoderValue = getfrontLifterEncoder();
    double rearLiftEncoderValue = getRearLiftEncoder();
    boolean frontBottomLimitValue = getFrontLiftTopLimit();
    boolean rearBottomLimitValue = getRearLiftTopLimit();

    
    //When front bottom limit is online add to this statement
    if (frontLifterEncoderValue > 0.8 && rearLiftEncoderValue > 0.9) {

      if (frontLifterEncoderValue < rearLiftEncoderValue - .075) {
        frontLifterMotor.set(.5);
        rearLifterMotor.set(.7);
      }

      else {
        frontLifterMotor.set(.7);
        rearLifterMotor.set(.5);
      }

    }

    else {
      frontLifterMotor.set(0);
      rearLifterMotor.set(0);
    }

  }

  // Method to call for default commmand to keep motors still during teleop
  public void stopLiftMotors() {
    frontLifterMotor.set(0);
    rearLifterMotor.set(0);
  }

  // Lifter drive motor methods---------------------------------------------
  public void lifterDriveForward() {
    liftDriveMotor.set(1);
  }

  public void lifterDriveReverse() {

    liftDriveMotor.set(-1);
  }

  public void lifterDriveStop() {
    liftDriveMotor.set(0);
  }

  // PID setpoint methods ---------------------------------------------------

  public void setFrontLifterPID(double setPoint) {
    frontLiftPID.setSetpoint(setPoint);
  }

  public void setRearLifterPID(double setPoint) {
    rearLiftPID.setSetpoint(setPoint);
  }

  // This method sets the default command to stop motors- this causes motors to
  // default to stop during normal robot operation
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new StopLifterMotorsCom());
  }
}