/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.*;

/**
 * Add your docs here.
 */
public class LifterSub extends Subsystem {

  private AnalogPotentiometer frontLifterPot;
  private AnalogPotentiometer rearLifterPot;

  private PIDController frontLifterPID;
  private PIDController rearLifterPID;
  private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput;

  private WPI_VictorSPX frontLifter, rearLifter, lifterDrive;
  private DigitalInput frontLimitTop, frontLimitBottom, rearLimitTop, rearLimitBottom;

  public LifterSub() {
    // set lifter motors (see RobotMap for IDs)
    frontLifter = new WPI_VictorSPX(RobotMap.FRONT_LIFTER_CH);
    rearLifter = new WPI_VictorSPX(RobotMap.REAR_LIFTER_CH);
    lifterDrive = new WPI_VictorSPX(RobotMap.LIFTER_DRIVE_CH);

    // create lifter potentionmetrs
    frontLifterPot = new AnalogPotentiometer(RobotMap.FRONT_LIFTER_POT_CH);
    rearLifterPot = new AnalogPotentiometer(RobotMap.REAR_LIFTER_POT_CH);

    // create pidcontroller
    frontLifterPID = new PIDController(0, 0, 0, frontLifterPot, frontLifter);
    rearLifterPID = new PIDController(0, 0, 0, rearLifterPot, rearLifter);

    // create PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 150;
    kMaxOutput = 0.2;
    kMinOutput = -0.2;

    // set PID coefficients
    // Front Lifter PID
    frontLifterPID.setP(kP);
    frontLifterPID.setI(kI);
    frontLifterPID.setD(kD);
    frontLifterPID.setInputRange(kMinOutput, kMaxOutput);
    // Rear Lifter PID
    rearLifterPID.setP(kP);
    rearLifterPID.setI(kI);
    rearLifterPID.setD(kD);
    rearLifterPID.setInputRange(kMinOutput, kMaxOutput);

    // send to SmartDashboard
    SmartDashboard.putData(frontLifterPot);
    SmartDashboard.putData(rearLifterPot);
    SmartDashboard.putData(frontLifterPID);
    SmartDashboard.putData(rearLifterPID);

    // set lifter limits (see RobotMap for IDs)
    frontLimitTop = new DigitalInput(RobotMap.FRONT_LIMIT_TOP_CH);
    frontLimitBottom = new DigitalInput(RobotMap.FRONT_LIMIT_BOTTOM_CH);
    rearLimitTop = new DigitalInput(RobotMap.REAR_LIMIT_TOP_CH);
    rearLimitBottom = new DigitalInput(RobotMap.REAR_LIMIT_BOTTOM_CH);

    SmartDashboard.putData(frontLifter);
    SmartDashboard.putData(rearLifter);
    SmartDashboard.putData(lifterDrive);
    SmartDashboard.putData(frontLimitTop);
    SmartDashboard.putData(frontLimitBottom);
    SmartDashboard.putData(rearLimitTop);
    SmartDashboard.putData(rearLimitBottom);
    // SmartDashboard.putData(frontLifterPot);
    // SmartDashboard.putData(rearLifterPot);
  }

  public double getFrontLifterPot() {
    return frontLifterPot.get();
  }

  public void setFrontLifterPID(double setPoint) {
    frontLifterPID.setSetpoint(setPoint);
  }

  public double getRearLifterPot() {
    return rearLifterPot.get();
  }

  public void setRearLifterPID(double setPoint) {
    rearLifterPID.setSetpoint(setPoint);
  }

  public boolean getFrontTopLimit() {
    return frontLimitTop.get();
  }

  public boolean getFrontBottomLimit() {
    return frontLimitBottom.get();
  }

  public boolean getRearTopLimit() {
    return rearLimitTop.get();
  }

  public boolean getRearBottomLimit() {
    return rearLimitBottom.get();
  }

  public void liftUp() {
    if (getFrontTopLimit())
      frontLifter.set(-0.52);
    if (getRearTopLimit())
      rearLifter.set(-0.55);
  }

  public void liftDown() {
    if (getFrontBottomLimit())
      frontLifter.set(0.52);
    if (getRearBottomLimit())
      rearLifter.set(.55);
  }

  public void frontLiftUp() {
    frontLifter.set(-0.5);
  }

  public void frontLiftDown() {
    frontLifter.set(0.5);
  }

  public void rearLiftUp() {
    rearLifter.set(-0.5);
  }

  public void rearLiftDown() {
    rearLifter.set(0.5);
  }

  public void lifterDrive(double xSpeed) {
    lifterDrive.set(xSpeed);
  }

  public void analogLift(double rearX, double frontX) {
    frontLifter.set(frontX);
    rearLifter.set(rearX);
  }

  public void frontLiftStop() {
    frontLifter.set(0);
  }

  public void rearLiftStop() {
    rearLifter.set(0);
  }

  public void stopLifters() {
    frontLifter.set(0);
    rearLifter.set(0);
  }

  public void stopLifterDrive() {
    lifterDrive.set(0);
  }

  public void stopAll() {
    frontLifter.set(0);
    rearLifter.set(0);
    lifterDrive.set(0);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new LifterStick());
    setDefaultCommand(new RearLifterSlaveAndDriveCom());
  }
}
