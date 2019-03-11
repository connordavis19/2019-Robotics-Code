/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorStopCom;

/**
 * Add your docs here.
 */
public class LifterSub extends Subsystem {

  private WPI_VictorSPX frontLifter, rearLifter, lifterDrive;
  private DigitalInput fLimitTop, fLimitBottom, rLimitTop, rLimitBottom;

  public LifterSub() {
    // set lifter motors (see RobotMap for IDs)
    frontLifter = new WPI_VictorSPX(RobotMap.FRONT_LIFTER_CH);
    rearLifter = new WPI_VictorSPX(RobotMap.REAR_LIFTER_CH);
    lifterDrive = new WPI_VictorSPX(RobotMap.LIFTER_DRIVE_CH);

    //set lifter limits (see RobotMap for IDs)
    fLimitTop = new DigitalInput(RobotMap.FRONT_LIMIT_TOP_CH);
    fLimitBottom = new DigitalInput(RobotMap.FRONT_LIMIT_BOTTOM_CH);
    rLimitTop = new DigitalInput(RobotMap.REAR_LIMIT_TOP_CH);
    rLimitBottom = new DigitalInput(RobotMap.REAR_LIMIT_BOTTOM_CH);
  }

  public boolean getFrontTopLimit() {
    return fLimitTop.get();
  }

  public boolean getFrontBottomLimit() {
    return fLimitBottom.get();
  }

  public boolean getRearTopLimit() {
    return rLimitTop.get();
  }

  public boolean getRearBottomLimit() {
    return rLimitBottom.get();
  }

  public void liftUp() {
    frontLifter.set(0.2);
    rearLifter.set(0.2);
  }

  public void liftDown() {
    frontLifter.set(-0.2);
    rearLifter.set(-0.2);
  }

  public void frontLiftUp() {
    frontLifter.set(0.2);
  }

  public void frontLiftDown() {
    frontLifter.set(-0.2);
  }

  public void rearLiftUp() {
    rearLifter.set(0.2);
  }

  public void rearLiftDown() {
    rearLifter.set(-0.2);
  }

  public void stopAll() {
    frontLifter.set(0);
    rearLifter.set(0);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorStopCom());
  }
}
