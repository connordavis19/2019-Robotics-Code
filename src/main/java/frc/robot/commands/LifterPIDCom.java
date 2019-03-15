/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LifterPIDCom extends Command {
    public LifterPIDCom() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.lifterSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lifterSub.setFrontLifterPID(0); // TODO: find a midpoint to use to test if it can hold the herder in position
    Robot.lifterSub.setRearLifterPID(0);
    Robot.lifterSub.getFrontLifterPot();
    Robot.lifterSub.getRearLifterPot();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false; //!Robot.lifterSub.getRearBottomLimit() && !Robot.lifterSub.getFrontBottomLimit(); // limit switch code in LifterSub (so they will stop independantly)
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}