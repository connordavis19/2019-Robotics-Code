/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HerderSub extends Subsystem {

  private AnalogInput herderPot;

  public HerderSub() {
    // create new AnalogInput (potentiometer)
    herderPot = new AnalogInput(RobotMap.HERDER_POT_CH);

    // send to SmartDashboard
    SmartDashboard.putData(herderPot);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // get (voltage) of potentiometer
  public double getHerderPot() {
    return herderPot.getVoltage();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.

  }
}
