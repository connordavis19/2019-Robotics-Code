/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class PIDSub extends Subsystem {

  private PIDController myPID; // instantiated in methods (as settings may vary)
  private AnalogPotentiometer fLifterPot, rLifterPot;
  private AnalogPotentiometer elevatorPot, herderPot;

  public PIDSub() {
    // set encoders (see RobotMap for IDs)
    fLifterPot = new AnalogPotentiometer(RobotMap.FRONT_LIFTER_POT_CH);
    rLifterPot = new AnalogPotentiometer(RobotMap.REAR_LIFTER_POT_CH);

    elevatorPot = new AnalogPotentiometer(RobotMap.ELEVATOR_POT_CH);
    herderPot = new AnalogPotentiometer(RobotMap.HERDER_POT_CH);

    // put data to SmartDasbhoard
    SmartDashboard.putData(myPID);
  }

  public double getPID(double min, double max, double Kp, double Ki, double Kd, PIDSource source, PIDOutput output) {
    myPID = new PIDController(Kp, Ki, Kd, source, output);
    myPID.setInputRange(min, max);
    return myPID.get();
  }

  public double getFrontLifterPot() {
    return fLifterPot.get();
  }

  public double getRearLifterPot() {
    return rLifterPot.get();
  }

  public double getElevatorPot() {
    return elevatorPot.get();
  }

  public double getHerderPot() {
    return herderPot.get();
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new LifterStick()); // TODO: set/create default command
  }
}
