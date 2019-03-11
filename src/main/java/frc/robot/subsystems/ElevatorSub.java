/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorStopCom;

/**
 * Add your docs here.
 */
public class ElevatorSub extends Subsystem {

  private WPI_VictorSPX elevatorMotor;
  private DigitalInput elevatorTopLimit;
  private DigitalInput elevatorBottomLimit;

  public ElevatorSub() {
    elevatorMotor = new WPI_VictorSPX(RobotMap.ELEVATOR_MOTOR_CHANNEL);
    elevatorTopLimit = new DigitalInput(1);
    SmartDashboard.putData(elevatorTopLimit);
    elevatorBottomLimit = new DigitalInput(0);
    SmartDashboard.putData(elevatorBottomLimit);
  }

  public boolean getTopLimit() {
    return elevatorTopLimit.get();
  }

  public boolean getBottomLimit() {
    return elevatorBottomLimit.get();
  }

  public void elevatorUp() {
<<<<<<< HEAD
    elevatorMotor.set( .5);
  }

  public void elevatorDown() {
    elevatorMotor.set(-.5);
=======
    if (getTopLimit())
      elevatorMotor.set(.2);
  }

  public void elevatorDown() {
    if (getBottomLimit())
      elevatorMotor.set(-.2);
>>>>>>> 45f9f0cafa94508632e30e5df2244d7e4d5fade9
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorStopCom());
  }
}
