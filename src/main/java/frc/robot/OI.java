/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArcadeDriveCom;
import frc.robot.commands.ElevatorDownCom;
import frc.robot.commands.ElevatorUpCom;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private Joystick driveStick;
  private JoystickButton shiftButton;

  private Joystick elevatorStick;
  private JoystickButton elevatorUpButton;
  private JoystickButton elevatorDownButton;

  private double xSpeed;
  private double ySpeed;
  private double zRotation;

  private DigitalInput leftEye;
  private DigitalInput mideEye;
  private DigitalInput rightEye;

  public OI() {

    //Drive Joystick
    driveStick = new Joystick(RobotMap.DRIVE_STICK_CH);
    shiftButton = new JoystickButton(driveStick, RobotMap.SHIFT_BUTTON_CH);
    shiftButton.toggleWhenPressed(new ArcadeDriveCom());

    //Elevator Joystick
    elevatorStick = new Joystick(RobotMap.ELEVATOR_STICK_CH);
    elevatorUpButton = new JoystickButton(elevatorStick, 4);
    elevatorUpButton.whileHeld(new ElevatorUpCom());
    elevatorDownButton = new JoystickButton(elevatorStick, 2);
    elevatorDownButton.whileHeld(new ElevatorDownCom());
  }

  public double getX() {
    return driveStick.getRawAxis(1);
  }

  public double getY() {
    return driveStick.getRawAxis(0);
  }

  public double getTwist() {
    return driveStick.getRawAxis(4);
  }

  public boolean getLeftEye()
  {
    return leftEye.get();
  }

  public boolean getMidEye()
  {
    return leftEye.get();
  }

  public boolean getRightEye()
  {
    return leftEye.get();
  }

}
