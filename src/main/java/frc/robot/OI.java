/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

  private JoystickButton elevatorUpButton;
  private JoystickButton elevatorDownButton;
  private double xSpeed;
  private double ySpeed;
  private double zRotation;

  public OI() {
    driveStick = new Joystick(RobotMap.DRIVE_STICK_CH);
    shiftButton = new JoystickButton(driveStick, RobotMap.SHIFT_BUTTON_CH);
    shiftButton.toggleWhenPressed(new ArcadeDriveCom());

    elevatorUpButton = new JoystickButton(driveStick, 4);
    elevatorUpButton.whileHeld(new ElevatorUpCom());
    elevatorDownButton = new JoystickButton(driveStick, 2);
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


}
