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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.commands.*;

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

  private JoystickButton rearLiftersUpButton, rearLiftersDownButton, frontLiftersUpButton, frontLiftersDownButton, liftersUp, liftersDown, lifterDrive, lifterStick;

  private JoystickButton pidButton;

  private double xSpeed;
  private double ySpeed;
  private double zRotation;

  private DigitalInput leftEye;
  private DigitalInput mideEye;
  private DigitalInput rightEye;

  public OI() {

    // Drive Joystick
    driveStick = new Joystick(RobotMap.DRIVE_STICK_CH);
    shiftButton = new JoystickButton(driveStick, RobotMap.SHIFT_BUTTON_CH);
    shiftButton.toggleWhenPressed(new MecanumDriveCom());

    // Elevator Joystick
    elevatorStick = new Joystick(RobotMap.ELEVATOR_STICK_CH);
    elevatorUpButton = new JoystickButton(elevatorStick, 3);
    elevatorUpButton.whileHeld(new ElevatorUpCom());
    elevatorDownButton = new JoystickButton(elevatorStick, 1);
    elevatorDownButton.whileHeld(new ElevatorDownCom());
      // Lifter Controls (Using Elevator Joystick)
      rearLiftersUpButton = new JoystickButton(elevatorStick, 5);
      rearLiftersDownButton = new JoystickButton(elevatorStick, 2);
      frontLiftersUpButton = new JoystickButton(elevatorStick, 6);
      frontLiftersDownButton = new JoystickButton(elevatorStick, 22);
      liftersUp = new JoystickButton(elevatorStick, 2);
      liftersDown = new JoystickButton(elevatorStick, 4);
      lifterDrive = new JoystickButton(elevatorStick, 7);
      // lifterStick = new JoystickButton(elevatorStick, 8);
      // lifterStick.whileHeld(new LifterStick());
      lifterDrive.whileHeld(new LifterDriveCom());
      liftersUp.whileHeld(new LifterUpCom());
      liftersDown.whileHeld(new LifterDownCom());
      rearLiftersUpButton.whileHeld(new RearLifterUpCom());
      rearLiftersDownButton.whileHeld(new RearLifterDownCom());
      frontLiftersUpButton.whileHeld(new FrontLifterUpCom());
      frontLiftersDownButton.whileHeld(new FrontLifterDownCom());

    // PID Controls (Using Elevator Joystick)
    pidButton = new JoystickButton(elevatorStick, 8);
    pidButton.whileHeld(new HerderPIDCommand());
  }

  public double getX() {
    return -driveStick.getRawAxis(1);
  }

  public double getY() {
    return driveStick.getRawAxis(0);
  }

  public double getTwist() {
    return driveStick.getRawAxis(2);
  }

  public double getSecondaryY() {
    return -elevatorStick.getRawAxis(0);
  }

  public double getSecondaryLX() {
    return elevatorStick.getRawAxis(2) * 0.5;
  }

  public double getSecondaryRX() {
    return elevatorStick.getRawAxis(3) * 0.5;
  }

  public boolean getLeftEye() {
    return leftEye.get();
  }

  public boolean getMidEye() {
    return leftEye.get();
  }

  public boolean getRightEye() {
    return leftEye.get();
  }

}
