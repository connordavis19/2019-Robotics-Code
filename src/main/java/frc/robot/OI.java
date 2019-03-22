/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.BuffaloNoseInCom;
import frc.robot.commands.BuffaloNoseOutCom;
import frc.robot.commands.ElevatorDownCom;
import frc.robot.commands.ElevatorUpCom;
import frc.robot.commands.HerderCollect;
import frc.robot.commands.HerderDispense;
import frc.robot.commands.HerderDownCom;
import frc.robot.commands.HerderPIDCommand;
import frc.robot.commands.HerderUpCom;
import frc.robot.commands.LifterDriveCom;
import frc.robot.commands.MecanumDriveCom;

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
  private JoystickButton buffaloNoseIn;
  private JoystickButton buffaloNoseOut;

  private JoystickButton rearLiftersUpButton, rearLiftersDownButton, frontLiftersUpButton, frontLiftersDownButton,
      liftersUp, liftersDown, lifterDrive, lifterStick, collect, dispense, herderUpButton, herderDownButton;

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
    // rearLiftersDownButton = new JoystickButton(elevatorStick, 2);
    // frontLiftersDownButton = new JoystickButton(elevatorStick, 22);
    // liftersUp = new JoystickButton(elevatorStick, 2);
    // liftersDown = new JoystickButton(elevatorStick, 4);
    lifterDrive = new JoystickButton(elevatorStick, 7);
    lifterDrive.whileHeld(new LifterDriveCom());
    // liftersUp.whileHeld(new LifterUpCom());
    // liftersDown.whileHeld(new LifterDownCom());
    // rearLiftersDownButton.whileHeld(new RearLifterDownCom());
    // frontLiftersDownButton.whileHeld(new FrontLifterDownCom());
    // Herder Controls
    collect = new JoystickButton(elevatorStick, 5);
    dispense = new JoystickButton(elevatorStick, 6);
    herderUpButton = new JoystickButton(elevatorStick, 4);
    herderDownButton = new JoystickButton(elevatorStick, 2);


    buffaloNoseIn = new JoystickButton(elevatorStick, 9);
    buffaloNoseOut = new JoystickButton(elevatorStick, 10);
    buffaloNoseIn.whileHeld(new BuffaloNoseInCom());
    buffaloNoseOut.whileHeld(new BuffaloNoseOutCom());

    collect.whileHeld(new HerderCollect());
    dispense.whileHeld(new HerderDispense());


    herderUpButton.whileHeld(new HerderUpCom());
    herderDownButton.whileHeld(new HerderDownCom());

    // PID Controls (Using Elevator Joystick)
    pidButton = new JoystickButton(elevatorStick, 8);
    pidButton.toggleWhenPressed(new HerderPIDCommand());
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

  public double getCameraTwist() {
    return elevatorStick.getRawAxis(1);
  }

  public double getCameraYaw() {
    return elevatorStick.getRawAxis(0);
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
