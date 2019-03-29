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
  private JoystickButton buffaloNoseIn;
  private JoystickButton buffaloNoseOut;

  private JoystickButton rearLiftersUpButton, rearLiftersDownButton, frontLiftersUpButton, frontLiftersDownButton,
      liftersUp, liftersDown, lifterDrive, lifterStick, collect, dispense, herderUpButton, herderDownButton;

  private JoystickButton herderArmInButton;
  private JoystickButton herderArmOutButton;

  private JoystickButton frontLifterPID, rearLifterPID;

  private double xSpeed;
  private double xjoy;
  private double ySpeed;
  private double yjoy;
  private double zRotation;
  private double zjoy;
  private int throttleCurve = 10;

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
    // frontLiftersUpButton = new JoystickButton(elevatorStick, );
    // frontLiftersDownButton = new JoystickButton(elevatorStick, 10);
    collect = new JoystickButton(elevatorStick, 5);
    dispense = new JoystickButton(elevatorStick, 6);
    herderUpButton = new JoystickButton(elevatorStick, 4);
    herderDownButton = new JoystickButton(elevatorStick, 2);


    // buffaloNoseIn = new JoystickButton(elevatorStick, 9);
    // buffaloNoseOut = new JoystickButton(elevatorStick, 10);
    // buffaloNoseIn.whileHeld(new BuffaloNoseInCom());
    // buffaloNoseOut.whileHeld(new BuffaloNoseOutCom());

    collect.whileHeld(new HerderCollect());
    dispense.whileHeld(new HerderDispense());

    // PID Controls (Using Elevator Joystick)
    herderArmInButton = new JoystickButton(elevatorStick, RobotMap.HERDER_ARM_IN_BTN);
    herderArmInButton.whenPressed(new TestHerderArmInCom());

    herderArmOutButton = new JoystickButton(elevatorStick, RobotMap.HERDER_ARM_OUT_BTN);
    herderArmOutButton.whenPressed(new TestHerderArmOutCom());
  }

  //Added an exponential curve to the drive throttle between 0 and 1
  //Higher throttleCurve > steeper, smaller throttleCurve > shallower     -Andrew 
  public double getX() {
    // xjoy = driveStick.getRawAxis(1);
    // return (-xjoy*(1/throttleCurve)*Math.pow(throttleCurve, -xjoy));
    return -driveStick.getRawAxis(1);
  }

  public double getY() {
    // yjoy = driveStick.getRawAxis(0);
    // return (yjoy*(1/throttleCurve)*Math.pow(throttleCurve, yjoy));
    return driveStick.getRawAxis(0);
  }

  public double getTwist() {
    // zjoy = driveStick.getRawAxis(2);
    // return (zjoy*(1/throttleCurve)*Math.pow(throttleCurve, zjoy));
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
