/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HerderSub extends Subsystem {

  private AnalogPotentiometer herderPot;
  private PIDController myPID;
  private WPI_VictorSPX herder;
  private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput;

  public HerderSub() {
    // create new AnalogInput (potentiometer)
    herderPot = new AnalogPotentiometer(RobotMap.HERDER_POT_CH);
    // create herder motor (VictorSPX)
    herder = new WPI_VictorSPX(RobotMap.HERDER_MOTOR_CH);

    // create pidcontroller
    myPID = new PIDController(0, 0, 0, herderPot, herder);

    // create PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 150;
    kF = 0;
    kMaxOutput = 0.2;
    kMinOutput = -0.2;

    // set PID coefficients
    myPID.setP(kP);
    myPID.setI(kI);
    myPID.setD(kD);
    myPID.setF(kF);
    myPID.setInputRange(kMinOutput, kMaxOutput);

    // send to SmartDashboard
    SmartDashboard.putData(herderPot);
    SmartDashboard.putData(herder);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // get (voltage) of potentiometer
  public double getHerderPot() {
    return herderPot.get();
  }

  public void setPID(double setPoint) {
    myPID.setSetpoint(setPoint);
  }

  public void setHerder(double speed) {
    herder.set(speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.

  }
}
