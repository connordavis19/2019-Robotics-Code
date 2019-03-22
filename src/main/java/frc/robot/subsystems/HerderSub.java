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
import edu.wpi.first.wpilibj.DoubleSolenoid;
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

  private double kP, kI, kD, kIz, kF, kMaxOutput, kMinOutput;

  private DoubleSolenoid buffaloNose;

  private WPI_VictorSPX herder;
  private WPI_VictorSPX herderCollectorTop;
  private WPI_VictorSPX herderCollectorBottom;

  public HerderSub() {
    // create new AnalogInput (potentiometer)
    herderPot = new AnalogPotentiometer(RobotMap.HERDER_POT_CH);
    // create herder motor (VictorSPX)
    herder = new WPI_VictorSPX(RobotMap.HERDER_MOTOR_CHANNEL);
    herderCollectorTop = new WPI_VictorSPX(RobotMap.HERDER_TOP_COLLECTOR_CHANNEL);
    herderCollectorBottom = new WPI_VictorSPX(RobotMap.HERDER_BOTTOM_COLLECTOR_CHANNEL);
    buffaloNose = new DoubleSolenoid(RobotMap.BUFFALO_NOSE_FWD, RobotMap.BUFFALO_NOSE_BWD);

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
    SmartDashboard.putData(myPID);
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

  public void collect() {
    herderCollectorBottom.set(1);
    herderCollectorTop.set(1);
  }

  public void dispense() {
    herderCollectorBottom.set(-1);
    herderCollectorTop.set(-1);
  }

  public void herderCollectStop() {
    herderCollectorBottom.set(0);
    herderCollectorTop.set(0);
  }

  public void herderUp() {
    herder.set(0.25);
  }

  public void herderDown() {
    herder.set(-0.25);
  }

  public void herderStop() {
    herder.set(0);
  }

  public void buffaloNoseIn()
  {
    buffaloNose.set(DoubleSolenoid.Value.kForward);
  }

  public void buffaloNoseOut()
  {
    buffaloNose.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.

  }
}
