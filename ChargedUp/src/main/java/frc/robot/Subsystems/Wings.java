// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class Wings extends TrapezoidProfileSubsystem {

    private final WPI_TalonFX leftWingMotor = new WPI_TalonFX(Constants.kVerticalElevatorCanId);
    private final WPI_TalonFX rightWingMotor = new WPI_TalonFX(Constants.kVerticalElevatorCanId);

    private double armGoal = 0;

    //this subsystem uses a combination of a feedfoward and feedback control.
    //this gives us the advantage of better profiling from feedfoward and better precision from feedback.
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
        Config.kWingKS, Config.kWingKG, //kS is the static friction constant, kG is the gravity constant
        Config.kWingKV, Config.kWingKA //kV is the velocity constant, kA is the acceleration constant
      );

  /** Create a new ArmSubsystem. */
  public Wings() {
    //define a new trapezoid profile using wpi libraries. Configure the motor.
    super(
        new TrapezoidProfile.Constraints(
            Config.kMaxSpeedMetersPerSecond, Config.kMaxAccelerationMetersPerSecondSquared),
        Config.kWingEncoderOffset);
    initSystem();

  }

  /**
   * Reset and configure sensors, motors, and encoders.
   */
  public void initSystem() {
    leftWingMotor.configFactoryDefault(); //reset and configure the motor so we know it is correctly configured
    leftWingMotor.config_kP(0, Config.kWingKP);
    rightWingMotor.configFactoryDefault(); //reset and configure the motor so we know it is correctly configured
    rightWingMotor.config_kP(0, Config.kWingKP);

  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    leftWingMotor.config_kF(0, feedforward);
    leftWingMotor.set(ControlMode.Position, setpoint.position);
    rightWingMotor.config_kF(0, feedforward);
    rightWingMotor.set(ControlMode.Position, setpoint.position);
    
  }

  /**
   * Run arm to position
   * @param kArmOffsetRads
   * @return when arm is at desired position
   */
  public Command setArmGoalCommand(double kArmOffsetRads) {
    armGoal = kArmOffsetRads;
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

  /**
   * 
   * @return true if arm is within defined position tollerence.
   */
  public boolean getLeftWingAtPosition() {
    if(getLeftWingPosition() <= armGoal + Config.kWingPositionTollerenceDegrees && getLeftWingPosition() >= armGoal - Config.kWingPositionTollerenceDegrees) {
      return true;
    }
    return false;
  }
  /**
   * Get position of left wing in degrees
   * @return left wing encoder position in degrees
   */
  public double getLeftWingPosition() {
    return leftWingMotor.getSelectedSensorPosition() / Config.kWingEncoderPPR * Config.kWingDegreesPerRotation;
  }
  
  /**
   * 
   * @return true if arm is within defined position tollerence.
   */
  public boolean getRightWingAtPosition() {
    if(getRightWingPosition() <= armGoal + Config.kWingPositionTollerenceDegrees && getRightWingPosition() >= armGoal - Config.kWingPositionTollerenceDegrees) {
      return true;
    }
    return false;
  }

  public double getRightWingPosition() {
    return rightWingMotor.getSelectedSensorPosition() / Config.kWingEncoderPPR * Config.kWingDegreesPerRotation;
  }

  /**
   * Flap wings really fast to liftoff
   * Hovercrafts are boring!
   * @deprecated
   */
  public void fly() {
    //Flap your wings and fly
    //TODO
  }

}
