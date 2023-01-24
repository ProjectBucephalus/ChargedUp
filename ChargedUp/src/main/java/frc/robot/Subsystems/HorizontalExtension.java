// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class HorizontalExtension extends TrapezoidProfileSubsystem {


    private double armGoal = 0;
    private final WPI_TalonFX horizontalExtensionMotor = new WPI_TalonFX(Constants.kHorizontalElevatorCanId);
    private final CANCoder horizontalExtensionEncoder = new CANCoder(Constants.kHorizontalElevatorEncoderCanId); //we want an absolute encoder for reliabality
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward( //define feedFoward control for the elevator
        Config.kHorizontalExtensionKS, Config.kHorizontalExtensionKG, //kS is the static friction constant, kG is the gravity constant
        Config.kHorizontalExtensionKV, Config.kHorizontalExtensionKA //kV is the velocity constant, kA is the acceleration constant
      );

      //this subsystem uses a combination of a feedfoward and feedback control.
      //this gives us the advantage of better profiling from feedfoward and better precision from feedback.

  /** Create a new ArmSubsystem. */
  public HorizontalExtension() {
    //define a new trapezoid profile using wpi libraries. Configure the motor.
    super(
        new TrapezoidProfile.Constraints(
            Config.kMaxSpeedMetersPerSecond, Config.kMaxAccelerationMetersPerSecondSquared),
        Config.kHorizontalExtensionEncoderOffset);
    initSystem();
  }

  /**
   * Reset and configure sensors, motors, and encoders.
   */
  public void initSystem() {
    horizontalExtensionMotor.configFactoryDefault(); //reset and configure the motor so we know it is correctly configured
    horizontalExtensionMotor.config_kP(0, Config.kHorizontalExtensionKP);
    horizontalExtensionMotor.configRemoteFeedbackFilter(horizontalExtensionEncoder, 0);
    horizontalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    horizontalExtensionEncoder.configFactoryDefault(); //reset and configure the encoder
    horizontalExtensionEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
  }


  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    horizontalExtensionMotor.config_kF(0, feedforward);
    horizontalExtensionMotor.set(ControlMode.Position, setpoint.position);  
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
   * Method to calculate the desired porition of the motor based off a target x and y position.
   * @param x desired x position
   * @param y desired y position
   * @return the desired setpoint for the extension
   */
  public static double calculateHorizontalExtensionGoal(double x, double y) {
    return Config.kVerticalExtensionPerpendicularHeight * Math.cos(Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight));
  }

  /**
   * 
   * @return true if arm is within defined position tollerence.
   */
  public boolean getArmAtPosition() {
    if(getArmPosition() <= armGoal + Config.kHorizontalExtensionPositionTollerenceMetres && getArmPosition() >= armGoal - Config.kHorizontalExtensionPositionTollerenceMetres) {
      return true;
    }
    return false;
  }

  public double getArmPosition() {
    return horizontalExtensionEncoder.getPosition() / Config.kHorizontalExtensionEncoderPPR * Config.kHorizontalExtensionMetresPerRotation;
  }

}
