// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class VerticalExtension extends TrapezoidProfileSubsystem {

    private final WPI_TalonFX verticalExtensionMotor = new WPI_TalonFX(Constants.kHorizontalElevatorCanId);
    private final CANCoder verticalExtensionEncoder = new CANCoder(Constants.kHorizontalElevatorEncoderCanId);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        Config.kVerticalExtensionKS, Config.kVerticalExtensionKG,
        Config.kVerticalExtensionKV, Config.kVerticalExtensionKA
      );

  /** Create a new ArmSubsystem. */
  public VerticalExtension() {
    super(
        new TrapezoidProfile.Constraints(
            Config.kMaxSpeedMetersPerSecond, Config.kMaxAccelerationMetersPerSecondSquared),
        Config.kVerticalExtensionEncoderOffset);
    verticalExtensionMotor.config_kP(0, Config.kVerticalExtensionKP);
    verticalExtensionMotor.configRemoteFeedbackFilter(verticalExtensionEncoder, 0);
    verticalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    verticalExtensionMotor.config_kF(0, feedforward);
    verticalExtensionMotor.set(ControlMode.Position, setpoint.position);
    
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

    public static double calculateVerticalExtensionGoal(double x, double y) {
        return x * Math.cos(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) - y * Math.sin(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight));
      }

}
