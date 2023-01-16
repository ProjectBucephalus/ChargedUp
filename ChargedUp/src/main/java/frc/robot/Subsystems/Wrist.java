// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class Wrist extends TrapezoidProfileSubsystem {

    private final WPI_TalonFX wristMotor = new WPI_TalonFX(Constants.kWristMotorCanId);
    private final CANCoder wristEncoder = new CANCoder(Constants.kWristEncoderCanId);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
        Config.kWristKS, Config.kWristKG,
        Config.kWristKV, Config.kWristKA
      );

  /** Create a new ArmSubsystem. */
  public Wrist() {
    super(
        new TrapezoidProfile.Constraints(
            Config.kWristMaxVelocity, Config.kWristMaxAcceleration),
        Config.kWristEncoderOffset);
    wristMotor.config_kP(0, Config.kHorizontalExtensionKP);
    wristMotor.configRemoteFeedbackFilter(wristEncoder, 0);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    wristMotor.config_kF(0, feedforward);
    wristMotor.set(ControlMode.Position, setpoint.position);
    
  }

  public Command setWristGoalCommand(double kWristEncoderOffset) {
    return Commands.runOnce(() -> setGoal(kWristEncoderOffset), this);
  }


}
