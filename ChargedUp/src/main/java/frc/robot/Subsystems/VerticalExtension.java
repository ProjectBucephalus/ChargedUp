// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class VerticalExtension extends ProfiledPIDSubsystem {
    private WPI_TalonFX verticalExtension = new WPI_TalonFX(Constants.kVerticalElevatorCanId);
    private CANCoder verticalExtensionEncoder = new CANCoder(Constants.kVerticalElevatorEncoderCanId);

    private final ElevatorFeedforward verticalExtensionFeedfoward = new ElevatorFeedforward(
            Config.kVerticalExtensionKS, Config.kVerticalExtensionKG,
            Config.kVerticalExtensionKV, Config.kVerticalExtensionKA
          );

    public VerticalExtension() {
        super(
            new ProfiledPIDController(
                Config.kVerticalExtensionKP,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    Config.kVerticalExtensionMaxVelocity,
                    Config.kVerticalExtensionMaxAcceleration)),
            0);
        // Start arm at rest in neutral position
        setGoal(Config.kVerticalExtensionNeutralPosition + Config.kVerticalExtensionEncoderOffset);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = verticalExtensionFeedfoward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        verticalExtension.setVoltage(output + feedforward);
    }

    @Override
        public double getMeasurement() {
        return verticalExtensionEncoder.getPosition() / Config.kVerticalExtensionEncoderPPR + Config.kVerticalExtensionEncoderOffset;
    }

    public double calculateVerticalExtensionGoal(double x, double y) {
        return x * Math.cos(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) - y * Math.sin(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight));
      }

}
