// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class HorizontalExtension extends ProfiledPIDSubsystem {
    private WPI_TalonFX horizontalExtension = new WPI_TalonFX(Constants.kHorizontalElevatorCanId);
    private CANCoder horizontalExtensionEncoder = new CANCoder(Constants.kHorizontalElevatorEncoderCanId);

    private final ElevatorFeedforward horizontalExtensionFeedfoward = new ElevatorFeedforward(
            Config.kHorizontalExtensionKS, Config.kHorizontalExtensionKG,
            Config.kHorizontalExtensionKV, Config.kHorizontalExtensionKA
          );

    public HorizontalExtension() {
        super(
            new ProfiledPIDController(
                Config.kHorizontalExtensionKP,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    Config.kHorizontalExtensionMaxVelocity,
                    Config.kHorizontalExtensionMaxAcceleration)),
            0);
        // Start arm at rest in neutral position
        setGoal(Config.kHorizontalExtensionNeutralPosition + Config.kHorizontalExtensionEncoderOffset);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = horizontalExtensionFeedfoward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        horizontalExtension.setVoltage(output + feedforward);
    }

    @Override
        public double getMeasurement() {
        return horizontalExtensionEncoder.getPosition() / Config.kHorizontalExtensionEncoderPPR + Config.kHorizontalExtensionEncoderOffset;
    }

}
