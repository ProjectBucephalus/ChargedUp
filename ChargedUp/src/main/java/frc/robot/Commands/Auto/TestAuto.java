// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Config;
import frc.robot.Subsystems.Drive;

/** Add your docs here. */
public class TestAuto {

    Drive m_drive;
    public TestAuto() {
    }

    public Command getAutonomousCommand() {
        var autoVoltage = 
          new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), Drive.getDriveKinematics(), Config.kAutoMaxVolts);
    
        TrajectoryConfig config = new TrajectoryConfig(Config.kMaxSpeedMetersPerSecond, Config.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(Drive.getDriveKinematics()).addConstraint(autoVoltage);
    
        Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(1, 1), 
              new Translation2d(2, -1)), 
            new Pose2d(3, 0, new Rotation2d(0)), config);
            //@SuppressWarnings("unchecked")
    
        RamseteCommand ramseteCommand = 
          new RamseteCommand(testTrajectory, 
            (Supplier<Pose2d>) m_drive.getPose(), 
            new RamseteController(Config.kRamseteB, Config.kRamseteZeta), 
            new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), 
            Drive.getDriveKinematics(), 
            (Supplier<DifferentialDriveWheelSpeeds>) m_drive.getWheelSpeeds(), 
            new PIDController(Config.kPDriveVel, 0, 0), 
            new PIDController(Config.kPDriveVel, 0, 0),
            m_drive::tankDriveVolts,
            m_drive);
    
        return ramseteCommand;
        
      } 
    
}
