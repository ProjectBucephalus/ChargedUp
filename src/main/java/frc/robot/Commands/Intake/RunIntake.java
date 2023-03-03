// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake;

public class RunIntake extends SequentialCommandGroup {
  /** Creates a new Intake. */
  public RunIntake(Intake intakeSubsystem) {
    addCommands(
      new ExtendIntake(intakeSubsystem),
      
      new RunIntakeMotors(intakeSubsystem)
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }

}
