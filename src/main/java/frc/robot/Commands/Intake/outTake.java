// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.Intake;

public class outTake extends SequentialCommandGroup {
  /** Creates a new Intake. */
  public outTake(Intake intakeSubsystem, Claw m_claw) {
    addCommands(
      new ExtendIntake(intakeSubsystem,m_claw),
      new ReverseIntakeMotors(intakeSubsystem)
    );
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

}
