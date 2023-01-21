// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Wings;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.Subsystems.Wings;

public class StowWings extends CommandBase {
  private final Wings m_wings;
  /** Creates a new ArmHighPosCommand. */
  public StowWings(Wings wingSubsystem) {
    m_wings = wingSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wings.setArmGoalCommand(Config.kWingRetractedPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wings.getLeftWingAtPosition() && m_wings.getRightWingAtPosition(); //Check both wings are fully retracted
  }
}
