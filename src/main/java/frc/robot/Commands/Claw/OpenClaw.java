// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.Claw.ClawPosition;

public class OpenClaw extends Command {
  private Claw m_claw;
  /** Creates a new OpenClaw. */
  public OpenClaw(Claw clawSubsystem) {
    m_claw = clawSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.setClaw(ClawPosition.OPEN);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
