// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;

public class ArmLowPosCommand extends CommandBase {
  private final Wrist m_wrist;
  private final HorizontalExtension m_horizontalExtension;
  private final VerticalExtension m_verticalExtension;
  /** Creates a new ArmLowPosCommand. */
  public ArmLowPosCommand(Wrist wristSubsystem, VerticalExtension verticalExtensionSubsystem, HorizontalExtension horizontalExtensionSubsystem) {
    m_wrist = wristSubsystem;
    m_verticalExtension = verticalExtensionSubsystem;
    m_horizontalExtension = horizontalExtensionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setWristPosition(Config.kArmLowPosWrist);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_verticalExtension.setPosition(Config.kArmLowPosY);//m_verticalExtension.calculateVerticalExtensionGoal(Config.kArmLowPosX, Config.kArmLowPosY));
    m_horizontalExtension.setPosition(Config.kArmHighPosX);//m_horizontalExtension.calculateHorizontalExtensionGoal(Config.kArmLowPosX, Config.kArmLowPosY));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;//return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
  }
}
