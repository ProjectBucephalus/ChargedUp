// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.VerticalExtension.verticalState;

public class ArmShelfPosCommand extends Command {
  private final Wrist m_wrist;
  private final HorizontalExtension m_horizontalExtension;
  private final VerticalExtension m_verticalExtension;
  /** Creates a new ArmShelfPosCommand. */
  public ArmShelfPosCommand(Wrist wristSubsystem, VerticalExtension verticalExtensionSubsystem, HorizontalExtension horizontalExtensionSubsystem) {
    m_wrist = wristSubsystem;
    m_verticalExtension = verticalExtensionSubsystem;
    m_horizontalExtension = horizontalExtensionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_verticalExtension.setPosition(Config.kArmShelfPosY);//m_verticalExtension.calculateVerticalExtensionGoal(Config.kArmShelfPosX, Config.kArmShelfPosY));
    m_verticalExtension.setDesiredState(verticalState.SHELF);

    if(m_verticalExtension.getArmAtPosition()){
      m_horizontalExtension.setPosition(Config.kArmShelfPosX);//m_horizontalExtension.calculateHorizontalExtensionGoal(Config.kArmShelfPosX, Config.kArmShelfPosY));
      m_wrist.setWristPosition(Config.kArmShelfPosWrist);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setWristPosition(Config.kArmShelfPosWrist);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;//return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
  }
}
