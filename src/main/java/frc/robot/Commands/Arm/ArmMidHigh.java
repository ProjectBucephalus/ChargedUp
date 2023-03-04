// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;

public class ArmMidHigh extends CommandBase {
  private final Wrist m_wrist;
  private final HorizontalExtension m_horizontalExtension;
  private final VerticalExtension m_verticalExtension;
  /** Creates a new ArmsumPosCommand. */
  public ArmMidHigh(Wrist wristSubsystem, VerticalExtension verticalExtensionSubsystem, HorizontalExtension horizontalExtensionSubsystem) {
    m_wrist = wristSubsystem;
    m_verticalExtension = verticalExtensionSubsystem;
    m_horizontalExtension = horizontalExtensionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("hellow");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_verticalExtension.setPosition(Config.kArmMedPosY-.385);//m_verticalExtension.calculateVerticalExtensionGoal(Config.kArmMedPosX, Config.kArmMedPosY));
    if(m_verticalExtension.getArmAtPosition()){
      m_horizontalExtension.setPosition(Config.kArmMedPosX +1);//m_horizontalExtension.calculateHorizontalExtensionGoal(Config.kArmMedPosX, Config.kArmMedPosY));
      m_wrist.setWristPosition(Config.kArmMedPosWrist);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;//return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
  }
}
