// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Config;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;

public class ArmZeroPosCommand extends CommandBase {
  private final Wrist m_wrist;
  private final HorizontalExtension m_horizontalExtension;
  private final VerticalExtension m_verticalExtension;
  /** Creates a new ArmHomePosCommand. */
  public ArmZeroPosCommand(Wrist wristSubsystem, VerticalExtension verticalExtensionSubsystem, HorizontalExtension horizontalExtensionSubsystem) {
    m_wrist = wristSubsystem;
    m_verticalExtension = verticalExtensionSubsystem;
    m_horizontalExtension = horizontalExtensionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Start");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_verticalExtension.setPosition(0);//m_verticalExtension.calculateVerticalExtensionGoal(Config.kArmMedPosX, Config.kArmMedPosY));
    m_horizontalExtension.setPosition(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("Finish");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
return true;  
  }
}
