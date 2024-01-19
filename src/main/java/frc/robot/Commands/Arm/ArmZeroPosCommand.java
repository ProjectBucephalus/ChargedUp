// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.VerticalExtension.verticalState;
import frc.robot.Subsystems.Wrist.WristPosition;

public class ArmZeroPosCommand extends Command {
  private final Wrist m_wrist;
  private final HorizontalExtension m_horizontalExtension;
  private final VerticalExtension m_verticalExtension;
  /** Creates a new ArmHomePosCommand. */
  public ArmZeroPosCommand(Wrist wristSubsystem, VerticalExtension verticalExtensionSubsystem, HorizontalExtension horizontalExtensionSubsystem, Claw claw) {
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
    m_verticalExtension.setDesiredState(verticalState.ZERO);
   
    m_horizontalExtension.setPosition(0);
    m_wrist.setWristPosition(WristPosition.LOWERED);
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
