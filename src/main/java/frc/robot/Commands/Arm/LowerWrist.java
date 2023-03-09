// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Wrist.WristPosition;

public class LowerWrist extends CommandBase {
  private final Wrist m_wrist;
  /** Creates a new RaiseWrist. */
  public LowerWrist(Wrist subsystem) {
    m_wrist = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setWristPosition(WristPosition.LOWERED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //should investigate delay here TODO
  }
}
