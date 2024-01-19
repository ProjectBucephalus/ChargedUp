// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Wrist.WristPosition;

public class RaiseWrist extends Command {
  private final Wrist m_wrist;
  /** Creates a new RaiseWrist. */
  public RaiseWrist(Wrist subsystem) {
    m_wrist = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setWristPosition(WristPosition.RAISED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //should investigate delay here TODO
  }
}
