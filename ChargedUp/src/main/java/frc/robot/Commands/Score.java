// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;

public class Score extends SequentialCommandGroup {

  public enum ScorePos {
    LOW,
    MEDIUM,
    HIGH,
  }
  /** Creates a new Score. */
  public Score(ScorePos desiredpos, HorizontalExtension horizontalExtensionSubsystem, VerticalExtension verticalExtensionSubsystem, Wrist wristSubsystem) {

    switch(desiredpos) {
      case LOW:
        //TODO
      case HIGH:
        break;
      case MEDIUM:
        break;
      default:
        break;
    }
  }

}
