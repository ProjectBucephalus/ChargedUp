package frc.robot.Commands.Intake;

    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feed;
import frc.robot.Subsystems.Feed.FeedMotorsStatus;

public class reverseFeed extends Command {
  private Feed m_feed;
  /** Creates a new Feed. */
  public reverseFeed(Feed feedSubsystem) {
    m_feed = feedSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feed.setFeed(FeedMotorsStatus.REVERSE);
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


