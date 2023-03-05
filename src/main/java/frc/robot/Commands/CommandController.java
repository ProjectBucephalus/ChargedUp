// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Claw.CloseClaw;
import frc.robot.Commands.Claw.OpenClaw;
import frc.robot.Commands.Intake.ExtendIntake;
import frc.robot.Commands.Intake.RetractIntake;
import frc.robot.Commands.Intake.RunFeed;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.RunIntakeMotors;
import frc.robot.Commands.Intake.StopFeed;
import frc.robot.Commands.Intake.StopIntake;
import frc.robot.Commands.Intake.StopIntakeMotors;
import frc.robot.Subsystems.Claw;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Commands.Arm.ArmHighPosCommand;
import frc.robot.Commands.Arm.ArmHomePosCommand;
import frc.robot.Commands.Arm.ArmLowPosCommand;
import frc.robot.Commands.Arm.ArmMediumPosCommand;
import frc.robot.Commands.Arm.ArmMidHigh;
import frc.robot.Commands.Arm.ArmZeroPosCommand;
import frc.robot.Commands.Arm.intakeArm;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Feed;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;
import frc.robot.Utilities.PbSlewRateLimiter;
import frc.robot.Utilities.PbSlewRateLimiter.Constraints;

/** Add your docs here. */
public class CommandController {

    private final Drive m_drive = Drive.getInstance();
    private final Wrist m_wrist = new Wrist();
    private final Claw m_claw = new Claw();
    private final Feed m_feed = new Feed();
    private final HorizontalExtension m_horizontal = new HorizontalExtension();
    private final VerticalExtension m_vertical = VerticalExtension.getInstance();
    CommandJoystick m_driverJoystick = new CommandJoystick(0); //declare joystick on ds port 0 
    CommandXboxController m_driverHID = new CommandXboxController(1); //declare xbox on ds port 1
    private static PbSlewRateLimiter limiter = new PbSlewRateLimiter(new PbSlewRateLimiter.Constraints(2,.5),new PbSlewRateLimiter.State(5, 0), new PbSlewRateLimiter.State(0, 0) );
    private final Intake m_intake = new Intake();
    private boolean revState = false;


    /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called during {@link Robot#robotInit()}.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {

    // Control the drive with split-stick arcade controls
    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(
            () -> -m_driverJoystick.getY() * 1 * m_drive.getThrottleInput(m_driverJoystick),
            () -> -m_driverJoystick.getX() * 1 * m_drive.getThrottleInput(m_driverJoystick)));
    
    
    m_driverHID.y()
      .onTrue(
        new ArmHighPosCommand(m_wrist, m_vertical, m_horizontal)
      );
    m_driverHID.a()
      .onTrue(
        new ArmHomePosCommand(m_wrist, m_vertical, m_horizontal)
      );
    
    m_driverHID.x()
      .onTrue(
        new ArmLowPosCommand(m_wrist, m_vertical, m_horizontal)
      );

    m_driverHID.b()
      .onTrue(
        new ArmMediumPosCommand(m_wrist, m_vertical, m_horizontal)
      );

      m_driverHID.leftBumper()
       .onTrue(
        new ArmZeroPosCommand(m_wrist, m_vertical, m_horizontal)
      );
      m_driverHID.rightBumper().onTrue(
        new intakeArm(m_wrist, m_vertical, m_horizontal)
      );


      m_driverJoystick.button(2).onTrue(
        new CloseClaw(m_claw) 
      );
      m_driverJoystick.button(2).onFalse(
        new OpenClaw(m_claw)    
        );
      m_driverHID.leftTrigger().onTrue(
        new RunIntake(m_intake, m_claw)
      );
      m_driverHID.leftTrigger().onTrue(
        new RunFeed(m_feed)
      );
      m_driverHID.leftTrigger().onFalse(
        new StopIntake(m_intake)
      );
      m_driverHID.leftTrigger().onFalse(
        new StopFeed(m_feed)
      );
      m_driverHID.leftStick().onTrue(
        new ArmMidHigh(m_wrist, m_vertical, m_horizontal)
      );

  }

  

  


}
