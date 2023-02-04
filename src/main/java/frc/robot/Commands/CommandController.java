// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Claw.CloseClaw;
import frc.robot.Commands.Claw.OpenClaw;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.StopIntake;
import frc.robot.Subsystems.Claw;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Commands.Arm.ArmHighPosCommand;
import frc.robot.Commands.Arm.ArmHomePosCommand;
import frc.robot.Commands.Arm.ArmLowPosCommand;
import frc.robot.Commands.Arm.ArmMediumPosCommand;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;

/** Add your docs here. */
public class CommandController {

    private final Drive m_drive = new Drive();
    private final Wrist m_wrist = new Wrist();
    private final HorizontalExtension m_horizontal = new HorizontalExtension();
    private final VerticalExtension m_vertical = new VerticalExtension();
    CommandJoystick m_driverJoystick = new CommandJoystick(0); //declare joystick on ds port 0 
    CommandXboxController m_driverHID = new CommandXboxController(1); //declare xbox on ds port 1



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
            () -> m_driverJoystick.getX(), () -> m_driverJoystick.getY()));
    
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


  }

  

  


}
