// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.HorizontalExtension;

/** Add your docs here. */
public class CommandController {

    private final Drive m_drive = new Drive();
    private final HorizontalExtension m_horizontalEtension = new HorizontalExtension();
    private final VerticalExtension m_verticalExtension = new VerticalExtension();
    CommandJoystick m_driverJoystick = new CommandJoystick(0);
    CommandXboxController m_driverHID = new CommandXboxController(1);


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
    
    m_driverHID.a().onTrue(m_horizontalEtension.setArmGoalCommand(HorizontalExtension.calculateHorizontalExtensionGoal(1, 1)));
    m_driverHID.a().onTrue(m_verticalExtension.setArmGoalCommand(VerticalExtension.calculateVerticalExtensionGoal(1, 1)));

  }

  

  


}
