// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Commands.Wings.DeployWings;
import frc.robot.Commands.Wings.LiftWings;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Wings;

/** Add your docs here. */
public class CommandController {

    private final Drive m_drive = new Drive();
    private final Wings m_wings = new Wings();
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

    m_driverHID.pov(180).onTrue(new DeployWings(m_wings)); //Check for wing deploy button
    m_driverHID.pov(0).onTrue(new LiftWings(m_wings)); //Check for wing lift button


  }

  

  


}
