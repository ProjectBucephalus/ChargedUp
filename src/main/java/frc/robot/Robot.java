// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CommandController;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Utilities.Pneumatics;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Pneumatics m_pneumatics = Pneumatics.getInstance();
  private static VerticalExtension m_verticalExtension = VerticalExtension.getInstance();
  private static HorizontalExtension m_horizontalExtension = new HorizontalExtension();
  private static Command m_autonomousCommand;
   private final CommandController m_robot = new CommandController();
   private static Drive m_drive = new Drive().getInstance();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    m_verticalExtension.initSystem();
    m_horizontalExtension.initSystem();
     //setup bindings for drive, mechanisms etc.


  }




  /**
   * This f
   * 
   * 
   *  Shuffleboard.getTab("Autonomous")
   * unction is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    VerticalExtension.getInstance().checkCalibration();
    CommandScheduler.getInstance().run();
    Drive.getInstance().diag();
    //m_drive.periodic();
    SmartDashboard.putBoolean("Bottom", VerticalExtension.getInstance().getLowLimit());
    SmartDashboard.putBoolean("Top", VerticalExtension.getInstance().getHighLimit());
    SmartDashboard.putNumber("ArmPos", VerticalExtension.getInstance().getMeasurement());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Drive.getInstance().setBrakes(true); //run brakes
    m_pneumatics.setPneumatics(true);
    m_robot.getAutonomousCommand();
    m_autonomousCommand = m_robot.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  System.out.println(Drive.getInstance().getPose());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Drive.getInstance().setBrakes(false); //run brakes
    m_pneumatics.setPneumatics(true);
    m_verticalExtension.getMeasurement();


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //System.out.println(m_verticalExtension.getMeasurement());

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    Drive.getInstance().setBrakes(false); //disable brakes so robot is pushable
    m_pneumatics.setPneumatics(false);

  }
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}
