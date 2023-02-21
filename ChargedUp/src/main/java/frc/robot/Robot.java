// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.ThreadPoolExecutor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.CommandController;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Utilities.Pneumatics;
import frc.robot.Utilities.Jetson.Jetson;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static CANSparkMax s1 = new CANSparkMax(1,MotorType.kBrushed);
  private static CANSparkMax s2 = new CANSparkMax(2,MotorType.kBrushed);
  private static CANSparkMax s3 = new CANSparkMax(3,MotorType.kBrushed);
  private static CANSparkMax s4 = new CANSparkMax(4,MotorType.kBrushed);
  private MotorControllerGroup left = new MotorControllerGroup(s1, s2);
  private MotorControllerGroup right = new MotorControllerGroup(s3, s4);
  private Joystick stick = new Joystick(0);
  private static Jetson jetson  = new Jetson();
  double steering;
  double power;
  double throttle;
  double yaw;
  double yawtarget;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   // steering = stick.getX();
   // power = stick.getY();
   // throttle = stick.getThrottle();
   // left.set((power - steering) * throttle);
   // right.set(-(power + steering) * throttle);
   yaw = jetson.getRobotYaw();
   System.out.println(yaw);    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

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
