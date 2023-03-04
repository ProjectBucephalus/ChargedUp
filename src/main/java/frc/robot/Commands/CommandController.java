// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
import frc.robot.Commands.Arm.ArmZeroPosCommand;
import frc.robot.Subsystems.Drive;
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
    private final HorizontalExtension m_horizontal = new HorizontalExtension();
    private final VerticalExtension m_vertical = VerticalExtension.getInstance();
    CommandJoystick m_driverJoystick = new CommandJoystick(0); //declare joystick on ds port 0 
    CommandXboxController m_driverHID = new CommandXboxController(1); //declare xbox on ds port 1
    private static PbSlewRateLimiter limiter = new PbSlewRateLimiter(new PbSlewRateLimiter.Constraints(2,.5),new PbSlewRateLimiter.State(5, 0), new PbSlewRateLimiter.State(0, 0) );
    SendableChooser<Command> chooser = SendableChooser<>();

  public CommandController(){
    configureBindings();
    chooser.addOption("2,Bottom,Climb", null);
    chooser.addOption("2,Bottom", null);
    chooser.addOption("2,Climb", null);
    chooser.addOption("8,Bottom,Climb", null);
    chooser.addOption("8,Bottom", null);
    chooser.addOption("8,Climb", null);
    chooser.addOption("5,Bottom,Climb", null);
    chooser.addOption("5,Bottom", null);
    chooser.addOption("5,Climb", null);
    chooser.addOption("5,Top,Climb", null);
    chooser.addOption("5,Top", null);


    Shuffleboard.getTab("Autonomous").add(chooser);
  }
  public Command loadPathPlannerTrajectoryToRamseteCommand(String fileName, Boolean resetOdometry){
    Trajectory traj;
    try{
      Path tracjectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
      traj = TrajectoryUtil.fromPathweaverJson(tracjectoryPath);
    }catch(IOException exception){
      DriverStation.reportError("Unable to open trajectory in " + fileName, exception.getStackTrace());
      System.out.println("Unable to read from file " + fileName );
      return new InstantCommand();
    }
    RamseteCommand ramseteCommand = new RamseteCommand(traj, drivetrainSubsystem::getpose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ks, Constants.kv,Constants.ka)
      Constants.kDriveKinematics, drivetrainSubsystem::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),  drivetrainSubsystem::tankDriveVolts,
      drivetrainSubsystem);


  
    if (resetOdometry){
      return new SequentialCommandGroup(new InstandCommand(()->drivesubstyme.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    }else{
      return ramseteCommand;
    }
  
  }

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

      m_driverHID.leftTrigger().onTrue(
        new CloseClaw(m_claw) //closes the claw when left trigger is pushed
      );

      m_driverHID.rightTrigger().onTrue(
        new OpenClaw(m_claw)  //opens claw when right trigger is pushed
      );

  }

  

  


}
