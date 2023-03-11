// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import java.io.IOException;
import java.nio.file.Path;

import java.util.HashMap;
import java.util.function.BiConsumer;

import frc.robot.Autonomous.autoClimb;
import frc.robot.Autonomous.autoIntake;
import frc.robot.Autonomous.autoScore;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import edu.wpi.first.networktables.NetworkTableInstance; // we live in a society....


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Autonomous.autoScore;
import frc.robot.Commands.Arm.ArmHighPosCommand;
import frc.robot.Commands.Arm.ArmHomePosCommand;
import frc.robot.Commands.Arm.ArmLowPosCommand;
import frc.robot.Commands.Arm.ArmMediumPosCommand;
import frc.robot.Commands.Arm.ArmMidHigh;
import frc.robot.Commands.Arm.ArmZeroPosCommand;
import frc.robot.Commands.Arm.intakeArm;
import frc.robot.Commands.Claw.CloseClaw;
import frc.robot.Commands.Claw.OpenClaw;
import frc.robot.Commands.Intake.RunFeed;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.StopFeed;
import frc.robot.Commands.Intake.StopIntake;
import frc.robot.Commands.Intake.outTake;
import frc.robot.Commands.Intake.reverseFeed;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Feed;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;
import frc.robot.Utilities.Limelight;
import frc.robot.Utilities.PbSlewRateLimiter;
import frc.robot.Utilities.PbSlewRateLimiter.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

    private Limelight m_lime = new Limelight("limelight");


    SendableChooser<Command> chooser = new SendableChooser<>();

  public CommandController(){
    configureBindings();
    //chooser.addOption("2,Bottom,Climb", loadPathPlannerTrajectoryToRamseteCommand("2BOTTOMClimb", true));
    //chooser.addOption("2,Bottom", loadPathPlannerTrajectoryToRamseteCommand("2BOTTOM", true));
    chooser.addOption("8,Climb", loadPathPlannerTrajectoryToRamseteCommand("2Climb", true,2.6,.9));
    //chooser.addOption("8,TOP,Climb", loadPathPlannerTrajectoryToRamseteCommand("8TOPClimb", true));
    //chooser.addOption("8,TOP", loadPathPlannerTrajectoryToRamseteCommand("8TOP", true));
    chooser.addOption("2,Climb", loadPathPlannerTrajectoryToRamseteCommand("8Climb", true,2.6,.9));
    //chooser.addOption("5,Bottom,Climb", loadPathPlannerTrajectoryToRamseteCommand("5BOTTOMClimb", true));
    //chooser.addOption("5,Bottom", loadPathPlannerTrajectoryToRamseteCommand("5BOTTOM", true));
    chooser.addOption("5,Climb", loadPathPlannerTrajectoryToRamseteCommand("5Climb", true,.42,.375));
    //chooser.addOption("5,Top,Climb", loadPathPlannerTrajectoryToRamseteCommand("5TOPClimb", true));
    //chooser.addOption("5,Top", loadPathPlannerTrajectoryToRamseteCommand("5TOP", true));
    chooser.addOption("Start at 3, cone, climb", loadPathPlannerTrajectoryToRamseteCommand("3ConeClimb", true, 2.5, .85));
    chooser.addOption("Start at 7, cone, climb", loadPathPlannerTrajectoryToRamseteCommand("7ConeClimb", true, 2.5, .85));
    chooser.addOption("Start at 7, cone, cross line", loadPathPlannerTrajectoryToRamseteCommand("7Cone", true, 2.5, .85));
    chooser.addOption("Start at 3, cone, cross line", loadPathPlannerTrajectoryToRamseteCommand("3Cone", true, 2.5, .85));
    chooser.addOption("Start at 2, cube, cross line", loadPathPlannerTrajectoryToRamseteCommand("2Cube", true, 2.5, .85));
    chooser.addOption("Start at 8, cube, cross line", loadPathPlannerTrajectoryToRamseteCommand("8Cube", true, 2.5, .85));

    chooser.addOption("Move back in a line", loadPathPlannerTrajectoryToRamseteCommand("LuinStinks", true,1.2,.5));
    
    chooser.setDefaultOption("nuth n", loadPathPlannerTrajectoryToRamseteCommand("LuinStinks", true,0,0));

    Shuffleboard.getTab("Autonomous").add(chooser);
  }
  public Command loadPathPlannerTrajectoryToRamseteCommand(String fileName, Boolean resetOdometry,double maxvel, double maxacel){
    PathPlannerTrajectory traj;
    try{
      traj = PathPlanner.loadPath(fileName, new PathConstraints(maxvel, maxacel));
    }finally{
      System.out.println("haii :P");
    }

    RamseteController ramseteController = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);



    PIDConstants pidConstants = new PIDConstants(Constants.kPDriveVel, 0, 0);
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);
    
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("scoreGamepiece", new autoScore(m_wrist, m_vertical, m_horizontal, m_claw)); //STUB FUNCTION !
    //eventMap.put("intake", new autoIntake(m_drive)); //STUB FUNCTION!!
    eventMap.put("climb", new autoClimb(m_drive, m_driverJoystick)); //STUB FUCINGIOTON
    RamseteAutoBuilder ramseteAuto = new RamseteAutoBuilder(
      m_drive::getPose, 
      m_drive::resetOdometry,
      ramseteController, 
      m_drive.driveKinematics,
      feedForward,
      m_drive::getWheelSpeeds,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
      pidConstants,
      (leftVolts, rightVolts) -> {
      m_drive.tankDriveVolts(leftVolts, rightVolts);},
      eventMap, 
      true,
      m_drive);
  

  
    if (resetOdometry){
      return new SequentialCommandGroup( new InstantCommand(() -> Drive.getInstance().resetOdometry(m_drive.proseToPose(traj.getInitialState())                           )), ramseteAuto.fullAuto(traj));
        
    }else{
      return ramseteAuto.fullAuto(traj);
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
            () -> -m_driverJoystick.getY() * m_drive.getDriveDirMultiplier() * m_drive.getThrottleInput(m_driverJoystick),
            () -> -m_driverJoystick.getX() * m_drive.getDriveDirMultiplier() * m_drive.getThrottleInput(m_driverJoystick)));
    
    
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

      m_driverHID.rightStick().onTrue(new outTake(m_intake, m_claw));

      m_driverHID.rightStick().onFalse(new StopIntake(m_intake));
      m_driverHID.rightStick().onTrue(new reverseFeed(m_feed));

      m_driverHID.rightStick().onFalse(new StopFeed(m_feed));

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
      //m_driverJoystick.button(1).onTrue();
      m_driverJoystick.button(5).onTrue(
        new TurnToTarget(m_drive, m_driverJoystick, m_lime)
      );
// 
     //  m_driverJoystick.pov(0).onTrue(
      //   new InvertDirectionCommand(m_drive, false)
       //);
// 
      // 
      // m_driverJoystick.pov(180).onTrue(
        // new InvertDirectionCommand(m_drive, true)
      // );

  }

  
  public Command getAutonomousCommand(){
        return chooser.getSelected();
  }
  


}
