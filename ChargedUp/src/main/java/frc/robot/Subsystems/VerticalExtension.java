// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class VerticalExtension extends SubsystemBase {

    private final static WPI_TalonFX verticalExtensionMotor = new WPI_TalonFX(Constants.kVerticalElevatorCanId);
    private final CANCoder verticalExtensionEncoder = new CANCoder(Constants.kVerticalElevatorEncoderCanId);
    private double armGoal = 0;

    //this subsystem uses a combination of a feedfoward and feedback control.
    //this gives us the advantage of better profiling from feedfoward and better precision from feedback.
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        Config.kVerticalExtensionKS, Config.kVerticalExtensionKG, //kS is the static friction constant, kG is the gravity constant
        Config.kVerticalExtensionKV, Config.kVerticalExtensionKA //kV is the velocity constant, kA is the acceleration constant
      );

  /** Create a new ArmSubsystem. */
  public VerticalExtension() {
    //define a new trapezoid profile using wpi libraries. Configure the motor.
    // super(
    //     new ProfiledPIDController(
    //         Config.kVerticalExtensionKP,
    //         0.0,
    //         Config.kVerticalExtensionKD,
    //         new TrapezoidProfile.Constraints(
    //             Config.kVerticalExtensionMaxVelocity, 
    //             Config.kVerticalExtensionMaxAcceleration)), 
    //         0);
    //   initSystem();
    //   setGoal(0);
    //verticalExtensionMotor.configRemoteFeedbackFilter(verticalExtensionEncoder, 0);
    //verticalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

  }

  /**
   * Reset and configure sensors, motors, and encoders.
   */
  public void initSystem() {
    verticalExtensionMotor.configFactoryDefault(); //reset and configure the motor so we know it is correctly configured
    verticalExtensionMotor.config_kP(0, Config.kVerticalExtensionKP);
    verticalExtensionMotor.config_kD(0, Config.kVerticalExtensionKD);
    verticalExtensionMotor.config_kF(0, m_feedforward.calculate(Config.kVerticalExtensionMaxVelocity) / Config.kVerticalExtensionEncoderPPR);
    verticalExtensionMotor.config_kP(1, Config.kVerticalExtensionKP);
    verticalExtensionMotor.config_kD(1, Config.kVerticalExtensionKD);
    verticalExtensionMotor.config_kF(1, m_feedforward.calculate(Config.kVerticalExtensionMaxVelocity) / Config.kVerticalExtensionEncoderPPR);
    verticalExtensionMotor.configMotionAcceleration(Config.kVerticalExtensionMaxAcceleration / (Config.kVerticalExtensionMetresPerRotation / Config.kVerticalExtensionEncoderPPR));
    verticalExtensionMotor.configMotionCruiseVelocity(Config.kVerticalExtensionMaxVelocity / (Config.kVerticalExtensionMetresPerRotation / Config.kVerticalExtensionEncoderPPR));

    verticalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    //verticalExtensionMotor.configRemoteFeedbackFilter(verticalExtensionEncoder, 0);
    //verticalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    verticalExtensionEncoder.configFactoryDefault(); //reset and configure the encoder
    verticalExtensionEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    resetSensors();
  }

  // @Override
  // public void useOutput(double output, edu.wpi.first.math.trajectory.TrapezoidProfile.State setpoint) {
  //   // //System.out.println(setpoint.position);
  //   // // Calculate the feedforward from the sepoint
  //    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

  //   // // Add the feedforward to the PID output to get the motor output
  //   // //System.out.println(setpoint.position);
    
  //   // verticalExtensionMotor.setVoltage(output + feedforward);
  //   verticalExtensionMotor.set(getController().calculate(output) + feedforward);
  // }

  // /**
  //  * 
  //  * Run arm to position
  //  * @param kArmOffsetRads
  //  * @return when arm is at desired position
  //  */
  // public Command setArmGoalCommand(double kArmOffsetRads) {
  //   armGoal = kArmOffsetRads;
  //   return Commands.runOnce(() -> {
  //       this.setGoal(armGoal);
  //       this.enable();
  //     }, this);
  // }

   /**
   * Method to calculate the desired porition of the motor based off a target x and y position.
   * @param x desired x position
   * @param y desired y position
   * @return the desired setpoint for the extension in encoder units
   */
  public static double calculateVerticalExtensionGoal(double x, double y) {
    return x * Math.cos(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) - y * Math.sin(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) * Config.kVerticalExtensionMetresPerRotation;
  }

  /**
   * 
   * @return true if arm is within defined position tollerence.
   */
  public boolean getArmAtPosition() {
    return false;//this.getController().atGoal();
    // if(getMeasurement() <= armGoal + Config.kVerticalExtensionPositionTollerenceMetres && getMeasurement() >= armGoal - Config.kVerticalExtensionPositionTollerenceMetres) {
    //   return true;
    // }
    // return false;
  }

  //@Override
  public double getMeasurement() {
    return verticalExtensionMotor.getSelectedSensorPosition() * (Config.kVerticalExtensionMetresPerRotation / Config.kVerticalExtensionEncoderPPR);
  }

  public static void resetSensors() {
    verticalExtensionMotor.setSelectedSensorPosition(0);
  }

  public void setPosition(double position) {
    verticalExtensionMotor.set(ControlMode.MotionMagic, position / (Config.kVerticalExtensionMetresPerRotation / Config.kVerticalExtensionEncoderPPR));
  }

}
