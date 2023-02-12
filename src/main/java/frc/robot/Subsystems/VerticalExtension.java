// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    verticalExtensionMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    verticalExtensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    verticalExtensionMotor.setNeutralMode(NeutralMode.Brake);
    
    verticalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    //verticalExtensionMotor.configRemoteFeedbackFilter(verticalExtensionEncoder, 0);
    //verticalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    verticalExtensionEncoder.configFactoryDefault(); //reset and configure the encoder
    verticalExtensionEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    resetSensors();
  }

   /**
   * Method to calculate the desired porition of the motor based off a target x and y position.
   * @param x desired x position
   * @param y desired y position
   * @return the desired setpoint for the extension in encoder units
   */
  public double calculateVerticalExtensionGoal(double x, double y) {
    return Config.kElevatorBaseWidth * Math.sin(Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) + Config.kVerticalExtensionPerpendicularHeight * Math.cos(Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight));//x * Math.cos(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) - y * Math.sin(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) * Config.kVerticalExtensionMetresPerRotation;
  }

  /**
   * 
   * @return true if arm is within defined position tollerence.
   */
  public boolean getArmAtPosition() {
    if(getMeasurement() <= armGoal + Config.kVerticalExtensionPositionTollerenceMetres && getMeasurement() >= armGoal - Config.kVerticalExtensionPositionTollerenceMetres) {
      return true;
    }
    return false;
  }


  /**
   * Get position of arm
   * @return vertical extension position in metres
   */
  public static double getMeasurement() {
    return verticalExtensionMotor.getSelectedSensorPosition() / Config.kVerticalExtensionPulsesPerMetre;
  } 

  /**
   * Reset all sensors
   */
  public static void resetSensors() {
    verticalExtensionMotor.setSelectedSensorPosition(0);
  }

  /**
   * Move arm to desired position using motionMagic
   * @param position in metres
   */
  public void setPosition(double position) {
    // if(position > Config.kVerticalExtensionMaxHeight) { //Check if command is possible or not
    //   position = Config.kVerticalExtensionMaxHeight;
    // } else if(position <= 0) {
    //   position = 0;
    // }
    System.out.println("SP " + position * Config.kVerticalExtensionPulsesPerMetre);
    armGoal = position;
    verticalExtensionMotor.set(ControlMode.MotionMagic, position * Config.kVerticalExtensionPulsesPerMetre);
  }

}