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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class Wings extends SubsystemBase {

    private final static WPI_TalonFX leftWingMotor = new WPI_TalonFX(Constants.kLeftWingMotorCanId);
    private final static WPI_TalonFX rightWingMotor = new WPI_TalonFX(Constants.kRightWingMotorCanId);

    private double armGoal = 0;

    //this subsystem uses a combination of a feedfoward and feedback control.
    //this gives us the advantage of better profiling from feedfoward and better precision from feedback.
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        Config.kWingKS, Config.kWingKG, //kS is the static friction constant, kG is the gravity constant
        Config.kWingKV, Config.kWingKA //kV is the velocity constant, kA is the acceleration constant
      );

  /** Create a new ArmSubsystem. */
  public Wings() {

  }

  /**
   * Reset and configure sensors, motors, and encoders.
   */
  public void initSystem() {
    leftWingMotor.configFactoryDefault(); //reset and configure the motor so we know it is correctly configured
    leftWingMotor.config_kP(0, Config.kWingKP);
    leftWingMotor.config_kD(0, Config.kWingKD);
    leftWingMotor.config_kF(0, m_feedforward.calculate(Config.kWingMaxVelocity) / Config.kWingEncoderPPR);
    leftWingMotor.config_kP(1, Config.kWingKP);
    leftWingMotor.config_kD(1, Config.kWingKD);
    leftWingMotor.config_kF(1, m_feedforward.calculate(Config.kWingMaxVelocity) / Config.kWingEncoderPPR);
    leftWingMotor.configMotionAcceleration(Config.kWingMaxAcceleration / (Config.kWingMetresPerRotation / Config.kWingEncoderPPR));
    leftWingMotor.configMotionCruiseVelocity(Config.kWingMaxVelocity / (Config.kWingMetresPerRotation / Config.kWingEncoderPPR));
    leftWingMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    leftWingMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    leftWingMotor.setNeutralMode(NeutralMode.Brake);
    
    leftWingMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    //leftWingMotor.configRemoteFeedbackFilter(wingEncoder, 0);
    //leftWingMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    rightWingMotor.configFactoryDefault(); //reset and configure the motor so we know it is correctly configured
    rightWingMotor.config_kP(0, Config.kWingKP);
    rightWingMotor.config_kD(0, Config.kWingKD);
    rightWingMotor.config_kF(0, m_feedforward.calculate(Config.kWingMaxVelocity) / Config.kWingEncoderPPR);
    rightWingMotor.config_kP(1, Config.kWingKP);
    rightWingMotor.config_kD(1, Config.kWingKD);
    rightWingMotor.config_kF(1, m_feedforward.calculate(Config.kWingMaxVelocity) / Config.kWingEncoderPPR);
    rightWingMotor.configMotionAcceleration(Config.kWingMaxAcceleration / (Config.kWingMetresPerRotation / Config.kWingEncoderPPR));
    rightWingMotor.configMotionCruiseVelocity(Config.kWingMaxVelocity / (Config.kWingMetresPerRotation / Config.kWingEncoderPPR));
    rightWingMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rightWingMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rightWingMotor.setNeutralMode(NeutralMode.Brake);
    
    rightWingMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    

    resetSensors();
  }

  /**
   * 
   * @return true if arm is within defined position tollerence.
   */
  public boolean getArmAtPosition() {
    if(getMeasurement() <= armGoal + Config.kWingPositionTollerenceMetres && getMeasurement() >= armGoal - Config.kWingPositionTollerenceMetres) {
      return true;
    }
    return false;
  }


  /**
   * Get position of arm
   * @return wing extension position in metres
   */
  public static double getMeasurement() {
    return (leftWingMotor.getSelectedSensorPosition() + rightWingMotor.getSelectedSensorPosition()) /2 / Config.kWingPulsesPerMetre;
  } 

  /**
   * Reset all sensors
   */
  public static void resetSensors() {
    leftWingMotor.setSelectedSensorPosition(0);
    rightWingMotor.setSelectedSensorPosition(0);

  }

  /**
   * Move arm to desired position using motionMagic
   * @param position in metres
   */
  public void setPosition(double position) {
    // if(position > Config.kWingMaxHeight) { //Check if command is possible or not
    //   position = Config.kWingMaxHeight;
    // } else if(position <= 0) {
    //   position = 0;
    // }
    System.out.println("SP " + position * Config.kWingPulsesPerMetre);
    armGoal = position;
    leftWingMotor.set(ControlMode.MotionMagic, position * Config.kWingPulsesPerMetre);
    rightWingMotor.set(ControlMode.MotionMagic, position * Config.kWingPulsesPerMetre);

  }

}
