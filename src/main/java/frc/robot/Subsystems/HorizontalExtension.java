// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class HorizontalExtension extends SubsystemBase {

    private final static WPI_TalonFX horizontalExtensionMotor = new WPI_TalonFX(Constants.kHorizontalElevatorCanId);
    //private final CANCoder horizontalExtensionEncoder = new CANCoder(Constants.kHorizontalElevatorEncoderCanId);
    private double armGoal = 0;

    //this subsystem uses a combination of a feedfoward and feedback control.
    //this gives us the advantage of better profiling from feedfoward and better precision from feedback.
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
        Config.kHorizontalExtensionKS, Config.kHorizontalExtensionKG, //kS is the static friction constant, kG is the gravity constant
        Config.kHorizontalExtensionKV, Config.kHorizontalExtensionKA //kV is the velocity constant, kA is the acceleration constant
      );

  /** Create a new ArmSubsystem. */
  public HorizontalExtension() {

  }

  /**
   * Reset and configure sensors, motors, and encoders.
   */
  public void initSystem() {
    horizontalExtensionMotor.configFactoryDefault(); //reset and configure the motor so we know it is correctly configured
    horizontalExtensionMotor.config_kP(0, Config.kHorizontalExtensionKP);
    horizontalExtensionMotor.config_kD(0, Config.kHorizontalExtensionKD);
    horizontalExtensionMotor.config_kF(0, m_feedforward.calculate(Config.kHorizontalExtensionMaxVelocity) / Config.kHorizontalExtensionEncoderPPR);
    horizontalExtensionMotor.config_kP(1, Config.kHorizontalExtensionKP);
    horizontalExtensionMotor.config_kD(1, Config.kHorizontalExtensionKD);
    horizontalExtensionMotor.config_kF(1, m_feedforward.calculate(Config.kHorizontalExtensionMaxVelocity) / Config.kHorizontalExtensionEncoderPPR);
    horizontalExtensionMotor.configMotionAcceleration(Config.kHorizontalExtensionMaxAcceleration / (Config.kHorizontalExtensionMetresPerRotation / Config.kHorizontalExtensionEncoderPPR));
    horizontalExtensionMotor.configMotionCruiseVelocity(Config.kHorizontalExtensionMaxVelocity / (Config.kHorizontalExtensionMetresPerRotation / Config.kHorizontalExtensionEncoderPPR));


    horizontalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    //horizontalExtensionMotor.configRemoteFeedbackFilter(horizontalExtensionEncoder, 0);
    //horizontalExtensionMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    resetSensors();

  }

   /**
   * Method to calculate the desired porition of the motor based off a target x and y position.
   * @param x desired x position
   * @param y desired y position
   * @return the desired setpoint for the extension in encoder units
   */
  public double calculateHorizontalExtensionGoal(double x, double y) {
    return x * Math.cos(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) - y * Math.sin(180 - Math.atan(Config.kElevatorBaseWidth / Config.kVerticalExtensionPerpendicularHeight)) * Config.kHorizontalExtensionMetresPerRotation;
  }

  /**
   * 
   * @return true if arm is within defined position tollerence.
   */
  public boolean getArmAtPosition() {
    if(getMeasurement() <= armGoal + Config.kHorizontalExtensionPositionTollerenceMetres && getMeasurement() >= armGoal - Config.kHorizontalExtensionPositionTollerenceMetres) {
      return true;
    }
    return false;
  }
  public boolean getIntakeLegal() {
   // System.out.println(getMeasurement());
    if(getMeasurement() <= 0.01 ){
      return true;
    }
    return false;
  }

  /**
   * Get position of arm
   * @return horizontal extension position in metres
   */
  public static double getMeasurement() {
    return horizontalExtensionMotor.getSelectedSensorPosition() * (Config.kHorizontalExtensionMetresPerRotation / Config.kHorizontalExtensionEncoderPPR);
  }

  /**
   * Reset all sensors
   */
  public static void resetSensors() {
    horizontalExtensionMotor.setSelectedSensorPosition(0);
  }


  /**
   * Move arm to desired position using motionMagic
   * @param position in metres
   */
  public void setPosition(double position) {
    armGoal = position;
    horizontalExtensionMotor.set(ControlMode.MotionMagic, -position / (Config.kHorizontalExtensionMetresPerRotation / Config.kHorizontalExtensionEncoderPPR));
  }


  public double metresToPosition(double metres){
    return metres * Constants.kHorizontalMetresToPosition;
  }


}
