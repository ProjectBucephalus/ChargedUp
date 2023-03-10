package frc.robot;

import javax.swing.plaf.InputMapUIResource;

import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public class Constants {

    /**
     * 2023 robot config
     */
    // public static final int kLeftDriveACanId = 1;
    // public static final int kLeftDriveBCanId = 3;
    // public static final int kLeftDriveCCanId = 5;
    // public static final int kRightDriveACanId = 2;
    // public static final int kRightDriveBCanId = 4;
    // public static final int kRightDriveCCanId = 6;

    /**
     * Pegasus config
     */

    public static final int kLeftDriveACanId = 1;
    public static final int kLeftDriveBCanId = 3; //3
    public static final int kLeftDriveCCanId = 5; //5
    public static final int kRightDriveACanId = 2; //2
    public static final int kRightDriveBCanId = 4; //4
    public static final int kRightDriveCCanId = 6; //6

    public static final int kVerticalElevatorCanId = 10;
    public static final int kHorizontalElevatorCanId = 9; 
    public static final int kWristMotorCanId = 12;
    //public static final int kVerticalElevatorEncoderCanId = 31;
    public static final int kHorizontalElevatorEncoderCanId = 32; 
    public static final int kWristEncoderCanId = 33;
    public static final int kLeftWingMotorCanId = 22;
    public static final int kRightWingMotorCanId = 23;


    public static final int kIntakeMotorLeftCanId = 16;
    public static final int kIntakeMotorRightCanId = 17;
    public static final int kFeedMotorsACanId = 18; //Bottom Pass-Through
    public static final int kFeedMotorsBCanId = 19; //Top Roller
    public static final int kPneumaticsModuleCanId = 51;

    public static final int kPigeonCanId = 53;

    public static final int kDriveSpeedExpo = 1; //fix later
    public static final double kDriveTurnExpo = 1;

    public static final double AutoTiltPozisionKP = -0.013;
    public static final double AutoTiltPozisionKD = -0.07; 

    public static final double kPDriveVel = 0.23386;//0.089974;//0.84839;

    public static final double kvVoltSecondsPerMeter = 2.5034;//2.4337;//2.4531;
    public static final double kaVoltSecondsSquaredPerMeter = 0.49194;//0.25671;//0.29623;//0.52078;
    public static final double ksVolts = 0.17467;//0.13306;//0.13378;
    public static final double kRamseteB = 2;//2;
    public static final double kRamseteZeta = 0.7;//.7;
    

}
