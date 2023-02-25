// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class Feed extends SubsystemBase{
    private WPI_VictorSPX FeedMotors = new WPI_VictorSPX(Constants.kFeedMotorsCanId);
    
    public enum FeedMotorsStatus {
        ON,
        OFF
    };

    public void setFeed(FeedMotorsStatus status) {
        if (status == FeedMotorsStatus.ON) {
            FeedMotors.set(Config.kFeedMotorPower);
        } else if (status == FeedMotorsStatus.OFF) {
            FeedMotors.set(0);
        }
    }
}
