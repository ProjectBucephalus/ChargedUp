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
    private WPI_VictorSPX FeedMotorsBottom = new WPI_VictorSPX(Constants.kFeedMotorsACanId);
    private WPI_VictorSPX FeedMotorsTop = new WPI_VictorSPX(Constants.kFeedMotorsBCanId);
    
    public enum FeedMotorsStatus {
        ON,
        OFF,
        REVERSE
    };

    public void setFeed(FeedMotorsStatus status) {
        if (status == FeedMotorsStatus.ON) {
            FeedMotorsBottom.set(-Config.kFeedMotorPower);
            FeedMotorsTop.set(Config.kFeedMotorPower);
        } else if (status == FeedMotorsStatus.OFF) {
            FeedMotorsBottom.set(0);
            FeedMotorsTop.set(0);
        } else if (status == FeedMotorsStatus.REVERSE){
            FeedMotorsTop.set(-Config.kFeedMotorPower);
            FeedMotorsBottom.set(Config.kFeedMotorPower);

        }
    }
}
