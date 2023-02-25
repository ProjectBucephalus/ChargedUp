// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class Intake extends SubsystemBase{
    private Solenoid intakeSolenoid = new Solenoid(Config.kPneumaticsModuleCanId, PneumaticsModuleType.REVPH, Config.kIntakeSolenoidPort);
    private WPI_VictorSPX intakeMotor1 = new WPI_VictorSPX(Constants.kIntakeMotorLeftCanId);
    private WPI_VictorSPX intakeMotor2 = new WPI_VictorSPX(Constants.kIntakeMotorRightCanId);

    public enum IntakePosition {
        EXTENDED,
        RETRACTED
    };

    public enum IntakeMotorStatus {
        ON,
        OFF
    };

    public void setIntake(IntakePosition position) {
        if (position == IntakePosition.EXTENDED) {
            intakeSolenoid.set(true);
        }else if(position == IntakePosition.RETRACTED) {
            intakeSolenoid.set(false);
        }
    }

    public void setWheels(IntakeMotorStatus status) {
        if (status == IntakeMotorStatus.ON) {
            intakeMotor1.set(Config.kIntakeMotorPower);
            intakeMotor2.set(Config.kIntakeMotorPower);
        }else if(status == IntakeMotorStatus.OFF) {
            intakeMotor1.set(0);
            intakeMotor2.set(0);
        }
    }
}
