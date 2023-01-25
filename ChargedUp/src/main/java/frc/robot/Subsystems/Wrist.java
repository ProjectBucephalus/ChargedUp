// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

/** Add your docs here. */
public class Wrist extends SubsystemBase {

  public enum WristPosition {
    RAISED,
    LOWERED,
  }

  private DoubleSolenoid wristSolenoid = new DoubleSolenoid(Constants.kPneumaticsModuleCanId, PneumaticsModuleType.REVPH, Config.kWristFowardPortId, Config.kWristReversePortId);
  /** Create a new ArmSubsystem. */
  public Wrist() {
    
  }

  /**
   * Set wrist to desired position
   * @param wristPosition desired wrist position
   */
  public void setWristPosition(WristPosition wristPosition) {
    if(wristPosition == WristPosition.LOWERED) {
      wristSolenoid.set(Value.kReverse);
    } else if(wristPosition == WristPosition.RAISED) {
      wristSolenoid.set(Value.kForward);
    }
  }





}
