// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class Pneumatics extends SubsystemBase {

  private Compressor comp = new Compressor(PneumaticsModuleType.REVPH);
  private boolean compEnabled = false; //we can change this when we enable and disable the compressor. This means we can keep track of faults!
  private byte compCheckIteration = 0; //iterated once per cycle of suspected compressor fault. 
  private static Pneumatics m_instance;
  /** Creates a new Pneumatics. */
  public Pneumatics() {

  }

  public static Pneumatics getInstance() {
    if(m_instance == null) {
        m_instance = new Pneumatics();
    }
    return m_instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Enable or disable pneumatics with pressure values set from config file
   * @param boolean enable pneumatics
   */
  public void setPneumatics(boolean enabled) {
    if(enabled) {
        compEnabled = true;
        comp.enableAnalog(Config.kPneumaticsMinPressure, Config.kPneumaticsMaxPressure);
    } else {
        compEnabled = false;
        comp.disable();
    }
  }

  /**
   * Checks for potential compressor fault. Checks the following:
   *    - Compressor running
   *        - no current fault
   *    - Compressor stopped
   *        - current drawn fault
   * @return if fault detected with compressor
   */
  public boolean getCompressorCurrentFault() {
    if(compEnabled) {
        if(comp.getCurrent() <= Config.kCompressorRunCurrent) {
            compCheckIteration ++;
            if(compCheckIteration >= Config.kCompressorCheckIterations) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        if(comp.getCurrent() >= Config.kCompressorRunCurrent) {
            compCheckIteration ++;
            if(compCheckIteration >= Config.kCompressorCheckIterations) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
  }

}
