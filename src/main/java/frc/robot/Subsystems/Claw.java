package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    private Solenoid clawSolenoid = new Solenoid(Config.kPneumaticsModuleCanId, PneumaticsModuleType.REVPH, Config.kClawSolenoidPort);
    private WPI_VictorSPX clawMotor1 = new WPI_VictorSPX(Constants.kLeftClawCanId);
    private WPI_VictorSPX clawMotor2 = new WPI_VictorSPX(Constants.kRightClawCanId);

    public enum ClawPosition {
        OPEN,
        CLOSED
    };
    
    /**
     * Set claw to desired position
     * @param position
     */
    public void setClaw(ClawPosition position) {
        if (position == ClawPosition.OPEN) {
            clawSolenoid.set(false);
            clawMotor1.setVoltage(-2.2);
            clawMotor2.setVoltage(2.2);

        }else if(position == ClawPosition.CLOSED) {
            clawSolenoid.set(true); //claw should be opened on default
            clawMotor1.setVoltage(-0);
            clawMotor2.setVoltage(0);

        }
    }


}
