package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class Claw extends SubsystemBase {
    private Solenoid clawSolenoid = new Solenoid(Config.kPneumaticsModuleCanId, PneumaticsModuleType.REVPH, Config.kClawSolenoidPort);
    
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
        }else if(position == ClawPosition.CLOSED) {
            clawSolenoid.set(true); //claw should be opened on default
        }
    }


}
