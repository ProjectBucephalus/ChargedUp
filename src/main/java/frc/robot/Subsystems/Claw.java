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
        CLOSED,
        CONEPUSH,
        ZOOOOOOOM
    };
    
    /**
     * Set claw to desired position
     * @param position
     */
    public void setClaw(ClawPosition position) {
        switch(position){
            case CLOSED:
            clawSolenoid.set(false);
            clawMotor1.setVoltage(-2.9); //1.7
            clawMotor2.setVoltage(2.9);
            break;
            case OPEN:
            clawSolenoid.set(true); //claw should be opened on default
            clawMotor1.setVoltage(0);
            clawMotor2.setVoltage(0);
            break;
            case CONEPUSH:
            clawMotor1.setVoltage(3.25);
            clawMotor2.setVoltage(-3.25);

            break;
            case ZOOOOOOOM:
            clawMotor1.setVoltage(10.25);
            clawMotor2.setVoltage(-10.25);
            break;
        }
    }
    @Override
    public void periodic(){
            //ngl not bothered to reinstate insta stall we just gonna go constant and risk a little bujrninf, graded for 2 mins anyw.

    }

}



