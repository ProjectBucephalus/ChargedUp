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
    private ClawPosition currentPos = ClawPosition.OPENSTALL;
    public enum ClawPosition {
        OPENUNSTALL,
        OPENSTALL,
        CLOSED,
        CONEPUSH,
        ZOOOOOOOM
    };
    
    /**
     * Set claw to desired position
     * @param position
     */
    public void setClaw(ClawPosition position) {
        if (position == ClawPosition.OPENSTALL) {
            clawSolenoid.set(false);
            clawMotor1.setVoltage(-2.9); //1.7
            clawMotor2.setVoltage(2.9);
            currentPos = ClawPosition.OPENSTALL;
        } else if (position == ClawPosition.OPENUNSTALL) {
            clawSolenoid.set(false);
            clawMotor1.setVoltage(-9.5);
            clawMotor2.setVoltage(9.5);
            currentPos = ClawPosition.OPENUNSTALL;
        }else if(position == ClawPosition.CLOSED) {  //THIS IS ACTUALLY HOPEMN OR SOMETHING...
            clawSolenoid.set(true); //claw should be opened on default
            clawMotor1.setVoltage(0);
            clawMotor2.setVoltage(0);
            currentPos = ClawPosition.CLOSED;
        }else if(position == ClawPosition.CONEPUSH){
            clawMotor1.setVoltage(3.25);
            clawMotor2.setVoltage(-3.25);
            currentPos = ClawPosition.CONEPUSH;
        }else if(position == ClawPosition.ZOOOOOOOM){
            clawMotor1.setVoltage(10.25);
            clawMotor2.setVoltage(-10.25);
            currentPos = ClawPosition.ZOOOOOOOM;
        }
    }
    @Override
    public void periodic(){
        if((checkMotorStall(clawMotor1) && currentPos == ClawPosition.OPENUNSTALL)|| (checkMotorStall(clawMotor2) && currentPos == ClawPosition.OPENUNSTALL)){
            setClaw(ClawPosition.OPENSTALL);
        };

    }

    private boolean checkMotorStall(WPI_VictorSPX motoror){
       // if(motoror.get)
       return false;
    }
}



