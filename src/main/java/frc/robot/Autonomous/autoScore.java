package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.Wrist;
import frc.robot.Subsystems.Claw.ClawPosition;
import frc.robot.Config;

public class autoScore extends CommandBase{
    private final Wrist m_wrist;
    private final VerticalExtension m_vert;
    private final HorizontalExtension m_horiz;
    private final Claw m_claw;
    private autoScoreState currentState;
    private boolean newState;
    private autoScoreState desiredState;
    public autoScore( Wrist wristSubsystem, VerticalExtension vertExtensionSubsystem, HorizontalExtension horizExtensionSubsystem,Claw clawSubsystem){
        m_wrist = wristSubsystem;
        m_vert = vertExtensionSubsystem;
        m_horiz = horizExtensionSubsystem;
        m_claw = clawSubsystem;
        
    }
    int loops = 0;
    enum autoScoreState{
        INITIAL,
        VERTICAL,
        HORIZONTAL,
        SCORE,
        RETRACT,
        INTAKEREADY
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        currentState = autoScoreState.INITIAL;
        desiredState = autoScoreState.INITIAL;
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(currentState);
    switch(currentState){
        case INITIAL:
            m_claw.setClaw(ClawPosition.CLOSED); // CUD BE AN ERROR ?? ? DOUBLE CHECK
            if(newState){
                loops = 0;
                newState = false;
            }
            loops++;
            if(loops>10){
                desiredState = autoScoreState.VERTICAL;
                newState = true;
            }
            currentState = desiredState;
            break;
        case VERTICAL:
            m_vert.setPosition(Config.kArmHighPosY);
            if(newState){
                loops = 0;
                newState = false;
            }
            loops++;
            if(m_vert.getArmAtPosition() && loops >10){desiredState = autoScoreState.HORIZONTAL; newState = true;}
            currentState = desiredState;
            break;
        case HORIZONTAL:
            if(newState){
                loops = 0;
                newState = false;
            }
            loops++;
            m_horiz.setPosition(Config.kArmHighPosX);
            m_wrist.setWristPosition(Config.kArmHighPosWrist);
            if(m_horiz.getArmAtPosition() && loops>20){desiredState = autoScoreState.SCORE; newState = true;}
            currentState = desiredState;
            break;
        case SCORE:
            //INSERT CONSISTENT MICROADJUSTMENTS
            if(newState){
                loops = 0;
                newState = false;
            }
            loops++;
            m_claw.setClaw(ClawPosition.OPEN);
            if(m_horiz.getArmAtPosition() && loops >10){desiredState = autoScoreState.RETRACT; newState = true;}
            currentState = desiredState;
            break;
        case RETRACT:
            //FLIP BACK WRIST, RETRACT HORIZONTALLY AND VERTICALLY (maybes drive back,. unconfirmed.)
            m_wrist.setWristPosition(Config.kArmHomePosWrist);
            m_horiz.setPosition(Config.kArmHomePosX);
            m_vert.setPosition(Config.kArmHomePosY);
            if(newState){
                loops = 0;
                newState = false;
            }
            loops++;
            if(m_horiz.getArmAtPosition() && loops >15){desiredState = autoScoreState.INTAKEREADY; newState = true;}
            currentState = desiredState;
            break;
        case INTAKEREADY:
            //SHUD BE UNNECESARY
            currentState = desiredState;
            break;
    
    }



  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentState == autoScoreState.INTAKEREADY){
        return true;
    }
    return false;
    //return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
  }
}

