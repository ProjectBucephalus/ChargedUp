package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.VerticalExtension.verticalState;
import frc.robot.Utilities.Limelight;

public class MoveHorizontalExtension extends CommandBase{
    private CommandJoystick m_joy;
    private Limelight m_lime;
    private HorizontalExtension m_horiz;
    private VerticalExtension m_vert;
    private double distToTarget;  

    public MoveHorizontalExtension(CommandJoystick JOY, Limelight limeu, HorizontalExtension horiz, VerticalExtension vert){
        m_joy = JOY;
        m_lime = limeu;
        m_horiz = horiz;
        m_vert = vert;
    } 
    
    @Override
    public void initialize(){
        if(m_vert.getState() == verticalState.MEDIUM){
            m_lime.enableBotVision();
        }else{
            m_lime.enableTopVision();
        }    }
 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() { 
    distToTarget = m_lime.getHorizontalDistance();
    System.out.println(distToTarget);
    if(distToTarget != Constants.kLimelightErrorValue){
        var setPos = distToTarget - 0.7;
        setPos *= Constants.kHorizontalMetresToPosition;
        System.out.println(setPos);

        if(setPos < Config.kArmHighPosX + .03 && setPos > Config.kArmLowPosX - .02){
        m_horiz.setPosition(setPos);
        }
    }
}
 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
    m_lime.disableVision();
 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
    if(Math.abs(m_joy.getX()) > 0.3 || Math.abs(m_joy.getY()) > 0.3){
        return true;
    }
    return false;
   //return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
 }
}
