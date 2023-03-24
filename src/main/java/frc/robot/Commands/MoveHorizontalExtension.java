package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.VerticalExtension.verticalState;
import frc.robot.Utilities.Limelight;

public class MoveHorizontalExtension extends CommandBase{
    private final Drive m_drive;
    private CommandJoystick m_joy;
    private Limelight m_lime;
    private HorizontalExtension m_horiz;
    private VerticalExtension m_vert;
    private double tx;  
    private final double tuningVal = .3;
    private double distToTarget;  

    public MoveHorizontalExtension(Drive driveSubsystem, CommandJoystick JOY, Limelight limeu, HorizontalExtension horiz, VerticalExtension vert){
        m_drive = driveSubsystem;
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
    if(distToTarget != Constants.kLimelightErrorValue){
        if(distToTarget * Constants.kHorizontalMetresToPosition < Config.kArmHighPosX + .02 && distToTarget * Constants.kHorizontalMetresToPosition > Config.kArmLowPosX + .02){
        m_horiz.setPosition(distToTarget);
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
    if(Math.abs(m_joy.getX()) > 0.1 || Math.abs(m_joy.getY()) > 0.1){
        return true;
    }
    return false;
   //return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
 }
}
