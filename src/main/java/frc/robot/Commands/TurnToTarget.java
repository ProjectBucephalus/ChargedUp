package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.HorizontalExtension;
import frc.robot.Subsystems.VerticalExtension;
import frc.robot.Subsystems.VerticalExtension.verticalState;
import frc.robot.Utilities.Limelight;

public class TurnToTarget extends CommandBase{
    private final Drive m_drive;
    private CommandJoystick m_joy;
    private VerticalExtension m_vert;
    private Limelight m_lime;
    private double tx;  
    private final double tuningVal = .29;
    public TurnToTarget(Drive driveSubsystem, CommandJoystick JOY, Limelight limeu, VerticalExtension vert){
        m_drive = driveSubsystem;
        m_joy = JOY;
        m_lime = limeu;
        m_vert = vert;

    } 
    
    @Override
    public void initialize(){
        if(m_vert.getState() == verticalState.MEDIUM){
            m_lime.enableBotVision();
        }else{
            m_lime.enableTopVision();
        }
    }
 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() { 
    tx = m_lime.getAngleToTarget() -8;
    var turningVal = Math.copySign(Math.pow(Math.abs(tx), 0.0525 ), tx) * tuningVal;
    m_drive.driveMotors.arcadeDrive(0, -turningVal);
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
