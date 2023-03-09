package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.Drive;

public class autoClimb extends CommandBase{
    private final Drive m_drive;
    private CommandJoystick m_joy;
    public autoClimb(Drive driveSubsystem, CommandJoystick JOY){
        m_drive = driveSubsystem;
        m_joy = JOY;
    } 
    
 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
    System.out.println("Im here.");
   m_drive.autoPosition();
   }



 
 

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {

 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
    if(m_joy.getY() > 0.1){
        return true;

    }
    return false;
   //return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
 }
}
