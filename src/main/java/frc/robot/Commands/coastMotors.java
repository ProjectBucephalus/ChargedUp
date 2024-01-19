package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive;

public class coastMotors extends Command{
    private final Drive m_drive;
    public coastMotors(Drive driveSubsystem){
        m_drive = driveSubsystem;
    } 
    
 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
   m_drive.coastMotors();
   }



 
 

 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {

 }

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
    return true;
   //return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
 }
}
