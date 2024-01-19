package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.Drive;

public class teleopClimb extends Command{
    private final Drive m_drive;
    private CommandJoystick m_joy;
    public teleopClimb(Drive driveSubsystem, CommandJoystick JOY){
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
