package frc.robot.Autonomous;

import javax.naming.TimeLimitExceededException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drive;

public class autoIntake extends CommandBase{
    private final Drive m_drive;
    public autoIntake(Drive driveSubsystem){
        m_drive = driveSubsystem;
    } 
    public void initialize() {
    }

 
}





