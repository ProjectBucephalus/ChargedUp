package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drive;

public class autoClimb extends CommandBase{
    private final Drive m_drive;
    public autoClimb(Drive driveSubsystem){
        m_drive = driveSubsystem;
    } 
    
}
