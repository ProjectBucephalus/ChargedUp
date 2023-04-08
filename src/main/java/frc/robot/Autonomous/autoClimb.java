package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.Drive;

public class autoClimb extends CommandBase{
    private final Drive m_drive;
    private CommandJoystick m_joy;
    private climbState currentState;
    private climbState desiredState;
    private double currentPitch;
    private double lastPitch;
    private double deltaPitch;
    enum climbState{
        CROSSING,
        CLIMBING
    }
    public autoClimb(Drive driveSubsystem, CommandJoystick JOY){
        m_drive = driveSubsystem;
        m_joy = JOY;
    } 
    
    @Override
    public void initialize(){
        currentState = climbState.CROSSING;
    }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
    currentPitch = m_drive.getPitch();

    switch(currentState){
        case CROSSING:
            if( currentPitch < 7 && currentPitch > 0){
                desiredState = climbState.CLIMBING;
            }else{
                m_drive.getDriveMotors().arcadeDrive(Math.abs((Math.copySign( (Math.pow(Math.abs(currentPitch * .12), .2) - 1), currentPitch))), 0);
              // m_drive.getDriveMotors().arcadeDrive(.4, 0);

            }
        break;
        case CLIMBING:
            m_drive.autoPosition();
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
    if(m_joy.getY() > 0.2 || m_joy.getX() > 0.2){
        return true;

    }
    return false;
   //return m_verticalExtension.getArmAtPosition() && m_horizontalExtension.getArmAtPosition();
 }
}
