package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Subsystems.Drive;

public class autoClimb extends CommandBase{
    private final Drive m_drive;
    private CommandJoystick m_joy;
    private climbState currentState;
    private climbState desiredState;
    private double currentPitch;
    private double initialPitch;
    private double deltaPitch;
    private double currentYaw;
    private double initialYaw;
    private double targetYaw;
    private double time;
    private boolean weCross;
    private boolean gotPast;
    private boolean timepassed;
    private boolean newState = true;
    enum climbState{
        CROSSING,
        CROSSED,
        CLIMBING, 
        TURNED
    }
    public autoClimb(Drive driveSubsystem, CommandJoystick JOY){
        m_drive = driveSubsystem;
        m_joy = JOY;
    } 
    
    @Override
    public void initialize(){
        currentState = climbState.CROSSING;
        desiredState = climbState.CROSSING;
        initialPitch = m_drive.getPitch();
         //m_drive.zeroGyro();
        gotPast = false;
        time = 0;
        timepassed = false;
    }

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
    currentPitch = m_drive.getPitch();
    currentYaw = m_drive.getYaw();
    System.out.println(currentState);
    switch(currentState){
        case CROSSING:
            if(time + .375 < Timer.getFPGATimestamp() && gotPast == true){
                desiredState = climbState.CROSSED;
                newState = true;
            }else if( currentPitch < 9 && currentPitch > 2 && gotPast == false){
                time = Timer.getFPGATimestamp();
                gotPast = true;
            }else{
                //m_drive.getDriveMotors().arcadeDrive(Math.abs((Math.copySign( (Math.pow(Math.abs(currentPitch * .12), .2) - 1), currentPitch))), 0);
               m_drive.getDriveMotors().arcadeDrive(.675, 0);
               newState = false;
            }
            currentState = desiredState;
        break;
        case CROSSED:
            if(newState){
                initialYaw = m_drive.getYaw();
                targetYaw = initialYaw + 160;
                m_drive.getDriveMotors().arcadeDrive(0, 0);
                newState = false;

            }
            double yawError = currentYaw - targetYaw;
            if (Math.abs(yawError) < 4){
                desiredState = climbState.TURNED;
                System.out.println("T");
                newState = true;
            }
            else{
                // if(Math.abs((currentYaw - targetYaw)) <= 30){
                //     m_drive.getDriveMotors().arcadeDrive(0, .00003 * Math.pow(Math.abs(currentYaw- targetYaw), 2) +.35);
                // }   else{
                    m_drive.getDriveMotors().arcadeDrive(0,.55);
                    // }
            }
            currentState = desiredState;
            break;
        case TURNED:
            if(newState){
                time = Timer.getFPGATimestamp();
            }
            if( currentPitch > -10 && currentPitch < -6){
                desiredState = climbState.CLIMBING;
                newState = true;
            }else if(timepassed){
                //m_drive.getDriveMotors().arcadeDrive(Math.abs((Math.copySign( (Math.pow(Math.abs(currentPitch * .12), .2) - 1), currentPitch))), 0);
            m_drive.getDriveMotors().arcadeDrive(.6, 0);
            }else if(time + 0.5 < Timer.getFPGATimestamp()){
                timepassed = true;
            }
            newState = false;
            currentState = desiredState;
        break;
        case CLIMBING:
            m_drive.autoPosition();
            currentState = desiredState;
            newState = false;
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
