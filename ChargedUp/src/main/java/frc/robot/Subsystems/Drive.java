package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{

    private WPI_TalonFX leftDriveA = new WPI_TalonFX(Constants.kLeftDriveACanId);
    private WPI_TalonFX leftDriveB = new WPI_TalonFX(Constants.kLeftDriveBCanId);
    private WPI_TalonFX leftDriveC = new WPI_TalonFX(Constants.kLeftDriveCCanId);
    private WPI_TalonFX rightDriveA = new WPI_TalonFX(Constants.kRightDriveACanId);
    private WPI_TalonFX rightDriveB = new WPI_TalonFX(Constants.kRightDriveBCanId);
    private WPI_TalonFX rightDriveC = new WPI_TalonFX(Constants.kRightDriveCCanId); 
    
    private final MotorControllerGroup leftDrive = new MotorControllerGroup(leftDriveA, leftDriveB, leftDriveC);
    private final MotorControllerGroup rightDrive = new MotorControllerGroup(rightDriveA, rightDriveB, rightDriveC);

    private final DifferentialDrive driveMotors = new DifferentialDrive(leftDrive, rightDrive);


    public void initSystem() {
        leftDriveA.configFactoryDefault();
        leftDriveB.configFactoryDefault();
        leftDriveC.configFactoryDefault();
        rightDriveA.configFactoryDefault();
        rightDriveB.configFactoryDefault();
        rightDriveC.configFactoryDefault();

        rightDriveA.setInverted(TalonFXInvertType.CounterClockwise);
        rightDriveB.setInverted(TalonFXInvertType.CounterClockwise);
        rightDriveC.setInverted(TalonFXInvertType.CounterClockwise);

    }

    public Drive() {
        initSystem();
    }

    /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> driveMotors.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  /**
   * Resets the drive encoders to a position of zero
   */
  public void resetDriveEncoders() {

    leftDriveA.setSelectedSensorPosition(0);
    leftDriveB.setSelectedSensorPosition(0);
    leftDriveC.setSelectedSensorPosition(0);
    rightDriveA.setSelectedSensorPosition(0);
    rightDriveB.setSelectedSensorPosition(0);
    rightDriveC.setSelectedSensorPosition(0);

  }

  /**
   * 
   * @return left encoder distance in metres (m), averaged from all three falcons
   */
  public double getLeftDriveEncodersDistanceMetres() {
    return ((leftDriveA.getSelectedSensorPosition() +
            leftDriveB.getSelectedSensorPosition())/2 );// + 
            //leftDriveC.getSelectedSensorPosition()) / 3);
  }

  /**
   * 
   * @return right encoder distance in metres (m), averaged from all three falcons
   */
  public double getRightDriveEncodersDistanceMetres() {
    return ((rightDriveA.getSelectedSensorPosition() +
            rightDriveB.getSelectedSensorPosition()) /2 ); //+ 
           // rightDriveC.getSelectedSensorPosition()) / 3);
  }

  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
   */
  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              resetDriveEncoders();
            })
        // Drive forward at specified speed
        .andThen(run(() -> driveMotors.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(getLeftDriveEncodersDistanceMetres(), getRightDriveEncodersDistanceMetres())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> driveMotors.stopMotor());
  }


  

}
