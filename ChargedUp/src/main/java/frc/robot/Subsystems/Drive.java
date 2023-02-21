package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{

  /*
   * Setup drive motors
   * 
   *     Front
   *  ====|  |==== 
   * ||LA      RA||
   * ||LB      RB||
   * ||LC      RC||
   * ||          ||
   * ||          ||
   *  ============
   */

    private static WPI_TalonFX leftDriveA = new WPI_TalonFX(Constants.kLeftDriveACanId);
    private static WPI_TalonFX leftDriveB = new WPI_TalonFX(Constants.kLeftDriveBCanId);
    private static WPI_TalonFX leftDriveC = new WPI_TalonFX(Constants.kLeftDriveCCanId);
    private static WPI_TalonFX rightDriveA = new WPI_TalonFX(Constants.kRightDriveACanId);
    private static WPI_TalonFX rightDriveB = new WPI_TalonFX(Constants.kRightDriveBCanId);
    private static WPI_TalonFX rightDriveC = new WPI_TalonFX(Constants.kRightDriveCCanId); 
    
    //Setup objects for use with the DifferentialDrive
    private final MotorControllerGroup leftDrive = new MotorControllerGroup(leftDriveA, leftDriveB, leftDriveC);
    private final MotorControllerGroup rightDrive = new MotorControllerGroup(rightDriveA, rightDriveB, rightDriveC);

    private final DifferentialDrive driveMotors = new DifferentialDrive(leftDrive, rightDrive);

    private static boolean brakeState = false; //Define default state for the brakes


    /**
     * Configure all motor controllers, sensors, etc. of this subsystem
     * This means that in theory, on a controller fail, all that is needed to reconfigure
     * the new ones is to change the CAN ID and it will pick up its config.
     */
    public void initSystem() {
        leftDriveA.configFactoryDefault(); //Reset all settings on the drive motors
        leftDriveB.configFactoryDefault();
        leftDriveC.configFactoryDefault();
        rightDriveA.configFactoryDefault();
        rightDriveB.configFactoryDefault();
        rightDriveC.configFactoryDefault();

        rightDriveA.setInverted(TalonFXInvertType.CounterClockwise); //Invert the right side meaning that a foward command 
        rightDriveB.setInverted(TalonFXInvertType.CounterClockwise); //will result in all motors flashing green
        rightDriveC.setInverted(TalonFXInvertType.CounterClockwise);

    }

    public Drive() {
        initSystem(); //run the above method to initalise the controllers
        setBrakes(false); //Set the falcons to coast mode on robot init
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
    return run(() -> driveMotors.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble())) //run the WPILIB arcadeDrive method with supplied values
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
            //leftDriveC.getSelectedSensorPosition()) / 3); //comment for pegasus config.
  }

  /**
   * 
   * @return right encoder distance in metres (m), averaged from all three falcons
   */
  public double getRightDriveEncodersDistanceMetres() {
    return ((rightDriveA.getSelectedSensorPosition() +
            rightDriveB.getSelectedSensorPosition()) /2 ); //+ 
           // rightDriveC.getSelectedSensorPosition()) / 3); //comment for pegasus config
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
              resetDriveEncoders(); //check encoders are zeroed
            })
        // Drive forward at specified speed
        .andThen(run(() -> driveMotors.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(getLeftDriveEncodersDistanceMetres(), getRightDriveEncodersDistanceMetres()) //check have traveled far enough
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> driveMotors.stopMotor());
  }

  /**
   * Tests if any motors are disconnected from CANBus. Will print and return if so!
   * @return true if any of the drive assosciated devices are disconnected from CAN
   * This works by requesting the falcon's firmware verison, which will return -1 if
   * the device is not on the bus
   */
  public static boolean checkCanDevices() {
    boolean canOk = true;
    if(leftDriveA.getFirmwareVersion() == -1) {
      System.out.println("WARNING leftDriveA missing from CANBus!");
      canOk = false;
    } else if(leftDriveB.getFirmwareVersion() == -1) {
      System.out.println("WARNING leftDriveB missing from CANBus!");
      canOk = false;
    } else if(leftDriveC.getFirmwareVersion() == -1) {
      System.out.println("WARNING leftDriveC missing from CANBus!");
      canOk = false;
    } else if(rightDriveA.getFirmwareVersion() == -1) {
      System.out.println("WARNING rightDriveA missing from CANBus!");
      canOk = false;
    } else if(rightDriveB.getFirmwareVersion() == -1) {
      System.out.println("WARNING rightDriveB missing from CANBus!");
      canOk = false;
    } else if(rightDriveC.getFirmwareVersion() == -1) {
      System.out.println("WARNING rightDriveC missing from CANBus!");
      canOk = false;
    }
      return canOk;
  }

  /**
   * 
   * @param brakes true for brakes enabled
   */
  public static void setBrakes(boolean brakes) {
    if(brakes) {
      leftDriveA.setNeutralMode(NeutralMode.Brake);
      leftDriveB.setNeutralMode(NeutralMode.Brake);
      leftDriveC.setNeutralMode(NeutralMode.Brake);
      rightDriveA.setNeutralMode(NeutralMode.Brake);
      rightDriveB.setNeutralMode(NeutralMode.Brake);
      rightDriveC.setNeutralMode(NeutralMode.Brake);
      brakeState = true;
    } else {
      leftDriveA.setNeutralMode(NeutralMode.Coast);
      leftDriveB.setNeutralMode(NeutralMode.Coast);
      leftDriveC.setNeutralMode(NeutralMode.Coast);
      rightDriveA.setNeutralMode(NeutralMode.Coast);
      rightDriveB.setNeutralMode(NeutralMode.Coast);
      rightDriveC.setNeutralMode(NeutralMode.Coast);
      brakeState = false;
    }
  }

  /**
   * 
   * @return true if brakes are enabled
   */
  public boolean getBrakes() {
    return brakeState; //locally updated variable to track brake state
  }

}