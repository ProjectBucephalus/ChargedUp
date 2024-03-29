package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import frc.robot.Config;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

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

    private WPI_TalonFX leftDriveA = new WPI_TalonFX(Constants.kLeftDriveACanId);
    private WPI_TalonFX leftDriveB = new WPI_TalonFX(Constants.kLeftDriveBCanId);
    private WPI_TalonFX leftDriveC = new WPI_TalonFX(Constants.kLeftDriveCCanId);
    private WPI_TalonFX rightDriveA = new WPI_TalonFX(Constants.kRightDriveACanId);
    private WPI_TalonFX rightDriveB = new WPI_TalonFX(Constants.kRightDriveBCanId);
    private WPI_TalonFX rightDriveC = new WPI_TalonFX(Constants.kRightDriveCCanId); 
    private double lastpitch = 0;


    private WPI_Pigeon2 gyro = new WPI_Pigeon2(Constants.kPigeonCanId);
    //Setup objects for use with the DifferentialDrive
    private final MotorControllerGroup leftDrive = new MotorControllerGroup(leftDriveA, leftDriveB, leftDriveC);
    private final MotorControllerGroup rightDrive = new MotorControllerGroup(rightDriveA, rightDriveB, rightDriveC);

    public final DifferentialDrive driveMotors = new DifferentialDrive(leftDrive, rightDrive);



    public final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(.61);
   
    public DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(((gyro.getYaw()))), getLeftDriveEncodersDistanceMetres(), getRightDriveEncodersDistanceMetres()); //FIXME

    private boolean brakeState = false; //Define default state for the brakes
    private boolean directionInvert = false;

    public double getDesiredLeft(){
      return leftDrive.get();
    }
    public double getDesiredRight(){
      return rightDrive.get();
    }
    public Pose2d getPose() {
      return driveOdometry.getPoseMeters();
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
      return new DifferentialDriveWheelSpeeds(getLeftMotorVelocity(), getRightMotorVelocity());
    }
    private double getRightMotorVelocity(){
      return (((((rightDriveA.getSelectedSensorVelocity()/2048) * 8/60 * .3192))+
      (((rightDriveB.getSelectedSensorVelocity()/2048) * 8/60 * .3192)) + 
      (((rightDriveC.getSelectedSensorVelocity()/2048) * 8/60 * .3192))) / 3);
    }

    private double getLeftMotorVelocity(){
      return (((((leftDriveA.getSelectedSensorVelocity()/2048) * 8/60 * .3192))+
      (((leftDriveB.getSelectedSensorVelocity()/2048) * 8/60 * .3192)) + 
      (((leftDriveC.getSelectedSensorVelocity()/2048) * 8/60 * .3192))) / 3);
    }
    public void tankDriveVolts(double leftVolts, double rightVolts){
      leftDrive.setVoltage(leftVolts);
      rightDrive.setVoltage(rightVolts);
      driveMotors.feed();
    }
    public double getPitch(){
      return gyro.getPitch();
    }
    public void resetOdometry(Pose2d pose){
      resetDriveEncoders();
      driveOdometry.resetPosition(Rotation2d.fromDegrees(((gyro.getYaw()))), getLeftDriveEncodersDistanceMetres(), getRightDriveEncodersDistanceMetres(), pose);;
    }
    // public Pose2d proseToPose(PathPlannerState prose){
    //   Pose2d pose = new Pose2d(prose.poseMeters.getTranslation(), prose.holonomicRotation);
    //   return pose;
    // }
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

        rightDriveA.setInverted(TalonFXInvertType.CounterClockwise);
        rightDriveB.setInverted(TalonFXInvertType.CounterClockwise);
        rightDriveC.setInverted(TalonFXInvertType.CounterClockwise);

        leftDriveA.setInverted(TalonFXInvertType.Clockwise); //Invert the right side meaning that a foward command 
        leftDriveB.setInverted(TalonFXInvertType.Clockwise); //will result in all motors flashing green
        leftDriveC.setInverted(TalonFXInvertType.Clockwise);
        

      } 


    private static Drive myInstance;


    

    public static Drive getInstance()
    {
      if (myInstance == null)
      {
        myInstance = new Drive();
      }
      return myInstance;
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
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
      return run(() -> driveMotors.arcadeDrive(
        Math.copySign(Math.pow(fwd.getAsDouble(), Constants.kDriveSpeedExpo),-fwd.getAsDouble()),
        Math.copySign(Math.pow(rot.getAsDouble(), Constants.kDriveTurnExpo), -rot.getAsDouble()),
        false)) //run the WPILIB arcadeDrive method with supplied values
          .withName("arcadeDrive");
          
      }

      public Command arcadeDriveCommand(double fwd, double rot) {
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
          return run(() -> driveMotors.arcadeDrive(
            Math.copySign(Math.pow(fwd, Constants.kDriveSpeedExpo),-fwd),
            Math.copySign(Math.pow(rot, Constants.kDriveTurnExpo), -rot),
            false)) //run the WPILIB arcadeDrive method with supplied values
              .withName("arcadeDrive");

          }
  public Command autoDriveCommand() {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
      return run(() -> autoPosition());
          }


  public void brakeMotors(){
    leftDriveA.setNeutralMode(NeutralMode.Brake);
    leftDriveB.setNeutralMode(NeutralMode.Brake);
    leftDriveC.setNeutralMode(NeutralMode.Brake);
    rightDriveA.setNeutralMode(NeutralMode.Brake);
    rightDriveB.setNeutralMode(NeutralMode.Brake);
    rightDriveC.setNeutralMode(NeutralMode.Brake);
  }



  public void coastMotors(){
    leftDriveA.setNeutralMode(NeutralMode.Coast);
    leftDriveB.setNeutralMode(NeutralMode.Coast);
    leftDriveC.setNeutralMode(NeutralMode.Coast);
    rightDriveA.setNeutralMode(NeutralMode.Coast);
    rightDriveB.setNeutralMode(NeutralMode.Coast);
    rightDriveC.setNeutralMode(NeutralMode.Coast);
  }


  public void autoPosition()
  {
  double currentpitch = gyro.getPitch();
  double deltaPitch = currentpitch-lastpitch;
  driveMotors.arcadeDrive(
    ((deltaPitch * Constants.AutoTiltPozisionKD + (Math.copySign(Math.pow(Math.abs(currentpitch),.45), currentpitch) *(3.5) * Constants.AutoTiltPozisionKP)) * 1),
  0, false);
    // driveMotors.arcadeDrive(0.3, 0);
  lastpitch = currentpitch;
  }
  public void zeroGyro(){
    gyro.reset();
  }
  public void teleopPosition()
  {
  double currentpitch = gyro.getPitch();
  double deltaPitch = currentpitch-lastpitch;
  driveMotors.arcadeDrive(
    (deltaPitch * Constants.AutoTiltPozisionKD + (Math.copySign(Math.pow(Math.abs(currentpitch),.42), currentpitch) *(5.0) * Constants.AutoTiltPozisionKP)),
  0, false);
    // driveMotors.arcadeDrive(0.3, 0);
  lastpitch = currentpitch;
  }
  
  public boolean getLevel(){
    if(gyro.getPitch() == 0){
      return true;
    }
    return false;
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

  public double getThrottleInput(CommandJoystick m_joy) {
    return -((-m_joy.getThrottle() + 1) / 2);
  }

  /**
   * 
   * @return left encoder distance in metres (m), averaged from all three falcons
   */
  public double getLeftDriveEncodersDistanceMetres() {
    return (((((leftDriveA.getSelectedSensorPosition()/2048) * 8/60 * .3192))+
            (((leftDriveB.getSelectedSensorPosition()/2048) * 8/60 * .3192)) + 
            (((leftDriveC.getSelectedSensorPosition()/2048) * 8/60 * .3192))) / 3);
  }

  /**
   * 
   * @return right encoder distance in metres (m), averaged from all three falcons
   */
  public double getRightDriveEncodersDistanceMetres() {

    return (((((rightDriveA.getSelectedSensorPosition()/2048) * 8/60 * .3192))+
            (((rightDriveB.getSelectedSensorPosition()/2048) * 8/60 * .3192)) + 
            (((rightDriveC.getSelectedSensorPosition()/2048) * 8/60 * .3192))) / 3); //comment for pegasus config
  }

  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
   */
  public Command driveDistanceCommand(double distanceMeters, double speed) {
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
   * Return a compensated speed based off of the position of the 
   * @param requestedSpeeed
   * @return requested speed compensated for position of arm
   */
  public double getCompensatedDriveSpeed(double requestedSpeeed) {
    if(requestedSpeeed >= 0) {
      return (
          (
            (
              HorizontalExtension.getMeasurement() / Config.kHorizontalExtensionMaxLength
            ) * Config.kAntiTipHorizontalExtensionCompensationWeight + 
            (
              VerticalExtension.getInstance().getMeasurement() / Config.kElevatorVerticalExtensionLegnth
            ) * Config.kAntiTipVerticalExtensionCompensationWeight) / 2
          ) * Config.kAntiTipExtensionCompensationForwardModifier
          * requestedSpeeed;
    } else {
        return (
          (
            (
              HorizontalExtension.getMeasurement() / Config.kHorizontalExtensionMaxLength
            ) * Config.kAntiTipHorizontalExtensionCompensationWeight + 
            (
              VerticalExtension.getInstance().getMeasurement() / Config.kElevatorVerticalExtensionLegnth
            ) * Config.kAntiTipVerticalExtensionCompensationWeight) / 2
          ) * Config.kAntiTipExtensionCompansationReverseModifier
          * requestedSpeeed;
    }
  }

//  public double getDriveAntiTipMultiplier() {
//    if(imu.getPitch() >= Config.kAntiTipPositivePitchThreshold) {
//      return 1;//todo
//    } else if(imu.getPitch() <= Config.kAntiTipNegativePitchThreshold) {
//      return 1;//todo
//    } else {
//      return 1;
//    }
//  }

  /**
   * Tests if any motors are disconnected from CANBus. Will print and return if so!
   * @return true if any of the drive assosciated devices are disconnected from CAN
   * This works by requesting the falcon's firmware verison, which will return -1 if
   * the device is not on the bus
   */
  public boolean checkCanDevices() {
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
  public DifferentialDrive getDriveMotors(){
    return driveMotors;
  }
  /**
   * 
   * @param brakes true for brakes enabled
   */
  public void setBrakes(boolean brakes) {
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

  public void diag() {
    SmartDashboard.putBoolean("Direction swap", directionInvert);

    SmartDashboard.putNumber("left a current", leftDriveA.getStatorCurrent());
    SmartDashboard.putNumber("left b current", leftDriveB.getStatorCurrent());
    SmartDashboard.putNumber("left c current", leftDriveC.getStatorCurrent());

    SmartDashboard.putNumber("right a current", rightDriveA.getStatorCurrent());
    SmartDashboard.putNumber("right b current", rightDriveB.getStatorCurrent());
    SmartDashboard.putNumber("right c current", rightDriveC.getStatorCurrent());
    
    SmartDashboard.putNumber("left a temp", leftDriveA.getTemperature());
    SmartDashboard.putNumber("left b temp", leftDriveB.getTemperature());
    SmartDashboard.putNumber("left c temp", leftDriveC.getTemperature());

    SmartDashboard.putNumber("right a temp", rightDriveA.getTemperature());
    SmartDashboard.putNumber("right b temp", rightDriveB.getTemperature());
    SmartDashboard.putNumber("right c temp", rightDriveC.getTemperature());
   
    SmartDashboard.putNumber("right c temp", rightDriveC.getTemperature());
    SmartDashboard.putNumber("right c temp", rightDriveC.getTemperature());

  }
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    
    driveOdometry.update(
      Rotation2d.fromDegrees(gyro.getYaw()), getLeftDriveEncodersDistanceMetres(), getRightDriveEncodersDistanceMetres());
    
    
    }


  public void setDriveInvert(boolean invert) {

    directionInvert = invert;
  }

  public boolean getDriveInvert() {
    return directionInvert;
  }

  public double getYaw(){
    return gyro.getYaw();
  }

  public double getDriveDirMultiplier() {
    if(directionInvert) {
      return -1;
    } else {
      return 1;
    }
  }

}
