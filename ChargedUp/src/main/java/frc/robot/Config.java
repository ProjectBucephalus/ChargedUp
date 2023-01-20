package frc.robot;

public class Config {

    /**
     * Drive characterisation values //TODO
     */
    public static final double ksVolts = 0.21511; //constant of voltage needed to overcome static friction etc.
    public static final double kvVoltSecondsPerMeter = 1.1369; //velocity constant
    public static final double kaVoltSecondsSquaredPerMeter = 0.3275;

    public static final double kPDriveVel = 0.54175; //preportional

    public static final double kTrackwidthMeters = 0.762; //distance in metres between the centre of the each set of wheels on the sides

    public static final double kMaxSpeedMetersPerSecond = 2.5915;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2; //preportional term
    public static final double kRamseteZeta = 0.7; //dampening

    public static final double wheelDiameterMetres = (6*2.54)/100; //6 inches to m
    public static final double wheelCircumferenceMetres = wheelDiameterMetres * Math.PI;
    public static final double kRatioMotorToWheel = (1/11.1/wheelCircumferenceMetres); //gearbox is 11.1:1

    public static final double kHorizontalExtensionKS = 0;
    public static final double kHorizontalExtensionKV = 0;
    public static final double kHorizontalExtensionKG = 0;
    public static final double kHorizontalExtensionKA = 0;
    public static final double kHorizontalExtensionKP = 0;
    public static final double kHorizontalExtensionMaxVelocity = 0;
    public static final double kHorizontalExtensionMaxAcceleration = 0;
    public static final double kHorizontalExtensionEncoderPPR = 4096; //CANCoder, falcon is 2048 EPR
    public static final double kHorizontalExtensionNeutralPosition = 0;
    public static final double kHorizontalExtensionEncoderOffset = 0;

    public static final double kVerticalExtensionKS = 0;
    public static final double kVerticalExtensionKV = 0;
    public static final double kVerticalExtensionKG = 0;
    public static final double kVerticalExtensionKA = 0;
    public static final double kVerticalExtensionKP = 0;
    public static final double kVerticalExtensionMaxVelocity = 0;
    public static final double kVerticalExtensionMaxAcceleration = 0;
    public static final double kVerticalExtensionEncoderPPR = 4096; //CANCoder, falcon is 2048 EPR
    public static final double kVerticalExtensionNeutralPosition = 0;
    public static final double kVerticalExtensionEncoderOffset = 0;

    public static final int kWristFowardPortId = 0;
    public static final int kWristReversePortId = 1;

    /**
     * Elevator constants
     * ALL UNITS ARE METRES (M)
     */
    public static final double kElevatorBaseWidth = 30*0.0254; //30 inches to m
    public static final double kVerticalExtensionPerpendicularHeight = 40*0.0254;
    public static final double kElevatorOffestFromFront = 0;
    public static final double kElevatorVerticalExtensionLegnth = Math.sqrt(Math.pow(kVerticalExtensionPerpendicularHeight, 2) + Math.pow(kVerticalExtensionPerpendicularHeight, 2));
   
    /**
     * Pneumatics constants
     */

    public static final double kPneumaticsMinPressure = 110; //The pressure the pneumatics will start charging at
    public static final double kPneumaticsMaxPressure = 120; //The pressure the pneumatics will stop charging at
    public static final double kCompressorRunCurrent = 10;//Ampres. Used for error checking //TODO
    public static final byte kCompressorCheckIterations = 50;//50 cycles of code. Checks that the compressor still does not run after x cycles







}
