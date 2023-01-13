package frc.robot;

public class Config {

    /**
     * Drive characterisation values //TODO
     */
    public static final double ksVolts = 0.21511;
    public static final double kvVoltSecondsPerMeter = 1.1369;
    public static final double kaVoltSecondsSquaredPerMeter = 0.3275;

    public static final double kPDriveVel = 0.54175;

    public static final double kTrackwidthMeters = 0.762;

    public static final double kMaxSpeedMetersPerSecond = 2.5915;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double wheelDiameterMetres = (6*2.54)/100;
    public static final double wheelCircumferenceMetres = wheelDiameterMetres * Math.PI;
    public static final double kRatioMotorToWheel = (1/11.1/wheelCircumferenceMetres);



}
