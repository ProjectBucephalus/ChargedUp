package com.projectb.Utilities;

import com.projectb.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {
    //Internal Class Instance.
    private static Limelight m_instance;
    //The name of the limelights network table String
    private static String networkTableName;

    /**
     * Class Instantiation for unspecified limelight
     */
    public static Limelight getInstance(){
        m_instance = new Limelight();
        return m_instance;
    }

    /**
     * Class Instantiation for specified limelight
     * @param limeRefrence the name the limelight has in network tables.
     */
    public static Limelight getInstance(String limeRefrence){
        m_instance = new Limelight(limeRefrence);
        return m_instance;
    }


    //Convention.
    /**
     * Creating a limelight object
     * @param limeName used to refer to different limelights in case of using multiple. Typically named limelight, name is only specified is specifically using different names.
     * @default default limelight name is limelight, defaults to limelight if unspecified
     */
    public Limelight(){
        networkTableName = "limelight";
    }
    /**
     * Creating a limelight object
     * @param limeName used to refer to different limelights in case of using multiple. Typically named limelight, name is only specified is specifically using different names.
     */
    public Limelight(String limeName){
        networkTableName = limeName;
    }
    /**
     * Calculates the horizontal error between the Limelight center crosshair and the center of the target(s)
     * @return a double, negative being left and positive being right.
     */
    public double getXError(){
        if(boolTargetAcquired()){
            //Return networktable value for X error
            double tx = NetworkTableInstance.getDefault().getTable(networkTableName).getEntry("tx").getDouble(Constants.kLimelightDefaultValue);
            return tx;
        }
        return Constants.kLimelightDefaultValue;
    }
    /**
     * Calculates the vertical error between the Limelight center crosshair and the center of the target(s)
     * @return a double, negative being up and positive being down.
     */
    public double getYError(){
        if(boolTargetAcquired()){
            //Returns networktable value for Y error
            double ty = NetworkTableInstance.getDefault().getTable(networkTableName).getEntry("ty").getDouble(Constants.kLimelightDefaultValue);
            return ty;
        }
        return Constants.kLimelightDefaultValue;
    }

    /**
     * Checks if the limelight sees a target
     * @return an double value
     * 1.0 is a present target, 0.0 is no target found.
     */
    public double targetAcquired(){
        //Returns the networktable value for the target acquistion
        double tv = NetworkTableInstance.getDefault().getTable(networkTableName).getEntry("tv").getDouble(Constants.kLimelightDefaultValue);
        return tv;
    }
    
    /**
     * Converts the double from targetAcquired() into a boolean
     * @return A true/false statement if the limelight has spotted a target
     */
    public boolean boolTargetAcquired(){
        if(targetAcquired() == 1.0){
            return true;
        }
        return false;
    }
    /**
     * Sets pipeline number used by the Limelight
     * @param pipelineId the pipeline id from 0-9
     */
    private void setPipeline(int pipelineId) {
        //sets value to pipeline
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineId);
      }
    /**
     * Code for disabling the LED's and Vision on the robot
     */
    public void disableVision(){
        //Typically, the limelight pipeline 0 will have the default settings and will have LED's disabled in our robot. Other teams may choose to use other 
        setPipeline(Constants.kLimelightDisabledPipelineID);
    }
    /**
     * Code for enabling the limelight pipeline to be used in autonomous
     */
    public void enableVisionAutonomous(){
        //Typically pipeline 1
        setPipeline(Constants.kLimelightAutonomousPipelineID);
    }
    /**
     * Code for enabling the limelight pipeline to be used in teleop
     */
    public void enableVisionTeleop(){
        //Typically pipeline 2 unless not using differing pipelines.
        setPipeline(Constants.kLimelightTeleopPipelineID);
    }

        /**
     * Code for calculating the distance between the limelight and target by using an right angle triangle between them and the target to calculate distance. May not work on angle's of depression.
     * @param oppositeLength is the parameter for the length of the side opposite to that of the limelight. It is heavily recommended to use a constant when using this field. Often will be height of the robot minus the height of the target from the ground.
     * @param limelightAngle is the parameter for the set fixed angle of the limelight. If using a modifable hood, you need to run a calculation to find this using encoders and known values. MUST USE INCHES.
     * @param units (optional)the unit output for the limelight. Calculations are done initally in inches, as those are typically used for field measurements and simplify the math.
     * Be aware of potential conversion dissonance. Not case sensitive, select between FEET, INCHES, CENTIMETERS, METERS or MILLIMETERS in the enumeration field.
     * @return double for the distance to the limelight in the specified unit. Defaults to inches.
     */
    public double getDistanceToTarget(double oppositeLength, double limelightAngle){
        //Checks if target is visible
        if(boolTargetAcquired()){
            //Syntax Candy
            double oppAngle = oppositeLength;
            //Finds the limelight angle from ground rather then relative to the center of the limelight. This assumes that the base of your rightangle and the ground are parallel, which will always be true on a flat field.
            double orientedLimelightAngle = getYError() + limelightAngle; 
            //Converts to radian
            double orientedLimelightAngleRadians = orientedLimelightAngle * Constants.convertToRadians;
            //Calculates the adjacentangle using trig.
            double adjacentAngle = oppAngle/Math.tan(orientedLimelightAngleRadians);
            //Calculate Hypotenuse using pythagoras's theorem.
            double hypotenuse = Math.sqrt(Math.pow(oppositeLength, 2) + Math.pow(adjacentAngle, 2));
            return hypotenuse;
        }
        return Constants.kLimelightDefaultValue;
    }
        /**
     * Code for calculating the distance between the limelight and target by using an right angle triangle between them and the target to calculate distance. May not work on angle's of depression.
     * @param oppositeLength is the parameter for the length of the side opposite to that of the limelight. It is heavily recommended to use a constant when using this field. Often will be height of the robot minus the height of the target from the ground.
     * @param limelightAngle is the parameter for the set fixed angle of the limelight. If using a modifable hood, you need to run a calculation to find this using encoders and known values. MUST USE INCHES.
     * @param units (optional)the unit output for the limelight. Calculations are done initally in inches, as those are typically used for field measurements and simplify the math.
     * Be aware of potential conversion dissonance. Not case sensitive, select between FEET, INCHES, CENTIMETERS, METERS or MILLIMETERS in the enumeration field.
     * @return double for the distance to the limelight in the specified unit. Defaults to inches.
     */
    public double getDistanceToTarget(double oppositeLength, double limelightAngle, distanceUnits units){
        //Checks if target is acquired
        if(boolTargetAcquired()){
            switch(units){
                //If desiring output in inches (...)
                case INCHES:
                return getDistanceToTarget(oppositeLength, limelightAngle);
                //If desiring output in feet
                case FEET:
                return getDistanceToTarget(oppositeLength, limelightAngle) * Constants.inchesToFeetMultiplicatoryConversion;
                //If desiring output in millimetres (why)
                case MILLIMETRES:
                return getDistanceToTarget(oppositeLength, limelightAngle) * Constants.inchesToMillimetresMultiplicatoryConversion;
                //If desiring output in centimetre
                case CENTIMETRES:
                return getDistanceToTarget(oppositeLength, limelightAngle) * Constants.inchesToCentimetresMultiplicatoryConversion;
                //If desiring output in metres
                case METRES:
                return getDistanceToTarget(oppositeLength, limelightAngle) * Constants.inchesToMetresMultiplicatoryConversion;
                default:
                // output error 
                //TODO
            }
            
        }
        return Constants.kLimelightDefaultValue;
    }


    /**
     * Enumeration for the unit conversions for the limelight.
     */
    public enum distanceUnits{
        INCHES,
        FEET,
        CENTIMETRES,
        METRES,
        MILLIMETRES,
    }
}
