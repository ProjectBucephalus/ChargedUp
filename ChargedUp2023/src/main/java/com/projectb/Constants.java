package com.projectb;

public class Constants {
    //MATH:
        /**
         * Multiply numbers by this to convert to radians
         */
        public final static double convertToRadians = (Math.PI / 180.0);
        /**
         * Can be multiplied with any number in inches to return it's length in FEET
         */
        public final static double inchesToFeetMultiplicatoryConversion = .08333333333333333333333333333333;
        /**
         * Can be multiplied with any number in inches to return it's length in Millimetres
         */
        public final static double inchesToMillimetresMultiplicatoryConversion = 25.4;

        /**
         * Can be multiplied with any number in inches to return it's length in CM
         */
        public final static double inchesToCentimetresMultiplicatoryConversion = inchesToMillimetresMultiplicatoryConversion / 10;
        /**
         * Can be multiplied with any number in inches to return it's length in Metres
         */
        public final static double inchesToMetresMultiplicatoryConversion = inchesToCentimetresMultiplicatoryConversion / 100;

        

    //LIMELIGHT:
        /**
         * Unreachable value to clearly form a distinction between a working value and a defaulted value.
        */
        public final static double kLimelightDefaultValue = 54543254354354.5435344124324267567575563243245235;
        /**
         * The pipeline id for running the limelight without any LEDs.
         */
        public final static int kLimelightDisabledPipelineID = 0;
        /**
         * The pipeline id for running the limelight during autonomous.
         */
        public final static int kLimelightAutonomousPipelineID = 1;
        /**
         * The pipeline id for running the limelight during teleop.
         */
        public final static int kLimelightTeleopPipelineID = 2;

        /**
         * Angle of elevation for limelight from the ground
         */
        public final static double kLimeLightMountedAngle = 47.5;
        
        /**
         * Height of limelight from ground 
         */
        public final static double kLimelightHeight = 55;
    //Field Dimensions
        /**
         * Upper hub height from ground
         */
        public final static double kLengthToUpperHub = 100;
    //VISION:
        /**
         * Vision turn tuning number
         */
        public final static double kVisionTurnKp = 0.39;
}
