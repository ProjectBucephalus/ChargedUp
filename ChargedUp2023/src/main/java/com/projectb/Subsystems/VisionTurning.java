package com.projectb.Subsystems;

import java.lang.annotation.Target;

import com.projectb.Utilities.Limelight;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer.InterpolateFunction;

import com.projectb.Constants;

public class VisionTurning {
    //Vision instance
    private static VisionTurning m_instance;
    //Limelight Instance
    private static Limelight m_lime;

    /**
     * Class Instantiation for unspecified limelight
     */
    public static VisionTurning getInstance() {
        m_instance = new VisionTurning();
        return m_instance;
      }
    /**
     * Creating a vision instance
     * @param limeRefrence used to refer to different limelights in case of using multiple. Typically named limelight, name is only specified is specifically using different names.
     */
      private VisionTurning(){
        m_lime = new Limelight();
    }
    /**
     * Calculates the speed at which to turn the robot.
     * @return the steering to be put onto arcade drive to adjust it's positon aiming at the shooter
     */
    public double getSteering(){

        // regress this lol. curvefit.java :)
        //returns the x error
        double xError = m_lime.getXError();
        
        double drivetune = xError * Constants.kVisionTurnKp * -1;
        if(xError > 8){
            return drivetune * 1.5;
        } else if(xError > 4){
            return drivetune * 2;
        } else if(xError > 2.5){
            return drivetune * 4;
        } else if(xError > 1.5){
            return drivetune * 4.5;
        } else if(xError > 0.5){
            return drivetune * 6.5 - 2;
        }  else if(xError >= 0){
            return drivetune * 10 - 3;
        }  else if(xError < -8){
            return drivetune * 1.5;
        } else if(xError < -4){
            return drivetune * 2;
        } else if(xError < -2.5){
            return drivetune * 4;
        } else if(xError < -1.5){
            return drivetune * 4.5;
        } else if(xError < -0.5){
            return drivetune * 6.5 + 2;
        }  else if (xError <0){
            return drivetune * 10 + 3;
        }
        return 0.0;
    }

    /**
     * Using the curve fitting polynomial, it will configure the estimated shooter speed inputed. Curvefit.java will intergrate into the code, allowing for the polynomial to be adjusted 'on the fly' and for potential auto-tuning.
     * @return the speed to set the motors to to hit the target most accurately
     */
    public double getShooterSpeed(){
        if(m_lime.boolTargetAcquired()){
            double dist = m_lime.getDistanceToTarget(Constants.kLengthToUpperHub - Constants.kLimelightHeight, Constants.kLimeLightMountedAngle);
            double speed = dist; // equation idk. ill look into it later depending on how curvefit.java progresses.
            return speed;
        }
        return 0.0;
    }
}

