package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
/**
 * Put docs here // TODO
 */
public class Limelight {
    private static Limelight mInstance;

    public static Limelight getInstance() {
      if (mInstance == null) {
        mInstance = new Limelight();
      }
      return mInstance;
    }

    Limelight() {
      //setPipeline(0);
    }



    public boolean getTargetAcquired() {
      //setPipeline(0);
      double targetAcquired = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
      boolean yesorno;
      if(targetAcquired == 1.0){
        yesorno = true;
      }
      else{
        yesorno = false;
      }
      return yesorno; 
    }

    public double getAngleToTarget() {
      //setPipeline(0);
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
      return tx;
    }

    public void disableVision() {
      setPipeline(1);
    }

    private void setPipeline(int pipelineId) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineId);
    }


    public void enableVision() {
      setPipeline(0);
    }
}