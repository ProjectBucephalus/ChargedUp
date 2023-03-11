package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
/**
 * Put docs here // TODO
 */
public class Limelight {
    private String names;
    public Limelight(String name) {
      names = name;
    }



    public boolean getTargetAcquired() {
      //setPipeline(0);
      double targetAcquired = NetworkTableInstance.getDefault().getTable(names).getEntry("tv").getDouble(0.0);
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
      double tx = NetworkTableInstance.getDefault().getTable(names).getEntry("tx").getDouble(0.0);
      return tx;
    }

    public void disableVision() {
      setPipeline(1);
    }

    private void setPipeline(int pipelineId) {
      NetworkTableInstance.getDefault().getTable(names).getEntry("pipeline").setNumber(pipelineId);
    }
    

    public void enableVision() {
      setPipeline(0);
    }
}