// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities.Jetson;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

/** Add your docs here. */
public class Jetson {

    NetworkTable m_table;

    public enum JetsonCamera {
        CAM0,
        CAM1,
        CAM2,
        CAM3,
    }

    public Jetson() {
        m_table = NetworkTableInstance.getDefault().getTable("Apriltag Localisation");
    }

    /**
     * Get if an AprilTag has been seen in the selected camera
     * @param jetsonCamera camera desired to be checked
     * @return if an aprilTag is visible
     */
    public boolean getTargetAquired(JetsonCamera jetsonCamera) {
        switch(jetsonCamera) {
            case CAM0: 
                return m_table.getEntry("cam0Target").getBoolean(false);
            case CAM1: 
                return m_table.getEntry("cam1Target").getBoolean(false);
            case CAM2: 
                return m_table.getEntry("cam2Target").getBoolean(false);
            case CAM3: 
                return m_table.getEntry("cam3Target").getBoolean(false);
            default:
                return false;
        }
    }

    /**
     * Get if an aprilTag has been seen in any camera
     * @return if an aprilTag is visible
     */
    public boolean getTargetAquired() {
        return getTargetAquired(JetsonCamera.CAM0) ||
            getTargetAquired(JetsonCamera.CAM1) ||
            getTargetAquired(JetsonCamera.CAM2) ||
            getTargetAquired(JetsonCamera.CAM3);
    }

    /**
     * Get target robot x
     * @return robot x pos
     */
    public double getRobotX() {
        return m_table.getEntry("robotX").getDouble(0);
    }

    /**
     * Get target robot y
     * @return robot y pos
     */
    public double getRobotY() {
        return m_table.getEntry("robotY").getDouble(0);
    }

    public double getRobotZ(){
        return m_table.getEntry("robotZ").getDouble(0);

    }
    public double getRobotYaw(){
        return m_table.getEntry("robotYaw").getDouble(0);
    }
    /**
     * Disable camera
     * @param selectedCamera to disable
     */
    public void disableCamera(JetsonCamera selectedCamera) {
        switch(selectedCamera) {
            case CAM0:
                m_table.putValue("cam0Enabled", NetworkTableValue.makeBoolean(false));
            break;
            case CAM1:
                m_table.putValue("cam1Enabled", NetworkTableValue.makeBoolean(false));
            break;
            case CAM2:
                m_table.putValue("cam2Enabled", NetworkTableValue.makeBoolean(false));
            break;
            case CAM3:
                m_table.putValue("cam3Enabled", NetworkTableValue.makeBoolean(false));
            break;
        }
    }

    /**
     * Disable camera
     * @param selectedCamera to disable
     */
    public void enableCamera(JetsonCamera selectedCamera) {
        switch(selectedCamera) {
            case CAM0:
                m_table.putValue("cam0Enabled", NetworkTableValue.makeBoolean(true));
            break;
            case CAM1:
                m_table.putValue("cam1Enabled", NetworkTableValue.makeBoolean(true));
            break;
            case CAM2:
                m_table.putValue("cam2Enabled", NetworkTableValue.makeBoolean(true));
            break;
            case CAM3:
                m_table.putValue("cam3Enabled", NetworkTableValue.makeBoolean(true));
            break;
        }
    }

     



}
