// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

public class Conversion {
  /** Creates a new Conversion. */

  public static double inchesToCentimeters(double inches) {
    return inches / 2.54;
  }

  public static double inchesToMeters(double inches) {
    return inches / 254;
  }  

  public static double inchesToMillimeters(double inches) {
    return inches / 0.254;
  }

  public static double centimetersToInches(double centimeters) {
    return centimeters * 2.54;
  }

  public static double metersToInches(double meters) {
    return meters * 254;
  }  

  public static double millimetersToInches(double millimeters) {
    return millimeters * 0.254;
  }


  
}
