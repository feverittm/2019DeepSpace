/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.data;

/**
 * Add your docs here.
 */
public class ArmData {
  private static ArmData instance;

  public static double output, current, ticks, velocity, angle;
  public static boolean front, back;

  public static ArmData getInstance() {
    if (instance == null) {
      instance = new ArmData();
    }
    return instance;
  }

}
