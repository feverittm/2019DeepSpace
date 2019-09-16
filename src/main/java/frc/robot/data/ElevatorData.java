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
public class ElevatorData {

  private static ElevatorData instance;

  public static double output, current, velocity, height, setpoint;
  public static boolean bottom, top;

  public static ElevatorData getInstance() {
    if (instance == null) {
      instance = new ElevatorData();
    }
    return instance;
  }
}
