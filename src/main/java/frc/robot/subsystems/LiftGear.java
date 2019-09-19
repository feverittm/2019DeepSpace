/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * This subsystem manipulates the LiftGear
 */
public class LiftGear extends Subsystem {

  private Solenoid frontPiston;
  private Solenoid backPiston;
  //private AnalogInput frontDistanceIR;
  private AnalogInput backDistanceIR;
  
  public LiftGear() {
    frontPiston = new Solenoid(RobotMap.Ports.landingGearPiston);
    frontPiston.set(false);

    //frontDistanceIR = new AnalogInput(RobotMap.Ports.landingGearFloorSensor);

    backPiston = new Solenoid(RobotMap.Ports.rearGearPiston);
    backPiston.set(false);

    backDistanceIR = new AnalogInput(RobotMap.Ports.rearGearFloorSensor);
  }
  
  /**
   * Extends the single solenoid piston
   */
  public void extendFront() {
    frontPiston.set(true);
  }

  /**
   * Retracts the front landing gear (pulls it 
   * back up into the robot)
   */
  public void retractFront() {
    frontPiston.set(false);
  }

  /**
   * extent the back pistons against the floor
   * to lift the back of the robot
   */
  public void extendBack() {
    backPiston.set(true);
  }

  /**
   * retract the back pistons
   */
  public void retractBack() {
    backPiston.set(false);
  }

  /**
   * Gives you the current state of the front landing gear
   * 
   * @return True for piston is extended and False for retracted
   */
  public boolean getFrontPistonState() { return frontPiston.get(); }

  /**
   * Gives you the current state of the back pistons
   * 
   * @return True for piston is extended and False for retracted
   */
  public boolean getBackPistonState() { return backPiston.get(); }

  /**
   * Gets the voltage coming for the Infared Sensor's analog input
   * 
   * @return The voltage from the sensor. (For proto bot) Either around 1 ish for on
   * the ground or around 0.3 ish for up in the air
   */
  /*public double getFrontIRSensorVoltage() {
    //return frontDistanceIR.getVoltage();
  }*/

  public double getBackIRSensorVoltage() {
    return backDistanceIR.getVoltage();
  }

  /**
   * Updates the SmartDashboard with subsystem data
   */
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Gear/Front LiftGear State", getFrontPistonState());
    SmartDashboard.putBoolean("Gear/Back LiftGear State", getBackPistonState());
    //SmartDashboard.putNumber("Front LiftGear IR", frontDistanceIR.getVoltage());
    SmartDashboard.putNumber("Gear/Back LiftGear IR", backDistanceIR.getVoltage());
  }

  @Override
  public void initDefaultCommand() { }
}
