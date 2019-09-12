/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoRetractLandingGear extends Command {

  public AutoRetractLandingGear() {
    requires(Robot.liftGear);
  }
  
  @Override
  protected void initialize() { }
  
  @Override
  protected void execute() {

    // If the IR sensor detects the platform
    /*if (Robot.liftGear.getFrontIRSensorVoltage() > 0.95) {
      Robot.liftGear.retractFront();
    }*/

    if (Robot.liftGear.getBackIRSensorVoltage() > 0.95) {
      Robot.liftGear.retractBack();
    }
  }
  
  @Override
  protected boolean isFinished() {
   return !Robot.liftGear.getFrontPistonState();
  }
  @Override
  protected void end() { }
  
  @Override
  protected void interrupted() { end(); }
}
