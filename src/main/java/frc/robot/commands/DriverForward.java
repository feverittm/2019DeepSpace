/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriverForward extends Command {
  DriveTrain driveTrain;
  double speed;
  double duration;
  long start;

  public DriverForward(double speed, double duration, DriveTrain driveTrain) {
   this.driveTrain = driveTrain;
   this.duration = duration;
   this.speed = speed;
  }

  public DriverForward(double speed, double duration){
    this.driveTrain = new DriveTrain();
    this.duration = duration;
    this.speed = speed;

  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveTrain.setVolts(speed, speed);
    start = System.currentTimeMillis();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if ((start + duration)< System.currentTimeMillis()){
      return true;
    }else{
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
