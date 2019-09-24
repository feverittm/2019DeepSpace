/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Scheduler;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Used for moving the elevator high enough to swap the arm's side if so required.
 */
public class ElevatorToArmHeight extends Command {
  private double m_target = RobotMap.Values.armSwitchHeight + 2250;
  private double tolerance = RobotMap.Values.elevatorHeightAbsTolerance;

  public ElevatorToArmHeight() {
    requires(Robot.elevator);
    //this.tolerance = tolerance;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.SetPosition(m_target);
    Robot.elevator.enable();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return (Math.abs(RobotMap.Values.armSwitchHeight - Robot.elevator.GetPosition()) < tolerance);
    return (Robot.elevator.GetPosition() >= (m_target - tolerance));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("finished arm to height");
    Scheduler.getInstance().add(new LockElevator());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
