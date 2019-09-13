/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 *
 */
public class MoveElevator extends Command {
	public double m_value;
	
	public MoveElevator(double _value) {
    	requires(Robot.elevator);
    	this.m_value = _value;
    }
    protected void initialize() {
    }
    protected void execute() {
    	if (Robot.elevator.GetPosition() >= RobotMap.Values.elevatorTopHeight && m_value > 0) {
            Scheduler.getInstance().add(new LockElevator());
        }
        else if (Robot.elevator.GetPosition() <= 0 && m_value < 0) {
            Scheduler.getInstance().add(new LockElevator());
    	} else {
    		Robot.elevator.SetPower(m_value);
    	}
    }
    protected boolean isFinished() {
    	return false;
    }
    protected void end() {
    }

    protected void interrupted() {
        System.out.println("Elevator interrupted");
    	//end();
    }
}