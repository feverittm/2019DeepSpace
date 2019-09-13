package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LockArm extends Command {

	public double m_position = Double.MAX_VALUE;
	
    public LockArm() {
    	requires(Robot.arm);
    }

    public LockArm(double position) {
      requires(Robot.arm);
      this.m_position = position;
    }
    
    protected void initialize() {
      //Robot.arm.resetPID();
      m_position = m_position == Double.MAX_VALUE ? Robot.arm.readEncoder() : m_position;
      //System.out.println("---------------------\ninitted lockArm at " + position);
      //System.out.println("read arm encoder as " + Robot.arm.readEncoder());
      //System.out.println("raw arm encoder is " + Robot.arm.getRawEncoder());
      //Robot.arm.updatePID();
      Robot.arm.setSetpoint(m_position);
    }

    protected void execute() {
      //SmartDashboard.putNumber("Arm/Arm position", m_position);
      //System.out.println("Locking Arm at position " + position + "\nArm at " + Robot.arm.readEncoder());
    }

    protected boolean isFinished() {
    	return false;
    }

    protected void end() {
      Robot.arm.setSetpoint(Robot.arm.readEncoder());
    }

    protected void interrupted() {
      //System.out.println("arm lock interrupted");
      end();
    }
}

