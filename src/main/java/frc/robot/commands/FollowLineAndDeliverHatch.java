/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class FollowLineAndDeliverHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FollowLineAndDeliverHatch() {
    addSequential(new FollowLine(1000));
    //addSequential(new Waittill(.5));
    //addSequential(new BackingUp());
}
}
