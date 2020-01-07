/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.LimelightCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Limelight;

/**
 * Add your docs here.
 */
public class ToggleMode extends InstantCommand {
  /**
   * Add your docs here.
   */
  Limelight l = Robot.m_limelight;
  public ToggleMode() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    l.setLEDMode(l.getLEDMode() == 1 ? 0 : 1);
    l.setCamMode(l.getCamMode() == 1 ? 0 : 1);
  }

}
