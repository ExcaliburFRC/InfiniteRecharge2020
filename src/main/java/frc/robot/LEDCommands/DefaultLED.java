/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.LEDCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LEDs.LEDMode;

public class DefaultLED extends CommandBase {
  public DefaultLED() {
    addRequirements(Robot.m_leds);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.m_leds.setMode(LEDMode.BLUE);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
