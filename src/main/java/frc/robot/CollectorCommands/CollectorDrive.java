/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.CollectorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;

public class CollectorDrive extends CommandBase {

  double turnMultiplier;
  public CollectorDrive() {
    addRequirements(Robot.m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (OI.armJoystick.getRawButton(OI.collectorUpButton)){
      Robot.m_collector.setLifterPistonPosition(true);
    } else if (OI.armJoystick.getRawButton(OI.collectorDownButton)){
      Robot.m_collector.setLifterPistonPosition(false);
    }

    turnMultiplier = Robot.m_collector.getLifterPistonPosition() ? 1 : -1;

    if (OI.armJoystick.getRawButton(OI.collectorTakeInBallButton)){
      Robot.m_collector.setRollerMotorPower(turnMultiplier * 0.7);
    } else {
      Robot.m_collector.setRollerMotorPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
