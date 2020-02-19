/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.DebugCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class FuckedNavXTransport extends CommandBase {
  /**
   * Creates a new DebugTransport.
   */
  public FuckedNavXTransport() {
    addRequirements(Robot.m_transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.armJoystick.getRawButton(1) && Robot.m_shooter.isOnSpeed() && Robot.m_shooter.isOnAngle()){
      Robot.m_transporter.setTowerMotorSpeed(0.8);
      Robot.m_transporter.setLoadingMotorSpeed(-0.32);
      Robot.m_transporter.setDiagonalMotorSpeed(0.7);
    } else {
      Robot.m_transporter.setTowerMotorSpeed(0);
      Robot.m_transporter.setLoadingMotorSpeed(0);
      Robot.m_transporter.setDiagonalMotorSpeed(0);
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
