/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ChassiCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.Utils.RobotUtils;

public class RotateAngle extends CommandBase {
  double angleSetpoint, angleChange, error, turnPower; //angleChange is the change in angle needed for every run
  public RotateAngle(double angleRotation) {
    angleChange = angleRotation;
    addRequirements(Robot.m_chassi);
  }

  @Override
  public void initialize() {
    angleSetpoint = Robot.m_chassi.getGyroAngle() + angleChange;
  }

  @Override
  public void execute() {
    error = angleSetpoint - Robot.m_chassi.getGyroAngle();
    turnPower = error * DriveConstants.TURN_KP;
    error += error > 0 ? DriveConstants.TURN_AFF : -DriveConstants.TURN_AFF; 
    error = RobotUtils.clip(error,0.8);
    Robot.m_chassi.arcadeDrive(0, error);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.m_chassi.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(error) < DriveConstants.ANGLE_TOLERACE;
  }
}
