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
import frc.robot.subsystems.BooleanAverager;

public class StraightDistanceDrive extends CommandBase {
  private final BooleanAverager errorAverager;
  private double gyroError; //angleChange is the change in angle needed for every run
  private double forward, gyroCorrect, constAngle, distance, distanceSetpoint;
  private double distanceError, startPos;
  /**
   * Creates a new StraightDistanceDrive.
   */
  public StraightDistanceDrive(double distance) {
    addRequirements(Robot.m_chassi);
    errorAverager = new BooleanAverager(50);
    this.distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = Robot.m_chassi.getAverageEncoderDistance();
    distanceSetpoint = startPos + distance;
    constAngle = Robot.m_chassi.getGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroError = constAngle - Robot.m_chassi.getGyroAngle();
    gyroCorrect = gyroError * DriveConstants.TURN_KP;
    gyroCorrect += gyroError > 0 ? DriveConstants.TURN_AFF : -DriveConstants.TURN_AFF; 
    gyroCorrect = RobotUtils.clip(gyroCorrect, 0.4);

    distanceError = distanceSetpoint - Robot.m_chassi.getAverageEncoderDistance();
    forward = distanceError * DriveConstants.DISTANCE_KP;
    // forward += distanceError > DriveConstants.DISTANCE_TOLERANCE ? DriveConstants.DISTANCE_AFF : 0;
    forward = RobotUtils.clip(forward, 0.5);
    errorAverager.update(Math.abs(distanceError) < DriveConstants.DISTANCE_TOLERANCE);

    Robot.m_chassi.arcadeDrive(forward, gyroCorrect);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_chassi.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return errorAverager.getAverage();
  }
}
