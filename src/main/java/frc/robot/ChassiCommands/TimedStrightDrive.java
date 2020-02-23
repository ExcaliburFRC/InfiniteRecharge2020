/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ChassiCommands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants.DriveConstants;
import frc.robot.Utils.RobotUtils;

public class TimedStrightDrive extends CommandBase {
  private double gyroError; //angleChange is the change in angle needed for every run
  private double forward, gyroCorrect, constAngle;
  private double originalTime, timeAmount, power;
  public boolean hasEnded;
  /**
   * Creates a new StraightDistanceDrive.
   */
  public TimedStrightDrive(double time, double power) {
    addRequirements(Robot.m_chassi);
    timeAmount = time;
    this.power = power;
    hasEnded = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    constAngle = Robot.m_chassi.getGyroAngle();
    originalTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroError = constAngle - Robot.m_chassi.getGyroAngle();
    gyroCorrect = gyroError * DriveConstants.TURN_KP;
    gyroCorrect += gyroError > 0 ? DriveConstants.TURN_AFF : -DriveConstants.TURN_AFF; 
    gyroCorrect = RobotUtils.clip(gyroCorrect, 0.4);

    if (System.currentTimeMillis() - originalTime > (timeAmount - 100)){
      Robot.m_chassi.setIdleMode(IdleMode.kBrake);
      forward = 0;
    } else {
      forward = power;
    }

    Robot.m_chassi.arcadeDrive(forward, gyroCorrect);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_chassi.arcadeDrive(0, 0);
    Robot.m_chassi.setIdleMode(IdleMode.kCoast);
    hasEnded = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - originalTime) >= timeAmount;
  }
}
