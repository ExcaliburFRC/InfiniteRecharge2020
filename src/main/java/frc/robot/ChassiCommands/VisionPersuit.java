/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ChassiCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassi;
import frc.robot.subsystems.Limelight;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotUtils;

public class VisionPersuit extends CommandBase {
  Chassi c = Robot.m_chassi;
  Limelight l = Robot.m_limelight;

  double error;

  double forward;
  double lastTime;

  boolean ended;
  double endTimes;

  public VisionPersuit() {
    addRequirements(c);

    endTimes = 750;
    lastTime = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = (l.getTx()-0.2) * RobotConstants.ImageProccessing.TURN_KP;
    error += error > 0 ? RobotConstants.ImageProccessing.TURN_AFF : -RobotConstants.ImageProccessing.TURN_AFF; 
    SmartDashboard.putNumber("Error", System.currentTimeMillis());
    error = RobotUtils.clip(error,0.65);

    if (l.getTv() == 1) {
      lastTime = System.currentTimeMillis();
    }

    if (l.getTv() == 1 || System.currentTimeMillis() - lastTime <= endTimes){
      forward = -0.625;
    } else {
      forward = 0;
    }

    c.arcadeDrive(forward, error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    c.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - lastTime > endTimes);
  }
}
