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
import frc.robot.RobotConstants.ImageProccessingConstants;
import frc.robot.Utils.*;

public class FeederPersuit extends CommandBase {
  Chassi c = Robot.m_chassi;
  Limelight l = Robot.m_limelight;

  double error;

  double forward;
  double lastTime;

  boolean ended;

  public FeederPersuit() {
    addRequirements(c);
  }

  @Override
  public void initialize() {
    lastTime = System.currentTimeMillis();
    Robot.m_limelight.setCamMode(Limelight.CamModes.VISION);
    Robot.m_limelight.setLifterState(false);
  }

  @Override
  public void execute() {
    error = CalculateVisionValues.getShooterTX(Robot.m_limelight.getTx(), Robot.m_limelight.getTy()) * ImageProccessingConstants.TURN_KP;
    error += error > 0 ? ImageProccessingConstants.TURN_AFF : -ImageProccessingConstants.TURN_AFF; 
    error = RobotUtils.clip(error,0.65);

    if (l.getTv() == 1) {
      lastTime = System.currentTimeMillis();
    }

    forward = CalculateVisionValues.calculateDistanceFeeder(Robot.m_limelight.getTa()) * ImageProccessingConstants.FORWARD_KP + ImageProccessingConstants.FORWARD_AFF;

    c.arcadeDrive(forward, error);
  }

  @Override
  public void end(boolean interrupted) {
    c.arcadeDrive(0, 0);
    Robot.m_limelight.setCamMode(Limelight.CamModes.DRIVING);
    Robot.m_limelight.setLifterState(false);
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - lastTime > 500);
  }
}
