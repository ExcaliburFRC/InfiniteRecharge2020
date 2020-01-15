/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ChassiCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.ImageProccessingConstants;
import frc.robot.Utils.CalculateVisionValues;
import frc.robot.Utils.RobotUtils;

public class PursuitTX extends CommandBase {
  /**
   * Creates a new PersuitTX.
   */
  double error;
  double TX;
  public PursuitTX() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TX = CalculateVisionValues.getShooterTX(Robot.m_limelight.getTx(), Robot.m_limelight.getTy());
    error = (TX) * ImageProccessingConstants.TURN_KP;
    error += error > 0 ? ImageProccessingConstants.TURN_AFF : -ImageProccessingConstants.TURN_AFF; 
    error = RobotUtils.clip(error,0.75);

    if (!isReady()){
      Robot.m_chassi.arcadeDrive(0, error);
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

  public boolean isReady(){
    return ((TX) < RobotConstants.ImageProccessingConstants.TX_TOLERANCE);
  }
}
