/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.TransporterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConstants.TransporterConstants;

public class FuckedNavXTransport extends CommandBase {
  /**
   * Creates a new DebugTransport.
   */
  double timeSinceTop;
  boolean wasInTop;

  public FuckedNavXTransport() {
    addRequirements(Robot.m_transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSinceTop = 0;
    wasInTop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var isOkToShoot = (Robot.m_transporter.isAutoShoot() || OI.armJoystick.getRawButton(1)) && Robot.m_shooter.isOnAngle() && Robot.m_shooter.isOnSpeed();
    var isInNoReturnMode = (System.currentTimeMillis() - timeSinceTop) < TransporterConstants.STOP_TIME;
    var isInSlowSpeedTime = (System.currentTimeMillis() - timeSinceTop) < TransporterConstants.END_SLOW_TIME && (System.currentTimeMillis() - timeSinceTop) > TransporterConstants.STOP_TIME;

    if (isOkToShoot && !isInNoReturnMode){
      var transportSpeed = isInSlowSpeedTime ? 0.35 : 0.6;
      Robot.m_transporter.setTowerMotorSpeed(transportSpeed);
      Robot.m_transporter.setLoadingMotorSpeed(-0.32);
      Robot.m_transporter.setDiagonalMotorSpeed(1); //it used to work with .7
    } else {
      Robot.m_transporter.setTowerMotorSpeed(0);
      Robot.m_transporter.setLoadingMotorSpeed(0);
      Robot.m_transporter.setDiagonalMotorSpeed(0);
    }

    if (!wasInTop && Robot.m_transporter.isBallInShooter()){
      timeSinceTop = System.currentTimeMillis();
    }

    wasInTop = Robot.m_transporter.isBallInShooter();
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
