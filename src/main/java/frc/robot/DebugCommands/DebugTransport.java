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

public class DebugTransport extends CommandBase {
  /**
   * Creates a new DebugTransport.
   */
  public DebugTransport() {
    addRequirements(Robot.m_transporter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_transporter.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double diagonalSpeed = OI.armJoystick.getRawAxis(0);
    // double towerSpeed = OI.armJoystick.getRawAxis(1);
    double loadingSpeed = OI.armJoystick.getRawAxis(1);

    if (OI.armJoystick.getRawAxis(3) < 0){
      Robot.m_transporter.setDiagonalMotorSpeed(0.5);
      Robot.m_transporter.setTowerMotorSpeed(0.32);
    } else {
      Robot.m_transporter.setDiagonalMotorSpeed(diagonalSpeed);
      Robot.m_transporter.setTowerMotorSpeed(0);
    }
    Robot.m_transporter.setLoadingMotorSpeed(loadingSpeed);

    SmartDashboard.putNumber("DEBUG_DiagonalDistance", Robot.m_transporter.getBallDiagonalDistance());
    SmartDashboard.putBoolean("DEBUG_EntranceSensor", Robot.m_transporter.isBallInEntrance());
    SmartDashboard.putNumber("DEBUG_EntranceSensor_Distance", Robot.m_transporter.getBallEntranceDistance());
    SmartDashboard.putBoolean("DEBUG_ShooterSensor", Robot.m_transporter.isBallInShooter());
    SmartDashboard.putBoolean("DEBUG_OmniSensor", Robot.m_transporter.isBallUnderTiming());
    // SmartDashboard.putNumber("DEBUG_TowerSpeed", towerSpeed);
    SmartDashboard.putNumber("DEBUG_LoadingSpeed", loadingSpeed);
    SmartDashboard.putNumber("DEBUG_TowerEncoder", Robot.m_transporter.getEncoderValue());
    SmartDashboard.putNumber("DEBUG_BallNumber", Robot.m_transporter.getBallAmount());
    SmartDashboard.putNumber("DEBUG_DIAGONALPOWER", diagonalSpeed);

    if(OI.armJoystick.getRawButton(3)) {
      Robot.m_transporter.resetEncoder();
      Robot.m_transporter.setBallNumber(0);
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
