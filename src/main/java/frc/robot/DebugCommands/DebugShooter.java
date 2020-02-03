/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.DebugCommands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;

public class DebugShooter extends CommandBase {
  /**
   * Creates a new ShooterDrive.
   */
  public DebugShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double leftSpeed = (OI.armJoystick.getRawAxis(3)+1)/2;
      double rightSpeed = (OI.armJoystick.getRawAxis(4)+1)/2;
      
      if (OI.armJoystick.getRawButton(1)){
        Robot.m_shooter.setSpeedSetpoint(25000);
        Robot.m_shooter.setIsSpeedPersuit(true);
      } else {
        Robot.m_shooter.setIsSpeedPersuit(false);
        Robot.m_shooter.setLeftMotorSpeed(leftSpeed);
        Robot.m_shooter.setRightMotorSpeed(rightSpeed);
      }

      double angleMotorSpeed = OI.armJoystick.getRawAxis(5);
      if (OI.armJoystick.getRawButton(2)){
        Robot.m_shooter.setAngleSetpoint(45);
        Robot.m_shooter.setIsAnglePersuit(true);
      } else {
        Robot.m_shooter.setIsAnglePersuit(false);
        Robot.m_shooter.setAngleMotorPower(angleMotorSpeed);
      }

      SmartDashboard.putNumber("DEBUG_LEFTSPEED", Robot.m_shooter.getLeftMotorSpeed());
      SmartDashboard.putNumber("DEBUG_RIGHTSPEED", Robot.m_shooter.getRightMotorSpeed());
      SmartDashboard.putNumber("DEBUG_ANGLE", Robot.m_shooter.getAngle());
      SmartDashboard.putBoolean("DEBUG_isOnSpeed", Robot.m_shooter.isOnSpeed());
      SmartDashboard.putBoolean("DEBUG_isOnAngle", Robot.m_shooter.isOnAngle());
      SmartDashboard.putNumber("DEBUG_ROBOT_VOLTAGE", RobotController.getBatteryVoltage());




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
