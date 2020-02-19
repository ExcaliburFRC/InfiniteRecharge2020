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
import frc.robot.Utils.CalculateVisionValues;

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
      double leftSpeed = OI.driverJoystick.getRawAxis(1);
      // double rightSpeed = OI.driverJoystick.getRawAxis(5);
      var dist = CalculateVisionValues.calculateDistanceShooter(Robot.m_limelight.getVar("ty"));

      if (OI.driverJoystick.getRawButton(1)){
        Robot.m_shooter.setSpeedSetpoint(22000);
        Robot.m_shooter.setIsSpeedPursuit(true);
      } else if (OI.driverJoystick.getRawButton(3)) {
        Robot.m_shooter.setSpeedSetpoint(CalculateVisionValues.getOptimalShooterSpeed(dist));
        Robot.m_shooter.setIsSpeedPursuit(true);
      }else {
        Robot.m_shooter.setIsSpeedPursuit(false);
        Robot.m_shooter.setLeftMotorSpeed(leftSpeed);
        Robot.m_shooter.setRightMotorSpeed(leftSpeed);
      }

      double angleMotorSpeed = OI.driverJoystick.getRawAxis(5)/5;
      if (OI.driverJoystick.getRawButton(2)){
        Robot.m_shooter.setAngleSetpoint(CalculateVisionValues.getOptimalShooterAngle(dist));
        Robot.m_shooter.setIsAnglePursuit(true);
      } else {
        Robot.m_shooter.setAngleSetpoint(Robot.m_shooter.getAngle());
        Robot.m_shooter.setIsAnglePursuit(false);
        Robot.m_shooter.setAngleMotorPower(angleMotorSpeed);
      }

      SmartDashboard.putNumber("DEBUG_LEFTSPEED", 20000 - Robot.m_shooter.getLeftMotorSpeed());
      SmartDashboard.putNumber("DEBUG_RIGHTSPEED", 20000 - Robot.m_shooter.getRightMotorSpeed());
      SmartDashboard.putNumber("DEBUG_ANGLE", Robot.m_shooter.getAngle());
      // SmartDashboard.putNumber("DEBUG_ANGLE_POWER", Robot.m_shooter.getAngleMotorPower());
      SmartDashboard.putBoolean("DEBUG_isOnSpeed", Robot.m_shooter.isOnSpeed());
      SmartDashboard.putBoolean("DEBUG_isOnAngle", Robot.m_shooter.isOnAngle());
      // SmartDashboard.putNumber("DEBUG_ROBOT_VOLTAGE", RobotController.getBatteryVoltage());
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
