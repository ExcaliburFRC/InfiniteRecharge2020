/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class KeepTarget extends CommandBase {
  private DoubleSupplier angles, speeds;
  /**
   * Creates a new KeepTarget.
   */
  public KeepTarget(DoubleSupplier angleTargetSupplier, DoubleSupplier speedTargetSupplier) {
    addRequirements(Robot.m_shooter);
    angles = angleTargetSupplier;
    speeds = speedTargetSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_shooter.setIsSpeedPursuit(true);
    Robot.m_shooter.setSpeedSetpoint(speeds.getAsDouble());

    Robot.m_shooter.setIsAnglePursuit(true);
    Robot.m_shooter.setAngleSetpoint(angles.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_shooter.setSpeedSetpoint(speeds.getAsDouble());
    Robot.m_shooter.setAngleSetpoint(angles.getAsDouble());

    //chassi PID on TX 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_shooter.setIsSpeedPursuit(false);
    Robot.m_shooter.setIsAnglePursuit(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isReady(){
    return Robot.m_shooter.isOnAngle() && Robot.m_shooter.isOnSpeed();
  }
}
