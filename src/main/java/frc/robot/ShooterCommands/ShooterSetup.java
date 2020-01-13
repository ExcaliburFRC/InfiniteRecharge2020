/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

public class ShooterSetup extends CommandBase {
  private double angleTarget, speedTarget;
  /**
   * Creates a new ShooterDrive.
   */
  private Shooter s;
  public ShooterSetup(double angleTarget, double speedTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    s = Robot.m_shooter;
    addRequirements(Robot.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s.setAngleSetpoint(angleTarget);
    s.setIsAnglePersuit(true);

    s.setSpeedSetpoint(speedTarget);
    s.setIsSpeedPersuit(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s.isOnSpeed() && s.isOnAngle());
  }
}
