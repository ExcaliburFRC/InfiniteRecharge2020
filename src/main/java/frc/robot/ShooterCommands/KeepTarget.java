/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.subsystems.BooleanAverager;

import java.util.function.DoubleSupplier;

public class KeepTarget extends CommandBase {
  private DoubleSupplier angles, speeds;
  private double speedTolerance;
  private boolean isOnSpecialTolerance;
  private BooleanAverager speedAverager;
  /**
   * Creates a new KeepTarget.
   */
  public KeepTarget(DoubleSupplier angleTargetSupplier, DoubleSupplier speedTargetSupplier) {
    this(angleTargetSupplier, speedTargetSupplier, ShooterConstants.SPEED_TOLERANCE);
  }

  public KeepTarget(DoubleSupplier angleTargetSupplier, DoubleSupplier speedTargetSupplier, double speedTolerance) {
    addRequirements(Robot.m_shooter);
    angles = angleTargetSupplier;
    speeds = speedTargetSupplier;
    this.speedTolerance = speedTolerance;
    this.isOnSpecialTolerance = speedTolerance != ShooterConstants.SPEED_TOLERANCE;
    if (isOnSpecialTolerance){
      speedAverager = new BooleanAverager(ShooterConstants.SPEED_BUCKET_SIZE);
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_shooter.setIsSpeedPursuit(true);
    Robot.m_shooter.setSpeedSetpoint(speeds.getAsDouble());

    Robot.m_shooter.setIsAnglePursuit(true);
    Robot.m_shooter.setAngleSetpoint(angles.getAsDouble());
    
    if (isOnSpecialTolerance) speedAverager.reset();
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_shooter.setSpeedSetpoint(speeds.getAsDouble());
    Robot.m_shooter.setAngleSetpoint(angles.getAsDouble());

    if (isOnSpecialTolerance) speedAverager.update(Robot.m_shooter.isOnSpeed(speedTolerance));
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
    return Robot.m_shooter.isOnAngle() && isOnSpecialTolerance ? speedAverager.getAverage() : Robot.m_shooter.isOnSpeed();
  }
}
