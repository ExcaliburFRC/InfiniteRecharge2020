/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.GeneralCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.ShooterCommands.KeepTarget;
import frc.robot.ShooterCommands.ShooterSetup;
import frc.robot.TransporterCommands.PutInShooterWhenOI;
import frc.robot.Utils.CalculateVisionValues;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.ChassiCommands.PursuitTX;
import frc.robot.Utils.LEDMode;

public class ManualShootProccess extends CommandBase {
  /**
   * Creates a new ShooterDrive.
   */
  // Command shootWhenOI, setupBlock;
  CommandGroupBase shootGroup;
  PursuitTX txPursuit;
  KeepTarget setupBlock;

  public ManualShootProccess() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DoubleSupplier angleSupplier = () -> CalculateVisionValues.getAngleWhenSetSpeed(Robot.m_limelight.getDistance());
    DoubleSupplier speedSupplier = () -> ShooterConstants.CONSTANT_SHOOT_SPEED;

    txPursuit = new PursuitTX();
    setupBlock = new KeepTarget(angleSupplier, speedSupplier);
    Command shootWhenOI = new PutInShooterWhenOI(this::isReady);

    shootGroup = shootWhenOI.deadlineWith(setupBlock.alongWith(txPursuit));
    shootGroup.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isReady()){
      Robot.m_leds.setMode(LEDMode.GREEN);
    } else {
      Robot.m_leds.setMode(LEDMode.RED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootGroup.cancel(); // cancels the commandGroup
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootGroup.isFinished();
  }

  public boolean isReady(){
    return txPursuit.isReady() && setupBlock.isReady();
  }
}

