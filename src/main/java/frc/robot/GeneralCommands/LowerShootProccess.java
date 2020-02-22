/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.GeneralCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.ShooterCommands.KeepTarget;
import frc.robot.ShooterCommands.ShooterDown;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.subsystems.Limelight;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.subsystems.LEDs.LEDMode;

public class LowerShootProccess extends CommandBase {
  /**
   * Creates a new ShooterDrive.
   */
  // Command shootWhenOI, setupBlock;
  CommandGroupBase shootGroup;
  KeepTarget setupBlock;
  boolean isAuto;
  private IdleMode originalMode;
  

  public LowerShootProccess(boolean isAuto) {
    addRequirements(Robot.m_leds);
    this.isAuto = isAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_limelight.setLifterState(true);
    Robot.m_limelight.setCamMode(Limelight.CamModes.VISION);
    Robot.m_limelight.setLEDMode(Limelight.LedModes.ON);
    Robot.m_transporter.setAutoShoot(isAuto);

    DoubleSupplier angleSupplier = () -> ShooterConstants.LOWER_ANGLE;
    DoubleSupplier speedSupplier = () -> ShooterConstants.LOWER_SPEED;

    setupBlock = new KeepTarget(angleSupplier, speedSupplier);

    setupBlock.schedule();

    Robot.m_chassi.setIdleMode(IdleMode.kBrake);
    originalMode = Robot.m_chassi.getIdleMode();
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
    Robot.m_limelight.setLifterState(false);
    Robot.m_limelight.setCamMode(Limelight.CamModes.DRIVING);
    Robot.m_limelight.setLEDMode(Limelight.LedModes.OFF);
    Robot.m_transporter.setAutoShoot(false);
    Robot.m_chassi.setIdleMode(originalMode);

    new ShooterDown().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isReady(){
    return setupBlock.isReady();
  }
}

