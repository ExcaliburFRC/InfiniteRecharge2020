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
import frc.robot.Utils.CalculateVisionValues;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.ChassiCommands.PursuitTX;
import frc.robot.subsystems.Limelight;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.subsystems.LEDs.LEDMode;

public class ShootProccess extends CommandBase {
  /**
   * Creates a new ShooterDrive.
   */
  // Command shootWhenOI, setupBlock;
  CommandGroupBase shootGroup;
  PursuitTX txPursuit;
  KeepTarget setupBlock;
  boolean isAuto;
  private IdleMode originalMode;
  private double ballsShot, ballLimit;
  private boolean isCountingBalls, lastShooterSwitchStatus;

  public ShootProccess(boolean isAuto) {
    this(isAuto, -1);
  }

  public ShootProccess(boolean isAuto, double balls) {
    addRequirements(Robot.m_leds);
    this.isAuto = isAuto;

    isCountingBalls = balls != -1;
    if (isCountingBalls){
      ballLimit = balls;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_limelight.setLifterState(true);
    Robot.m_limelight.setCamMode(Limelight.CamModes.VISION);
    Robot.m_limelight.setLEDMode(Limelight.LedModes.ON);
    Robot.m_transporter.setAutoShoot(isAuto);

    DoubleSupplier shooterDistance = () -> CalculateVisionValues.calculateDistanceShooter(Robot.m_limelight.getTy());
    DoubleSupplier angleSupplier = () -> CalculateVisionValues.getOptimalShooterAngle(shooterDistance.getAsDouble());
    DoubleSupplier speedSupplier = () -> CalculateVisionValues.getOptimalShooterSpeed(shooterDistance.getAsDouble());

    txPursuit = new PursuitTX();
    setupBlock = new KeepTarget(angleSupplier, speedSupplier);
    // Command shootWhenOI = new PutInShooterWhenOI(this::isReady);

    shootGroup = setupBlock.alongWith(txPursuit);
    shootGroup.schedule();

    Robot.m_chassi.setIdleMode(IdleMode.kBrake);
    originalMode = Robot.m_chassi.getIdleMode();

    if (isCountingBalls){
      ballsShot = 0;
      lastShooterSwitchStatus = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isReady()){
      Robot.m_leds.setMode(LEDMode.GREEN);
    } else {
      Robot.m_leds.setMode(LEDMode.RED);
    }

    if (isCountingBalls){
      if (lastShooterSwitchStatus && !Robot.m_transporter.isBallInShooter()){
        ballsShot++;
      }
      lastShooterSwitchStatus = Robot.m_transporter.isBallInShooter();
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
    return false || (isCountingBalls && ballsShot == ballLimit);
  }

  public boolean isReady(){
    return txPursuit.isReady() && setupBlock.isReady();
  }
}
