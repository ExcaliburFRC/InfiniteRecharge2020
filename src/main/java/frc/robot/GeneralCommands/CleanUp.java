/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.GeneralCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
/**
 * Cleans up the {@link Shooter}, {@link Chassi}, and {@link Transporter}
 */
public class CleanUp extends InstantCommand {
  public CleanUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_chassi, Robot.m_shooter, Robot.m_transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_chassi.tankDrive(0, 0);

    Robot.m_shooter.setIsSpeedPersuit(false);
    Robot.m_shooter.setIsAnglePersuit(false);
    Robot.m_shooter.setShooterMotorPower(0);
    Robot.m_shooter.setAngleMotorPower(0);

    Robot.m_transporter.setMotorSpeed(0);
  }
}