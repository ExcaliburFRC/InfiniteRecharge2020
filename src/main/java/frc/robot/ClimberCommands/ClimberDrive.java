/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConstants.ClimbConstants;

public class ClimberDrive extends CommandBase {
  double pov;

  /**
   * Creates a new ClimberDrive.
   */
  public ClimberDrive() {
    addRequirements(Robot.m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pov = OI.armJoystick.getPOV();

    if (pov == 0){
      Robot.m_climber.setAbsHeightMotorSpeed(0.5 + ClimbConstants.AFF);
    } else if (pov == 180){
      Robot.m_climber.setAbsHeightMotorSpeed(-0.5 + ClimbConstants.AFF);
    } else {
      Robot.m_climber.setAbsHeightMotorSpeed(ClimbConstants.AFF);
    }

    if (pov == 90){
      Robot.m_climber.setRobotClimbersPower(0.5);
    } else if (pov == 270) {
      Robot.m_climber.setRobotClimbersPower(-0.5);
    } else {
      Robot.m_climber.setRobotClimbersPower(0);
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
