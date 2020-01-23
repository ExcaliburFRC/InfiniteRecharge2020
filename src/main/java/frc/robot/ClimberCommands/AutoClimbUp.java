/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.RobotConstants.ClimbConstants;
import frc.robot.Utils.RobotUtils;

public class AutoClimbUp extends CommandBase {
  /**
   * Creates a new AutoClimbUp.
   */
  PIDController heightController;
  double setpoint;
  double topPower;

  public AutoClimbUp() {
    addRequirements(Robot.m_climber);
    heightController = new PIDController(ClimbConstants.KP, ClimbConstants.KI, ClimbConstants.KD);
    setpoint = ClimbConstants.MAX_HEIGHT;
    topPower = 0.9;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.m_climber.getHeightEncoderValue() > ClimbConstants.MAX_HEIGHT){
      Robot.m_climber.setAbsHeightMotorSpeed(-0.25);
    } else {
      Robot.m_climber.setAbsHeightMotorSpeed(topPower - RobotUtils.clip(heightController.calculate(Robot.m_climber.getHeightEncoderValue()), topPower));
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
