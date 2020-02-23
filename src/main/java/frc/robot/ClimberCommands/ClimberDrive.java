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
import frc.robot.subsystems.LEDs.LEDMode;

public class ClimberDrive extends CommandBase {

  public ClimberDrive() {
    addRequirements(Robot.m_climber, Robot.m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_leds.setMode(LEDMode.YELLOW);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_climber.setAbsHeightMotorSpeed(OI.armJoystick.getRawAxis(1)); //Elevator height

    if (OI.driverJoystick.getRawButton(6)){ //forward
      Robot.m_climber.setRobotClimbersPower(0.9);
    } else if (OI.driverJoystick.getRawButton(5)){ //backwards
      Robot.m_climber.setRobotClimbersPower(-0.9);
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
