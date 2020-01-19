package frc.robot.TransporterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Transporter;

public class TransporterDrive extends CommandBase { 
  BallManipulationState state;
  double beltSetpoint;

  public TransporterDrive() {
    addRequirements(Robot.m_transporter);
    beltSetpoint = 0;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    state = decideState(Robot.m_transporter);
    //TODO: switch according to the states
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private BallManipulationState decideState(Transporter t){
    if (t.getBallAmount() == 0 && t.getBallAmount() == 1){
      return BallManipulationState.EMPTY_SPACE_IN_TOWER;
    } else if (!t.isBallInEnterence()){
      return BallManipulationState.NO_SPACE_IN_TOWER;
    } else {
      return BallManipulationState.NO_SPACE_AT_ALL;
    }
  }
}
