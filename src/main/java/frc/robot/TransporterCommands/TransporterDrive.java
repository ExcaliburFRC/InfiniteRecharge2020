package frc.robot.TransporterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConstants.TransporterConstants;
import frc.robot.subsystems.Transporter;

public class TransporterDrive extends CommandBase { 
  BallManipulationState state;
  double transportSetpoint, originalTransportLocation, transporterLocationDelta;
  boolean wasBallUnderOmni;

  public TransporterDrive() {
    addRequirements(Robot.m_transporter);
  }

  @Override
  public void initialize() {
    wasBallUnderOmni = false;
    transportSetpoint = 0;
    originalTransportLocation = Robot.m_transporter.getEncoderValue();
  }

  @Override
  public void execute() {
    state = decideState(Robot.m_transporter);
    switch (state){
      case EMPTY_SPACE_IN_TOWER: //There is space in the transport tower
        putInOmni();
        putInTower();
        break;
      case NO_SPACE_IN_TOWER: //There is no space in the tower, but there is still space under the omni-wheel
        putInOmni();
        break;
      case NO_SPACE_AT_ALL: //There is no space at all in the trasport system (there may still be space in the container)
        break;
    }

    transporterLocationDelta = Robot.m_transporter.getEncoderValue() - originalTransportLocation;
    if (isShooting()){
      Robot.m_transporter.setTowerMotorSpeed(0.4);
    } else {
      if (transporterLocationDelta < transportSetpoint - TransporterConstants.TRANSPORT_TOLERANCE){
        Robot.m_transporter.setTowerMotorSpeed(0.3);
      } else if (transporterLocationDelta > transportSetpoint - TransporterConstants.TRANSPORT_TOLERANCE){
        Robot.m_transporter.setTowerMotorSpeed(-0.3);
      } else {
        Robot.m_transporter.setTowerMotorSpeed(0);
      }
    }  
  }

  private boolean isShooting(){
    return Robot.m_transporter.getIsReady() && OI.armJoystick.getRawButton(OI.shootButtonPort);
  }

  private void putInOmni(){
    if (Robot.m_transporter.isBallInEntrance()){
      Robot.m_transporter.setLoadingMotorSpeed(0.3);
    } else {
      Robot.m_transporter.setLoadingMotorSpeed(0);
    }
  }

  private void putInTower(){
    if (Robot.m_transporter.isBallUnderOmni() && !wasBallUnderOmni){
      transportSetpoint += TransporterConstants.TRANSPORT_STEP;
    }
    wasBallUnderOmni = Robot.m_transporter.isBallUnderOmni();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private BallManipulationState decideState(Transporter t){
    if (t.getBallAmount() < 2){
      return BallManipulationState.EMPTY_SPACE_IN_TOWER;
    } else if (t.getBallAmount() >= 2 && !t.isBallUnderOmni()){
      return BallManipulationState.NO_SPACE_IN_TOWER;
    } else {
      return BallManipulationState.NO_SPACE_AT_ALL;
    }
  }
}
