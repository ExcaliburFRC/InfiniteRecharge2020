package frc.robot.TransporterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotConstants.TransporterConstants;
import frc.robot.subsystems.Transporter;

public class TransporterDrive extends CommandBase { 
  BallManipulationState state;
  double transportSetpoint, originalTransportLocation, transporterLocationDelta;
  boolean wasBallUnderOmni, waitingForUnderTiming, isInPursuit, waitForTower, readyForTower, lastUnderTimingStatus;
  double timeSinceUnderTiming;

  public TransporterDrive() {
    addRequirements(Robot.m_transporter);
  }

  @Override
  public void initialize() {
    wasBallUnderOmni = false;
    waitingForUnderTiming = false;
    transportSetpoint = 0;
    originalTransportLocation = Robot.m_transporter.getEncoderValue();
    timeSinceUnderTiming = 0;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("ballNumber", Robot.m_transporter.getBallAmount());
    state = decideState(Robot.m_transporter);
    switch (state){
      case EMPTY_SPACE_IN_TOWER: //There is space in the transport tower
        putInTower();
        putInTurn();
        diagonalMotorWithSpaceInTower();
        break;
      case NO_SPACE_IN_TOWER: //There is no space in the tower, but there is still space under the omni-wheel
        Robot.m_transporter.setDiagonalMotorSpeed(0);
        // putInTurn();
        // diagonalMotorNoSpaceInTower();
        break;
      case NO_SPACE_AT_ALL: //There is no space at all in the trasport system (there may still be space in the container)
        break;
    }

    transporterLocationDelta = Robot.m_transporter.getEncoderValue() - originalTransportLocation;
    if (isShooting()){
      Robot.m_transporter.setTowerMotorSpeed(0.8);
      transportSetpoint = transporterLocationDelta;
    } else {
      readyForTower = !waitForTower;
      if (readyForTower){
        if (transporterLocationDelta < transportSetpoint - TransporterConstants.TRANSPORT_TOLERANCE && !Robot.m_transporter.isBallInShooter()){
          Robot.m_transporter.setTowerMotorSpeed(0.32);
          isInPursuit = true;
        } else if (transporterLocationDelta > transportSetpoint + TransporterConstants.TRANSPORT_TOLERANCE){
          Robot.m_transporter.setTowerMotorSpeed(-0.32);
          isInPursuit = true;
        } else {
          Robot.m_transporter.setTowerMotorSpeed(0);
          isInPursuit = false;
        }
      }
    }  
    if (waitForTower){
      if (timeSinceUnderTiming > TransporterConstants.IN_BETWEEN_TIME){
        waitForTower = false;
      }
      timeSinceUnderTiming++;
    }
    System.out.println(Robot.m_transporter.getEncoderValue());
    // SmartDashboard.putNumber("DEBUG_timeSinceUnderTiming", timeSinceUnderTiming);
  }

  private boolean isShooting(){
    // return OI.armJoystick.getRawButton(1);
    return Robot.m_shooter.isOnSpeed() && Robot.m_shooter.isOnAngle() && OI.armJoystick.getRawButton(1); 
    // return Robot.m_transporter.getIsReady() && (OI.armJoystick.getRawButton(OI.shootButtonPort) || Robot.m_transporter.isAutoShoot());
  }

  private void putInTurn(){
    if (Robot.m_transporter.isBallInEntrance() && !(waitForTower || Robot.m_transporter.isBallUnderTiming())){
      waitingForUnderTiming = true;
    } else if (!Robot.m_transporter.isBallUnderTiming() && lastUnderTimingStatus){
      waitingForUnderTiming = false;
    }

    Robot.m_transporter.setLoadingMotorSpeed(waitingForUnderTiming? -0.32 : 0);

    lastUnderTimingStatus = Robot.m_transporter.isBallUnderTiming();
  }

  private void putInTower(){
    if (Robot.m_transporter.isBallUnderTiming() && !wasBallUnderOmni && waitingForUnderTiming){
      transportSetpoint += TransporterConstants.TRANSPORT_STEP;
      waitForTower = true;
      timeSinceUnderTiming = 0;
    }
    wasBallUnderOmni = Robot.m_transporter.isBallUnderTiming();
  }

  private void diagonalMotorWithSpaceInTower(){
    if (Robot.m_transporter.isBallInDiagonal()){
      Robot.m_transporter.setDiagonalMotorSpeed(0.5);
    } else if (waitingForUnderTiming || waitForTower) {
      Robot.m_transporter.setDiagonalMotorSpeed(0.7);
    } else {
      Robot.m_transporter.setDiagonalMotorSpeed(0);
    }
  }

  private void diagonalMotorNoSpaceInTower(){
    if (Robot.m_transporter.isBallInDiagonal() && !Robot.m_transporter.isBallUnderTiming()){
      Robot.m_transporter.setDiagonalMotorSpeed(0.5);
    } else {
      Robot.m_transporter.setDiagonalMotorSpeed(0);
    }
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
    } else if (t.getBallAmount() >= 2 && !t.isBallUnderTiming()){
      return BallManipulationState.NO_SPACE_IN_TOWER;
    } else {
      return BallManipulationState.NO_SPACE_AT_ALL;
    }
  }
}