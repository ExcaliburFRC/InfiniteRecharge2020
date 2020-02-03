/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Utils.RobotUtils;

public class Transporter extends SubsystemBase {
  private Spark towerMotor, loadingMotor, diagonalMotor;
  private BallDetector shooterSensor, entranceSensor, timingBeltSensor;
  private UltrasonicBallDetector diagonalSensor;
  private int ballAmount = 0;
  private boolean isReady;
  private Encoder timingEncoder;
  private boolean isAutoShoot;

  public Transporter() {
    towerMotor = new Spark(RobotMap.TOWER_MOTOR_PORT);
    loadingMotor = new Spark(RobotMap.LOADING_MOTOR_PORT);
    diagonalMotor = new Spark(RobotMap.DIAGONAL_MOTOR_PORT);

    entranceSensor = new MicroswitchBallDetector(RobotMap.ENTRANCE_SENSOR_PORT);
    timingBeltSensor = new MicroswitchBallDetector(RobotMap.UNDERTIMEING_SENSOR_PORT);
    shooterSensor = new UltrasonicBallDetector(RobotMap.OUT_PING_PORT, RobotMap.OUT_ECHO_PORT);
    diagonalSensor = new UltrasonicBallDetector(RobotMap.DIAGONAL_PING_PORT, RobotMap.DIAGONAL_ECHO_PORT);

    timingEncoder = new Encoder(RobotMap.TIMING_ENCODER_PORT1,RobotMap.TIMING_ENCODER_PORT2);

    isReady = false;
    isAutoShoot = false;
  }


  public void setDiagonalMotorSpeed(double s){
    diagonalMotor.set(s);
  }

  public boolean isBallInDiagonal(){
    return diagonalSensor.isBallDetected();
  }

  public double getBallDiagonalDistance(){
    return diagonalSensor.getMeasuredDistance();
  }

  public boolean isBallInShooter(){
    return shooterSensor.isBallDetected();
  }

  public boolean isBallUnderTiming(){
    return timingBeltSensor.isBallDetected();
  }

  public boolean isBallInEntrance(){
    return entranceSensor.isBallDetected();
  }
  
  public void setTowerMotorSpeed(double speed){
    towerMotor.set(RobotUtils.clip(speed, 1));
  }

  public void setLoadingMotorSpeed(double speed){
    loadingMotor.set(RobotUtils.clip(speed, 1));
  }

  public int getBallAmount() {
    return ballAmount;
  }

  public boolean getIsReady(){
    return isReady;
  }

  public void setIsReady(boolean isReady){
   this.isReady = isReady; 
  }

  public void resetEncoder(){
    timingEncoder.reset();
  }

  public double getEncoderValue(){
    return timingEncoder.get();
  }
  
  public boolean isSystemEmpty(){
    return (getBallAmount() <= 0 && !isBallInEntrance() && !isBallInShooter() && !isBallUnderTiming());
  }
  
  public void setAutoShoot(boolean isAuto){
    isAutoShoot = isAuto;
  }

  public boolean isAutoShoot(){
    return isAutoShoot;
  }

  private boolean lastInStatus = false, lastOutStatus = false;
  @Override
  public void periodic() {
    if (lastInStatus && !isBallUnderTiming()){ // check if status has changed and there is a ball in the lower
        ballAmount++;
    } 
    if (lastOutStatus && !isBallInShooter()){ // check if status has changed and there is a ball in the upper
      ballAmount--;
    }

    lastInStatus = isBallUnderTiming();
    lastOutStatus = isBallInShooter();
  }
}