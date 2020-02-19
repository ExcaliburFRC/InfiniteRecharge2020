/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Utils.RobotUtils;

public class Transporter extends SubsystemBase {
  private VictorSPX towerMotor, loadingMotor, diagonalMotor;
  private BallDetector shooterSensor, timingBeltSensor;
  private UltrasonicBallDetector diagonalSensor, entranceSensor;
  private int ballAmount = 0;
  private boolean isReady;
  private Encoder timingEncoder;
  private boolean isAutoShoot;

  private BooleanAverager timingBooleanAvarager, shooterSensorAvarager;

  public Transporter() {
    towerMotor = new VictorSPX(RobotMap.TOWER_MOTOR_PORT);
    loadingMotor = new VictorSPX(RobotMap.LOADING_MOTOR_PORT);
    diagonalMotor = new VictorSPX(RobotMap.DIAGONAL_MOTOR_PORT);
    diagonalMotor.setInverted(true);

    entranceSensor = new UltrasonicBallDetector(RobotMap.ENTRANCE_PING_PORT, RobotMap.ENTRANCE_ECHO_PORT, 30);
    timingBeltSensor = new MicroswitchBallDetector(RobotMap.UNDERTIMEING_SENSOR_PORT);
    shooterSensor = new MicroswitchBallDetector(RobotMap.OUT_MICROSWITCH);
    diagonalSensor = new UltrasonicBallDetector(RobotMap.DIAGONAL_PING_PORT, RobotMap.DIAGONAL_ECHO_PORT, 52);

    timingEncoder = new Encoder(RobotMap.TIMING_ENCODER_PORTS[0],RobotMap.TIMING_ENCODER_PORTS[1]);

    // entranceSensor.setAuto(true);
    diagonalSensor.setAuto(true);

    isReady = false;
    isAutoShoot = false;

    timingBooleanAvarager = new BooleanAverager(2);
    shooterSensorAvarager = new BooleanAverager(3);
  }

  public double getBallEntranceDistance(){
    return entranceSensor.getMeasuredDistance();
  }

  public void setDiagonalMotorSpeed(double s){
    diagonalMotor.set(ControlMode.PercentOutput, s);
  }

  public boolean isBallInDiagonal(){
    return diagonalSensor.isBallDetected();
  }

  public void setBallNumber(double num){
    ballAmount = 0;
  }

  public double getBallDiagonalDistance(){
    return diagonalSensor.getMeasuredDistance();
  }

  public boolean getRawShooterSensor(){
    return !shooterSensor.isBallDetected();
  }

  public boolean isBallInShooter(){
    return shooterSensorAvarager.getAverage();
  }

  public boolean isBallUnderTiming(){
    return timingBooleanAvarager.getAverage();
  }
  
  public boolean getRawUnderTiming(){
    return !timingBeltSensor.isBallDetected();
  }

  public boolean isBallInEntrance(){
    return entranceSensor.isBallDetected();
  }
  
  public void setTowerMotorSpeed(double speed){
    towerMotor.set(ControlMode.PercentOutput, RobotUtils.clip(speed, 1));
  }

  public void setLoadingMotorSpeed(double speed){
    loadingMotor.set(ControlMode.PercentOutput, RobotUtils.clip(speed, 1));
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

    timingBooleanAvarager.update(getRawUnderTiming());
    shooterSensorAvarager.update(getRawShooterSensor());

    // System.out.println(getEncoderValue());
  }
}