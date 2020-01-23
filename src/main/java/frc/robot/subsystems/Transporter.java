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
import frc.robot.TransporterCommands.TransporterDrive;
import frc.robot.Utils.RobotUtils;

public class Transporter extends SubsystemBase {
  private Spark towerMotor, loadingMotor;
  private BallDetector shooterSensor, entranceSensor, omniSensor;
  private int ballAmount = 0;
  private boolean isReady;
  private Encoder timingEncoder;

  /**
   * Creates a new Transporter.
   */
  public Transporter() {
    towerMotor = new Spark(RobotMap.TOWER_MOTOR_PORT);
    loadingMotor = new Spark(RobotMap.LOADING_MOTOR_PORT);
    entranceSensor = new MicroswitchBallDetector(RobotMap.ENTRANCE_SENSOR_PORT);
    omniSensor = new UltrasonicBallDetector(RobotMap.IN_PING_PORT, RobotMap.IN_ECHO_PORT);
    shooterSensor = new UltrasonicBallDetector(RobotMap.OUT_PING_PORT, RobotMap.OUT_ECHO_PORT);
    timingEncoder = new Encoder(RobotMap.TIMING_ENCODER_PORT1,RobotMap.TIMING_ENCODER_PORT2);
    isReady = false;
    setDefaultCommand(new TransporterDrive());
  }

  public boolean isBallInShooter(){
    return shooterSensor.isBallDetected();
  }

  public boolean isBallUnderOmni(){
    return omniSensor.isBallDetected();
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
  


  private boolean lastInStatus = false, lastOutStatus = false;
  @Override
  public void periodic() {
    if (lastInStatus && !isBallUnderOmni()){ // check if status has changed and there is a ball in the lower
        ballAmount++;
    } 
    if (lastOutStatus && !isBallInShooter()){ // check if status has changed and there is a ball in the upper
      ballAmount--;
    }

    lastInStatus = isBallUnderOmni();
    lastOutStatus = isBallInShooter();

    if(getBallAmount() < 0 || getBallAmount() > 5){// Something is terribly wrong...
      System.err.println("Something is terribly wrong here, there are " + ballAmount + " balls in the system.");
    }
  }
}