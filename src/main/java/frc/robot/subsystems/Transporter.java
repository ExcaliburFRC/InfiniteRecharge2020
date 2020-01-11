/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotConstants.TransporterConstants;
import frc.robot.TransporterCommands.TransporterDrive;
import frc.robot.Utils.RobotUtils;

public class Transporter extends SubsystemBase {
  private Spark motor;
  private Ultrasonic shooterSensor, collectorSensor;
  private int ballAmount = 0;

  /**
   * Creates a new Transporter.
   */
  public Transporter() {
    super();
    motor = new Spark(RobotMap.MOTOR_PORT);
    collectorSensor = new Ultrasonic(RobotMap.IN_PING_PORT, RobotMap.IN_ECHO_PORT);
    shooterSensor = new Ultrasonic(RobotMap.OUT_PING_PORT, RobotMap.OUT_ECHO_PORT);

    setDefaultCommand(new TransporterDrive());
  }

  public double getCollectorDistance(){
    return collectorSensor.getRangeMM() / 10;
  }

  public double getShooterDistance(){
    return shooterSensor.getRangeMM() / 10;
  }

  public boolean isBallInCollector(){
    return (getCollectorDistance() < TransporterConstants.BALL_DETECTION_TOLARANCE);
  }
  public boolean isBallInShooter(){
    return (getShooterDistance() < TransporterConstants.BALL_DETECTION_TOLARANCE);
  }
  
  public void setMotorSpeed(double speed){
    motor.set(RobotUtils.clip(speed, 1));
  }

  /**
   * @return the ballAmount
   */
  public int getBallAmount() {
    return ballAmount;
  }

  private boolean lastInStatus = false, lastOutStatus = false;
  @Override
  public void periodic() {
    if (!lastInStatus && isBallInCollector()){ // check if status has changed and there is a ball in the collector
        ballAmount++;
    } 
    if (!lastOutStatus && isBallInShooter()){ // check if status has changed and there is a ball in the shooter
      ballAmount--;
    }

    lastInStatus = isBallInCollector();
    lastOutStatus = isBallInShooter();

    if(getBallAmount() < 0 || getBallAmount() > 5){// Something is terribly wrong...
      System.err.println("Something is terribly wrong here, there are " + ballAmount + " balls in the system.");
    }
  }
}