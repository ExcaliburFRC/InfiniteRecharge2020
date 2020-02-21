/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private Encoder heightEncoder;
  private Spark climberLifterMotor;
  private Spark robotLifterMotor1, robotLifterMotor2;
  // private SpeedControllerGroup robotLifterGroup;

  private boolean hasClimbed = false;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    climberLifterMotor = new Spark(RobotMap.CLIMBER_LIFTER_MOTOR_PORT);
    robotLifterMotor1= new Spark(RobotMap.ROBOT_LIFTER_MOTOR_PORT1);
    robotLifterMotor2 = new Spark(RobotMap.ROBOT_LIFTER_MOTOR_PORT2);
    // robotLifterGroup = new SpeedControllerGroup(robotLifterMotor1, robotLifterMotor2);

    heightEncoder = new Encoder(RobotMap.HEIGHT_ENCODER_PORT1,RobotMap.HEIGHT_ENCODER_PORT2);
  }

  public void setAbsHeightMotorSpeed(double speed){
    climberLifterMotor.set(speed);
  }

  public int getHeightEncoderValue(){
    return heightEncoder.get();
  }
  public void resetHeightEncoder(){
    heightEncoder.reset();
  }
  public void setRobotClimbersPower(double power){
    robotLifterMotor1.set(power);
    robotLifterMotor2.set(power);
  }

  public boolean getHasClimbed(){
    return hasClimbed;
  }
}