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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.ClimberCommands.ClimberDrive;

public class Climber extends SubsystemBase {
  private Encoder heightEncoder;
  private TalonSRX climberLifterMotor;
  private VictorSPX robotLifterMotor;

  private boolean hasClimbed = false;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    heightEncoder = new Encoder(RobotMap.HEIGHT_ENCODER_PORT1,RobotMap.HEIGHT_ENCODER_PORT2);
    climberLifterMotor = new TalonSRX(RobotMap.CLIMBER_LIFTER_MOTOR_PORT);
    robotLifterMotor = new VictorSPX(RobotMap.ROBOT_LIFTER_MOTOR_PORT);

    setDefaultCommand(new ClimberDrive());
  }

  public void setClimberHeight(double height){
    climberLifterMotor.set(ControlMode.Position, height);
  }

  public void setAbsHeightMotorSpeed(double speed){
    climberLifterMotor.set(ControlMode.PercentOutput, speed);
  }

  public int getHeightEncoderValue(){
    return heightEncoder.get();
  }
  public void resetHeightEncoder(){
    heightEncoder.reset();
  }
  public void setRobotClimbersPower(double power){
    hasClimbed = true;
    robotLifterMotor.set(ControlMode.PercentOutput, power);
  }

  public boolean getHasClimbed(){
    return hasClimbed;
  }

}
