/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotConstants;
import frc.robot.RobotMap;
import frc.robot.Utils.RobotUtils;

public class Shooter extends SubsystemBase {
  VictorSPX shooterMotor1, shooterMotor2;
  VictorSPX angleMotor;
  AnalogPotentiometer anglePotentiometer;

  boolean isSpeedPersuit, isAnglePersuit;
  double speedSetpoint, angleSetpoint;


  public Shooter() {
    shooterMotor1 = new VictorSPX(RobotMap.SHOOTER_MOTOR_1);
    shooterMotor2 = new VictorSPX(RobotMap.SHOOTER_MOTOR_2);

    angleMotor = new VictorSPX(RobotMap.SHOOTER_ANGLER_MOTOR);
    anglePotentiometer = new AnalogPotentiometer(RobotMap.ANGLE_POTENTIOMETER, ShooterConstants.POTENTIOMETER_FULL_RANGE, -ShooterConstants.ZERO_ANGLE);

    isSpeedPersuit = false;
    isAnglePersuit = false;

    speedSetpoint = 0;
    angleSetpoint = 0;
  }

  public void setAngleSetpoint(double setpoint){
    this.angleSetpoint = setpoint;
  }

  public void setIsAnglePersuit(boolean isAnglePersuit){
    this.isAnglePersuit = isAnglePersuit;
  }

  public void setAngleMotorPower(double p){
    angleMotor.set(ControlMode.PercentOutput, p);
  }

  public void setSpeedSetpoint(double setpoint){
    this.speedSetpoint = setpoint;
  }

  public void setIsSpeedPersuit(boolean isSpeedPersuit){
    this.isSpeedPersuit = isSpeedPersuit;
  }

  public void setShooterMotorPower(double p){
    shooterMotor1.set(ControlMode.PercentOutput, RobotUtils.clip(p,1));
    shooterMotor2.set(ControlMode.PercentOutput, RobotUtils.clip(-p,1));
  }

  public double getAngle(){
    return anglePotentiometer.get();
  }

  public double getShooterMotor1Speed(){
    return shooterMotor1.getSelectedSensorVelocity();
  }

  public double getShooterMotor2Speed(){
    return shooterMotor2.getSelectedSensorVelocity();
  }

  public boolean isOnSpeed(){
    boolean motor1 = Math.abs(speedSetpoint - shooterMotor1.getSelectedSensorVelocity()) < ShooterConstants.SPEED_TOLERANCE;
    boolean motor2 = Math.abs(speedSetpoint - shooterMotor2.getSelectedSensorVelocity()) < ShooterConstants.SPEED_TOLERANCE;
    return motor1 && motor2;
  }

  public boolean isOnAngle(){
    boolean angleError = Math.abs(angleSetpoint - angleMotor.getSelectedSensorPosition()) < ShooterConstants.ANGLE_TOLERANCE;
    return angleError;
  }

  @Override
  public void periodic() {
    if (isSpeedPersuit){
      shooterMotor1.set(ControlMode.Velocity, speedSetpoint);
      shooterMotor2.set(ControlMode.Velocity, speedSetpoint);
    } else {
      shooterMotor1.set(ControlMode.PercentOutput, 0);
      shooterMotor2.set(ControlMode.PercentOutput, 0);
    }
    
    if (isAnglePersuit){
      angleMotor.setSelectedSensorPosition((int) getAngle());
      angleMotor.set(ControlMode.Position, angleSetpoint, DemandType.ArbitraryFeedForward, getFeedForward());
    } else {
      angleMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  private double getFeedForward(){
    return Math.cos(Math.toRadians(anglePotentiometer.get())) * RobotConstants.ShooterConstants.ABSOLUTE_FEEDFORWARD;
  }
}
