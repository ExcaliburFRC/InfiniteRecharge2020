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

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotMap;
import frc.robot.Utils.RobotUtils;

public class Shooter extends SubsystemBase {
  TalonSRX leftShooterMotor, rightShooterMotor;
  VictorSPX angleMotor;
  AnalogPotentiometer anglePotentiometer;
  PIDController angleController;

  boolean isSpeedPersuit, isAnglePersuit;
  double speedSetpoint, angleSetpoint;


  public Shooter() {
    leftShooterMotor = new TalonSRX(RobotMap.SHOOTER_MOTOR_1);
    rightShooterMotor = new TalonSRX(RobotMap.SHOOTER_MOTOR_2);

    angleMotor = new VictorSPX(RobotMap.SHOOTER_ANGLER_MOTOR);
    anglePotentiometer = new AnalogPotentiometer(RobotMap.ANGLE_POTENTIOMETER, ShooterConstants.POTENTIOMETER_FULL_RANGE, -ShooterConstants.ZERO_ANGLE);

    isSpeedPersuit = false;
    isAnglePersuit = false;
    
    angleController = new PIDController(ShooterConstants.ANGLE_KP, ShooterConstants.ANGLE_KI, ShooterConstants.ANGLE_KD);
    angleController.setTolerance(ShooterConstants.ANGLE_TOLERANCE);

    speedSetpoint = 0;
    angleSetpoint = 0;
  }

  public void setAngleSetpoint(double setpoint){
    this.angleSetpoint = setpoint;
    angleController.setSetpoint(setpoint);
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
    leftShooterMotor.set(ControlMode.PercentOutput, RobotUtils.clip(p,1));
    rightShooterMotor.set(ControlMode.PercentOutput, RobotUtils.clip(-p,1));
  }

  public double getAngle(){
    return anglePotentiometer.get();
  }

  public double getLeftMotorSpeed(){
    return leftShooterMotor.getSelectedSensorVelocity();
  }

  public double getRightMotorSpeed(){
    return rightShooterMotor.getSelectedSensorVelocity();
  }

  public boolean isOnSpeed(){
    boolean motor1 = Math.abs(speedSetpoint - leftShooterMotor.getSelectedSensorVelocity()) < ShooterConstants.SPEED_TOLERANCE;
    boolean motor2 = Math.abs(speedSetpoint - rightShooterMotor.getSelectedSensorVelocity()) < ShooterConstants.SPEED_TOLERANCE;
    return motor1 && motor2;
  }

  public boolean isOnAngle(){
    boolean angleError = Math.abs(angleSetpoint - angleMotor.getSelectedSensorPosition()) < ShooterConstants.ANGLE_TOLERANCE;
    return angleError;
  }

  @Override
  public void periodic() {
    if (isSpeedPersuit){
      leftShooterMotor.set(ControlMode.Velocity, speedSetpoint);
      rightShooterMotor.set(ControlMode.Velocity, speedSetpoint);
    } else {
      leftShooterMotor.set(ControlMode.PercentOutput, 0);
      rightShooterMotor.set(ControlMode.PercentOutput, 0);
    }
    
    if (isAnglePersuit){
      angleMotor.set(ControlMode.PercentOutput, angleController.calculate(getAngle(), angleSetpoint) + getFeedForward());
    } else {
      angleMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  private double getFeedForward(){
    return Math.cos(Math.toRadians(anglePotentiometer.get())) * ShooterConstants.ABSOLUTE_FEEDFORWARD;
  }

  public void setLeftMotorSpeed(double s){
    leftShooterMotor.set(ControlMode.PercentOutput, s);
  }

  public void setRightMotorSpeed(double s){
    rightShooterMotor.set(ControlMode.PercentOutput, s);
  }
}
