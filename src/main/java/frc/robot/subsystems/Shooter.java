/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotMap;
import frc.robot.Utils.RobotUtils;

public class Shooter extends SubsystemBase {
  TalonSRX leftShooterMotor, rightShooterMotor;
  CANSparkMax angleMotor;
  PIDController angleController;
  Encoder angleEncoder;
  DigitalInput zeroAngle;

  boolean isSpeedPursuit, isAnglePursuit;
  double speedSetpoint, angleSetpoint;


  public Shooter() {
    leftShooterMotor = new TalonSRX(RobotMap.LEFT_SHOOTER_MOTOR_PORT);
    rightShooterMotor = new TalonSRX(RobotMap.RIGHT_SHOOTER_MOTOR_PORT);
    rightShooterMotor.setSensorPhase(true);

    angleMotor = new CANSparkMax(RobotMap.SHOOTER_ANGLER_MOTOR, MotorType.kBrushless);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(true);

    angleEncoder = new Encoder(RobotMap.angleEncoder[0], RobotMap.angleEncoder[1]);
    angleEncoder.setDistancePerPulse(ShooterConstants.TICKS_TO_ANGLES);
    angleEncoder.setReverseDirection(true);

    isSpeedPursuit = false;
    isAnglePursuit = false;
    
    angleController = new PIDController(ShooterConstants.ANGLE_KP, ShooterConstants.ANGLE_KI, ShooterConstants.ANGLE_KD);
    angleController.setTolerance(ShooterConstants.ANGLE_TOLERANCE);

    zeroAngle = new DigitalInput(0);

    speedSetpoint = 0;
    angleSetpoint = 0;
  }

  public void setAngleSetpoint(double setpoint){
    this.angleSetpoint = setpoint;
    angleController.setSetpoint(setpoint);
  }

  public void setIsAnglePursuit(boolean isAnglePersuit){
    this.isAnglePursuit = isAnglePersuit;
  }

  public void setAngleMotorPower(double p){
    angleMotor.set(p);
  }

  public void setSpeedSetpoint(double setpoint){
    this.speedSetpoint = setpoint;
  }

  public void setIsSpeedPursuit(boolean isSpeedPersuit){
    this.isSpeedPursuit = isSpeedPersuit;
  }

  public void setShooterMotorPower(double p){
    leftShooterMotor.set(ControlMode.PercentOutput, RobotUtils.clip(p,1));
    rightShooterMotor.set(ControlMode.PercentOutput, RobotUtils.clip(-p,1));
  }

  public double getAngle(){
    return angleEncoder.getDistance(); 
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
    boolean angleError = Math.abs(angleSetpoint - angleEncoder.getDistance()) < ShooterConstants.ANGLE_TOLERANCE;
    return angleError;
  }

  @Override
  public void periodic() {
    if (isSpeedPursuit){
      var leftSetpoint = speedSetpoint;
      var rightSetpoint = speedSetpoint * 0.92;
      
      var leftAFF = compensateVoltage(ShooterConstants.LEFT_KV * leftSetpoint)/12.0;
      var rightAFF = compensateVoltage(ShooterConstants.RIGHT_KV * rightSetpoint)/12.0;

      var leftError = leftSetpoint - leftShooterMotor.getSelectedSensorVelocity();
      var rightError = rightSetpoint - rightShooterMotor.getSelectedSensorVelocity();

      var leftP = RobotUtils.clip(ShooterConstants.SPEED_KP * leftError, 0.125);
      var rightP = RobotUtils.clip(ShooterConstants.SPEED_KP * rightError, 0.125);

      leftShooterMotor.set(ControlMode.PercentOutput, leftAFF + leftP);
      rightShooterMotor.set(ControlMode.PercentOutput, rightAFF + rightP);
    } else {
      leftShooterMotor.set(ControlMode.PercentOutput, 0);
      rightShooterMotor.set(ControlMode.PercentOutput, 0);
    }
    
    if (isAnglePursuit){
      if (getAngle() >= ShooterConstants.MAX_ANGLE || isOnAngle()){
        angleMotor.set(0);
      } else {
        var anglePower = -1 * RobotUtils.clip(angleController.calculate(getAngle(), angleSetpoint), 0.175 - ShooterConstants.ANGLE_MOTOR_EXTRA);
        anglePower += anglePower > 0 ? ShooterConstants.ANGLE_MOTOR_EXTRA : -ShooterConstants.ANGLE_MOTOR_EXTRA;
        angleMotor.set(anglePower);
      }
    } else {
      angleMotor.set(0);
    }

    if (!zeroAngle.get()){
      angleEncoder.reset();
    }
  }

  private double getFeedForward(){
    return Math.cos(Math.toRadians(angleEncoder.getDistance())) * ShooterConstants.ABSOLUTE_FEEDFORWARD;
  }

  public double getAngleMotorPower(){
    return angleMotor.get();
  }

  public void setLeftMotorSpeed(double s){
    leftShooterMotor.set(ControlMode.PercentOutput, s);
  }

  public void setRightMotorSpeed(double s){
    rightShooterMotor.set(ControlMode.PercentOutput, s);
  }

  private double compensateVoltage(double originalVoltage){
    return originalVoltage * (ShooterConstants.VOLTAGE_AT_TOP_SPEED/RobotController.getBatteryVoltage());
  }
}
