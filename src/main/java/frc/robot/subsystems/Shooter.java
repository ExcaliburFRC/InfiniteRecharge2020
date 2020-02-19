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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  double rightSpeedSetPoint, leftSpeedSetpoint;
  BooleanAverager speedReadyAverager, angleReadyAverager;

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

    speedReadyAverager = new BooleanAverager(60);
    angleReadyAverager = new BooleanAverager(45);
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

  public boolean getRawIsOnSpeed(){
    boolean motor1 = Math.abs(speedSetpoint - getLeftMotorSpeed()) < ShooterConstants.SPEED_TOLERANCE;
    boolean motor2 = Math.abs(speedSetpoint - getRightMotorSpeed()) < ShooterConstants.SPEED_TOLERANCE;
    return motor1 && motor2;
  }

  public boolean isOnSpeed(){
    return speedReadyAverager.getAverage();
  }

  public boolean getRawIsOnAngle(){
    boolean angleError = Math.abs(angleSetpoint - angleEncoder.getDistance()) < ShooterConstants.ANGLE_TOLERANCE;
    return angleError;
  }
  public boolean isOnAngle(){
    return angleReadyAverager.getAverage();
  }

  @Override
  public void periodic() {
    if (isSpeedPursuit){
      var finePower = 0.98;
      rightSpeedSetPoint = speedSetpoint * finePower;
      leftSpeedSetpoint = speedSetpoint * finePower;
      
      var leftAFF = compensateVoltage(ShooterConstants.LEFT_KV * leftSpeedSetpoint)/12.0;
      var rightAFF = compensateVoltage(ShooterConstants.RIGHT_KV * rightSpeedSetPoint)/12.0;

      var leftError = leftSpeedSetpoint - leftShooterMotor.getSelectedSensorVelocity();
      var rightError = rightSpeedSetPoint - rightShooterMotor.getSelectedSensorVelocity();

      var leftP = RobotUtils.clip(ShooterConstants.SPEED_KP * leftError, ShooterConstants.kPEffectiveness);
      var rightP = RobotUtils.clip(ShooterConstants.SPEED_KP * rightError, ShooterConstants.kPEffectiveness);

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

    angleReadyAverager.update(getRawIsOnAngle());
    speedReadyAverager.update(getRawIsOnSpeed());

    // SmartDashboard.putNumber("IsOkToShooter", isOnSpeed() ? 1 : 0);
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

  public boolean getZeroSwitch(){
    return !zeroAngle.get();
  }
}
