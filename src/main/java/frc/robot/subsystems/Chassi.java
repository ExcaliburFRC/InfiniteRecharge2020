/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.OI;
import frc.robot.RobotConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import com.kauailabs.navx.frc.AHRS;

public class Chassi extends SubsystemBase {
  //4 spark max + neo
  CANSparkMax LBM,LFM,RBM,RFM;
  SpeedControllerGroup leftMotor, rightMotor;
  DifferentialDrive differentialDrive;
  //2 encoders (PWM)
  Encoder rightEncoder, leftEncoder;
  //AHRS gyro
  AHRS gyro;
  //Compressor

  DifferentialDriveOdometry driveOdometry;

  public Chassi() {
    LBM = new CANSparkMax(RobotMap.LEFT_BACK_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    LFM = new CANSparkMax(RobotMap.LEFT_FRONT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    RBM = new CANSparkMax(RobotMap.RIGHT_BACK_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    RFM = new CANSparkMax(RobotMap.RIGHT_FRONT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftMotor = new SpeedControllerGroup(LBM, LFM);
    rightMotor = new SpeedControllerGroup(RBM, RFM);

    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

    rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_P[0], RobotMap.RIGHT_ENCODER_P[1]);
    rightEncoder.setDistancePerPulse(RobotConstants.Drive.ENCODER_DISTANCE_PER_PULSE);

    leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_P[0], RobotMap.LEFT_ENCODER_P[1]);
    leftEncoder.setDistancePerPulse(RobotConstants.Drive.ENCODER_DISTANCE_PER_PULSE);

    resetEncoders();

    gyro = new AHRS(SPI.Port.kMXP);

    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroAngle()));

    setDefaultCommand(new RunCommand(()->{
      curvatureDrive(OI.driverJoystick.getRawAxis(OI.xSpeedAxis),
                    OI.driverJoystick.getRawAxis(OI.zRotationAxis),
                    OI.driverJoystick.getRawButton(OI.quickTurnButton));
    }, this));
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean quickTurn){
    this.differentialDrive.curvatureDrive(xSpeed, zRotation * RobotConstants.Drive.MANUAL_TURN_MAX, quickTurn);
  }

  public void arcadeDrive(double xSpeed, double zRotation){
    this.differentialDrive.arcadeDrive(xSpeed, zRotation * RobotConstants.Drive.MANUAL_TURN_MAX);
  }

  public void tankDrive(double lSpeed, double rSpeed){
    this.differentialDrive.tankDrive(lSpeed, rSpeed);
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  public void resetEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }
  
  public void resetGyro(){
    gyro.reset();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  @Override
  public void periodic() {
    driveOdometry.update(Rotation2d.fromDegrees(getGyroAngle()), leftEncoder.getDistance(),
                      rightEncoder.getDistance());
  }
}
