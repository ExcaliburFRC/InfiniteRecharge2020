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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.OI;
import frc.robot.RobotConstants;

public class Chassi extends SubsystemBase {
  /**
   * Creates a new Chassi.
   */
  //4 spark max + neo
  CANSparkMax LBM,LFM,RBM,RFM;
  SpeedControllerGroup leftMotor, rightMotor;
  DifferentialDrive differentialDrive;
  //2 encoders (PWM)
  //AHRS gyro
  //Compressor

  public Chassi() {
    LBM = new CANSparkMax(RobotMap.LBMP, CANSparkMaxLowLevel.MotorType.kBrushless);
    LFM = new CANSparkMax(RobotMap.LFMP, CANSparkMaxLowLevel.MotorType.kBrushless);
    RBM = new CANSparkMax(RobotMap.RBMP, CANSparkMaxLowLevel.MotorType.kBrushless);
    RFM = new CANSparkMax(RobotMap.RFMP, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftMotor = new SpeedControllerGroup(LBM, LFM);
    rightMotor = new SpeedControllerGroup(RBM, RFM);

    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

    setDefaultCommand(new RunCommand(()->{
      curvatureDrive(OI.mainDrive.getRawAxis(OI.xSpeedAxis),
                    OI.mainDrive.getRawAxis(OI.zRotationAxis),
                    OI.mainDrive.getRawButton(OI.quickTurnButton));
    }, this));
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean quickTurn){
    this.differentialDrive.curvatureDrive(xSpeed, zRotation * RobotConstants.Drive.ManualTurnMax, quickTurn);
  }

  public void arcadeDrive(double xSpeed, double zRotation){
    this.differentialDrive.arcadeDrive(xSpeed, zRotation * RobotConstants.Drive.ManualTurnMax);
  }

  public void tankDrive(double lSpeed, double rSpeed){
    this.differentialDrive.tankDrive(lSpeed, rSpeed);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
