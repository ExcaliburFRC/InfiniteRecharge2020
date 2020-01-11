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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.RunCommand;

public class Climber extends SubsystemBase {

  private TalonSRX climberLifterMotor;
  private VictorSPX robotLifterMotor1, robotLifterMotor2;

  private boolean hasClimbed = false;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    climberLifterMotor = new TalonSRX(RobotMap.CLIMBER_LIFTER_MOTOR_PORT);
    robotLifterMotor1 = new VictorSPX(RobotMap.ROBOT_LIFTER_MOTOR_PORT1);
    robotLifterMotor2 = new VictorSPX(RobotMap.ROBOT_LIFTER_MOTOR_PORT2);
  }

  public void setClimberHeight(double height){
    climberLifterMotor.set(ControlMode.Position, height);
  }

  public void setRobotClimbersPower(double power){
    hasClimbed = true;
    robotLifterMotor1.set(ControlMode.PercentOutput, power);
    robotLifterMotor2.set(ControlMode.PercentOutput, -power);
  }

  public boolean getHasClimed(){
    return hasClimbed;
  }

}
