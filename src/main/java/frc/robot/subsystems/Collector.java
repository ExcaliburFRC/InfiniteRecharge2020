/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Collector extends SubsystemBase {
  Spark rollerMotor;
  DoubleSolenoid lifterPiston;

  public Collector() {
    rollerMotor = new Spark(RobotMap.ROLLER_MOTOR_PORT);
    lifterPiston = new DoubleSolenoid(RobotMap.LIFTER_PORTS[0], RobotMap.LIFTER_PORTS[1]);
  }

  public void setRollerMotorPower(double p){
    rollerMotor.set(p);
  }

  public void setLifterPistonPosition(boolean on){
    Value value = on ? Value.kForward : Value.kReverse;
    lifterPiston.set(value);
  }

  public boolean getLifterPistonPosition(){
    return lifterPiston.get() == Value.kForward ? true : false;
  }
}
