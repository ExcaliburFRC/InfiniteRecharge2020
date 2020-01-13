/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.RobotMap;
import frc.robot.Utils.LEDMode;

public class LEDs extends SubsystemBase {
  private Spark ledPWM;

  public LEDs() {
    ledPWM = new Spark(RobotMap.LED_PWM_PORT);
  }

  public void setMode(LEDMode mode){
    ledPWM.set(mode.getValue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
