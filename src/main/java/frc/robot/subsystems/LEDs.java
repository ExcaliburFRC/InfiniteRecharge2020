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

public class LEDs extends SubsystemBase {
  private Spark ledPWM;
  private double currentLedMode;

  public LEDs() {
    ledPWM = new Spark(RobotMap.LED_PWM_PORT);
    currentLedMode = LEDMode.BLUE.value;
  }

  public void setMode(LEDMode mode){
    currentLedMode = mode.value;
  }
  
  public enum LEDMode{
    BLUE(0.87), RED(0.61), GREEN(0.73), YELLOW(0.67), RAINBOW(-0.97), OFF(0.99);

    public final double value;
    private LEDMode(double value){
        this.value = value;
    }
  }

  @Override
  public void periodic() {
    ledPWM.set(currentLedMode);
  }
}
