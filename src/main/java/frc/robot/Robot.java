/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Utils.LEDMode;
import frc.robot.subsystems.Chassi;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;

public class Robot extends TimedRobot {
  public static Chassi m_chassi;
  public static Limelight m_limelight;
  public static LEDs m_leds;
  
  @Override
  public void robotInit() {
    //Init subsystems
    m_chassi = new Chassi();
    m_limelight = new Limelight();
    m_leds = new LEDs();
    OI.init();
  }

  @Override
  public void autonomousInit() {
    m_leds.setMode(LEDMode.GREEN);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_leds.setMode(LEDMode.BLUE);
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    OI.updateSmartDashBoard();
  }

}
