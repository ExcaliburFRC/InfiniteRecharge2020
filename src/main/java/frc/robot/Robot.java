/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.Pathfinding.FollowerCommandGenorator;
import frc.robot.Pathfinding.TestingPaths;
import frc.robot.subsystems.Chassi;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.Collector;

public class Robot extends TimedRobot {
  public static Chassi m_chassi;
  public static Limelight m_limelight;
  public static LEDs m_leds;
  public static Shooter m_shooter;
  public static Climber m_climber;
  public static Transporter m_transporter;
  public static Collector m_collector;
  
  @Override
  public void robotInit() {
    //Init subsystems
    m_chassi = new Chassi();
    m_limelight = new Limelight();
    m_leds = new LEDs();
    m_shooter = new Shooter();
    m_climber = new Climber();
    m_transporter = new Transporter();
    m_collector = new Collector();
    OI.init();
  }

  @Override
  public void autonomousInit() {
    m_leds.setMode(LEDMode.GREEN);
    FollowerCommandGenorator.getRamseteCommandFromTrajectory(TestingPaths.sPatternPath).schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    m_leds.setMode(LEDMode.BLUE);

    CommandScheduler.getInstance().cancelAll();
    m_chassi.tankDrive(0, 0);
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
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
