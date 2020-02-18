/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.Pathfinding.FollowerCommandGenerator;
// import frc.robot.Pathfinding.TestingPaths;
import frc.robot.TransporterCommands.TransporterDrive;
import frc.robot.Utils.CalculateVisionValues;
import frc.robot.subsystems.Chassi;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.Collector;
import frc.robot.ClimberCommands.ClimberDrive;
import frc.robot.CollectorCommands.CollectorDrive;
import frc.robot.DebugCommands.DebugShooter;
import frc.robot.DebugCommands.DebugTransport;
import frc.robot.LEDCommands.DefaultLED;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
    initSubsystems();
    initDefaultCommands();
    // OI.init(); //TODO: after all subsystems work, uncomment this
    SmartDashboard.putNumber("AngleTOGO", Math.random());
  }

  @Override
  public void autonomousInit() {
    // FollowerCommandGenerator.getRamseteCommandFromTrajectory(TestingPaths.sPatternPath).schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // CommandScheduler.getInstance().cancelAll();
    m_limelight.setPipeline(0);
    SmartDashboard.putNumber("AngleTOGO", Math.random());
    // m_chassi.tankDrive(0, 0);
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    if (OI.armJoystick.getRawButton(5)){
      Robot.m_transporter.setBallNumber(0);
    }
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
    SmartDashboard.putNumber("limelightDist", CalculateVisionValues.calculateDistanceShooter(m_limelight.getVar("ty")));
  }

  private void initSubsystems(){
    // m_chassi = new Chassi();
    m_limelight = new Limelight();
    // m_leds = new LEDs();
    // m_shooter = new Shooter();
    // m_climber = new Climber();
    m_transporter = new Transporter();
    m_collector = new Collector();
  }

  private void initDefaultCommands(){
    // m_chassi.setDefaultCommand(new RunCommand(()->{
    //   m_chassi.curvatureDrive(-OI.driverJoystick.getRawAxis(OI.xSpeedAxis),
    //                 OI.driverJoystick.getRawAxis(OI.zRotationAxis),
    //                 OI.driverJoystick.getRawButton(OI.quickTurnButton));
    // }, m_chassi));   

    m_collector.setDefaultCommand(new CollectorDrive());

    m_transporter.setDefaultCommand(new TransporterDrive());

    // m_climber.setDefaultCommand(new ClimberDrive());

    // m_leds.setDefaultCommand(new DefaultLED());

    // m_shooter.setDefaultCommand(new DebugShooter());
  }

  @Override
  public void disabledInit() {
  }
}
