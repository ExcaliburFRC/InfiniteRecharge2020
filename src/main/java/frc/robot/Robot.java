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
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.ChassiCommands.TimedStrightDrive;
import frc.robot.ClimberCommands.ClimberDrive;
import frc.robot.CollectorCommands.CollectorDrive;
import frc.robot.DebugCommands.DebugShooter;
import frc.robot.DebugCommands.DebugTransport;
import frc.robot.GeneralCommands.ShootProccess;
import frc.robot.GeneralCommands.Wait;
import frc.robot.TransporterCommands.FuckedNavXTransport;
import frc.robot.LEDCommands.DefaultLED;
import frc.robot.ShooterCommands.ShooterDown;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Robot extends TimedRobot {
  public static Chassi m_chassi;
  public static Limelight m_limelight;
  public static LEDs m_leds;
  public static Shooter m_shooter;
  public static Climber m_climber;
  public static Transporter m_transporter;
  public static Collector m_collector;
  private boolean isFirstTime;
  Command autoCommand;
  
  @Override
  public void robotInit() {
    initSubsystems();
    initDefaultCommands();
    OI.init();
    isFirstTime = true;
    SmartDashboard.putNumber("ReqSpeed", 0.1);
    SmartDashboard.putNumber("ReqAngle", 0.1);
  }

  @Override
  public void autonomousInit() {
    if (isFirstTime){
      isFirstTime = false;
      new ShooterDown().schedule();
    }
    // FollowerCommandGenerator.getRamseteCommandFromTrajectory(TestingPaths.sPatternPath).schedule();
    var shootCommand = new ShootProccess(true);
    var shooterWaitCommand = new Wait(8);
    var moveBack = new TimedStrightDrive(1100, -0.6);

    autoCommand = moveBack.andThen(shooterWaitCommand.deadlineWith(shootCommand));
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    autoCommand.end(true);
    // CommandScheduler.getInstance().cancelAll();
    m_limelight.setPipeline(0);
    m_limelight.setCamMode(Limelight.CamModes.DRIVING);
    m_limelight.setLEDMode(Limelight.LedModes.OFF);
    m_chassi.resetGyro();

    if (isFirstTime){
      isFirstTime = false;
      new ShooterDown().schedule();
    }
    // m_limelight.setCamMode(Limelight.CamModes.VISION);
    // m_limelight.setLEDMode(Limelight.LedModes.ON);
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    if (OI.armJoystick.getRawButton(5)){
      Robot.m_transporter.setBallNumber(0);
    }

    SmartDashboard.putNumber("GYRO", m_chassi.getGyroAngle());

    m_climber.setRobotClimbersPower(-OI.armJoystick.getRawAxis(2));
    m_climber.setAbsHeightMotorSpeed(OI.armJoystick.getRawAxis(1));
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
    // SmartDashboard.putNumber("limelightAngle",m_limelight.getTx());

  }

  private void initSubsystems(){
    m_chassi = new Chassi();
    m_limelight = new Limelight();
    // m_leds = new LEDs();
    m_shooter = new Shooter(true);
    m_climber = new Climber();
    m_transporter = new Transporter();
    m_collector = new Collector();
  }

  private void initDefaultCommands(){
     m_chassi.setDefaultCommand(new RunCommand(()->{
      m_chassi.arcadeDrive(-OI.driverJoystick.getRawAxis(1), OI.driverJoystick.getRawAxis(2));
     }, m_chassi));   

    m_collector.setDefaultCommand(new CollectorDrive());

    m_transporter.setDefaultCommand(new FuckedNavXTransport());

    // m_climber.setDefaultCommand(new ClimberDrive());

    // m_leds.setDefaultCommand(new DefaultLED());

    // m_shooter.setDefaultCommand(new DebugShooter());
  }

  @Override
  public void disabledInit() {
  }
}
