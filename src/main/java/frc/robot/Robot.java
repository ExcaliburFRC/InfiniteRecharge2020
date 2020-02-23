/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.Pathfinding.FollowerCommandGenerator;
// import frc.robot.Pathfinding.TestingPaths;
import frc.robot.subsystems.Chassi;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.subsystems.Collector;
import frc.robot.ChassiCommands.TimedStrightDrive;
import frc.robot.CollectorCommands.CollectorDrive;
import frc.robot.GeneralCommands.ShootProccess;
import frc.robot.TransporterCommands.FuckedNavXTransport;
import frc.robot.Utils.AutoCommandGenerator;
import frc.robot.ShooterCommands.ShooterDown;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.LEDCommands.DefaultLED;

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
  TimedStrightDrive backCommand;
  boolean hasBackMovingEnded;
  
  @Override
  public void robotInit() {
    initSubsystems();
    initDefaultCommands();
    OI.init();
    isFirstTime = true;

    // CameraServer.getInstance().startAutomaticCapture("Camera", 0);
    m_chassi.setCompressorMode(false);
    resetAuto();
  }

  @Override
  public void autonomousInit() {
    if (isFirstTime){
      isFirstTime = false;
      new ShooterDown().schedule();
    }
    // FollowerCommandGenerator.getRamseteCommandFromTrajectory(TestingPaths.sPatternPath).schedule();

    // autoCommand = AutoCommandGenerator.goBackAndShoot().alongWith(new FuckedNavXTransport());
    resetAuto();
    backCommand.schedule();
    hasBackMovingEnded = false;
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    if (backCommand.hasEnded && !hasBackMovingEnded){
      hasBackMovingEnded = true;
      autoCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    autoCommand.end(false);
    // CommandScheduler.getInstance().cancelAll();
    m_limelight.setPipeline(0);
    m_limelight.setCamMode(Limelight.CamModes.DRIVING);
    m_limelight.setLEDMode(Limelight.LedModes.OFF);
    m_limelight.setLifterState(false);
    m_shooter.setIsAnglePursuit(false);
    m_shooter.setIsSpeedPursuit(false);
    m_leds.setMode(LEDMode.BLUE);
    m_chassi.resetGyro();

    // if (isFirstTime){
    //   isFirstTime = false;
    new ShooterDown().schedule();
    // }
    // m_limelight.setCamMode(Limelight.CamModes.VISION);
    // m_limelight.setLEDMode(Limelight.LedModes.ON);
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();

    // m_climber.setRobotClimbersPower(-OI.armJoystick.getRawAxis(2));
    // m_climber.setAbsHeightMotorSpeed(OI.armJoystick.getRawAxis(1));
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

  private void initSubsystems(){
    m_chassi = new Chassi();
    m_limelight = new Limelight();
    m_leds = new LEDs();
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

    m_leds.setDefaultCommand(new DefaultLED());

    // m_shooter.setDefaultCommand(new DebugShooter());
  }

  @Override
  public void disabledInit() {
  }
  
  private void resetAuto(){
    autoCommand = new ShootProccess(true).alongWith(new FuckedNavXTransport());
    backCommand = new TimedStrightDrive(1100, -0.6);
  }
}
