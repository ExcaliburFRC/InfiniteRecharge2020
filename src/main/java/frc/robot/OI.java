package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.GeneralCommands.ShootProccess;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.ShooterCommands.ShooterDown;
// import frc.robot.Utils.CalculateVisionValues;
import frc.robot.ClimberCommands.ClimberDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI{
    //Joysticks
    public static Joystick driverJoystick = new Joystick(1);;
    public static Joystick armJoystick = new Joystick(0);;

    public static JoystickButton shootSetupButton;
    
    //Driver Joystick Constants
    public static final int xSpeedAxis = 1;
    public static final int zRotationAxis = 4;
    public static final int quickTurnButton = 5;

    //Arm Joystick Constants
    public static final int shootSetupButtonPort = 1;
    public static final int shootButtonPort = 1;
    public static final int collectorUpButton = 3;
    public static final int collectorDownButton = 4;
    public static final int collectorTakeInBallButton = 2;

    private static double timeSinceRumble, timeToRumble;
    
    public static void init(){
        //Limelight up
        new JoystickButton(armJoystick, 7).whenPressed(new RunCommand(()->{
            Robot.m_limelight.setLifterState(true);
        }, Robot.m_limelight));

        //Limelight Down
        new JoystickButton(armJoystick, 8).whenPressed(new RunCommand(()->{
            Robot.m_limelight.setLifterState(false);
        }, Robot.m_limelight));

        //Reset Shooter
        new JoystickButton(armJoystick, 6).whenPressed(new ShooterDown());

        //High Shoot Proccess (Non-Auto)
        new JoystickButton(armJoystick, 11).toggleWhenPressed(new ShootProccess(false));

        //Climber
        new JoystickButton(armJoystick, 9).toggleWhenPressed(new ClimberDrive());

        //Compressor
        new JoystickButton(driverJoystick, 7).whenPressed(()->{
            Robot.m_chassi.setCompressorMode(true);
        });
        new JoystickButton(driverJoystick, 8).whenPressed(()->{
            Robot.m_chassi.setCompressorMode(false);
        });

        //Rumble
        new JoystickButton(driverJoystick, 1).whenPressed(()->{
            OI.rumbleFor(3000);
        });

        initSmartDashboard();
    }

    public static void initSmartDashboard(){
        // SmartDashboard.putNumber("ReqSpeed", 0.1);
        // SmartDashboard.putNumber("ReqAngle", 0.1);
    } 

    public static void updateSmartDashBoard(){
        if (System.currentTimeMillis() - timeSinceRumble < timeToRumble){
            OI.driverJoystick.setRumble(RumbleType.kRightRumble, 1);
            OI.driverJoystick.setRumble(RumbleType.kLeftRumble, 1);
          } else {
            OI.driverJoystick.setRumble(RumbleType.kRightRumble, 0);
            OI.driverJoystick.setRumble(RumbleType.kLeftRumble, 0);
          }
        // SmartDashboard.putNumber("limelightDist", CalculateVisionValues.calculateDistanceShooter(Robot.m_limelight.getVar("ty")));
    }

    public static void rumbleFor(double time){
        timeToRumble = time;
        timeSinceRumble = System.currentTimeMillis();
    }
}