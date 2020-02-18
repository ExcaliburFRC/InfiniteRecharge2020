package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ChassiCommands.RotateAngle;
import frc.robot.GeneralCommands.ShootProccess;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.ShooterCommands.ShooterDown;

public class OI{
    //Joysticks
    public static Joystick driverJoystick = new Joystick(1);;
    public static Joystick armJoystick = new Joystick(0);;

    public static JoystickButton shootSetupButton;
    
    //Driver Joystick Constants
    public static final int xSpeedAxis = 1;
    public static final int zRotationAxis = 2;
    public static final int quickTurnButton = 5;

    //Arm Joystick Constants
    public static final int shootSetupButtonPort = 1;
    public static final int shootButtonPort = 1;
    public static final int collectorUpButton = 3;
    public static final int collectorDownButton = 4;
    public static final int collectorTakeInBallButton = 2;

    
    public static void init(){
        shootSetupButton = new JoystickButton(armJoystick, 11);
        // shootSetupButton.toggleWhenPressed(new ShootProccess(false));

        var upBtn = new JoystickButton(armJoystick, 7).whenPressed(new RunCommand(()->{
            Robot.m_limelight.setLifterState(true);
        }, Robot.m_limelight));

        var downBtn = new JoystickButton(armJoystick, 8).whenPressed(new RunCommand(()->{
            Robot.m_limelight.setLifterState(false);
        }, Robot.m_limelight));

        var ShooterDownButton = new JoystickButton(armJoystick, 6).whenPressed(new ShooterDown());

        var rotateAngle = new JoystickButton(armJoystick, 10).whenPressed(new RotateAngle(90));

        initSmartDashboard();
    }

    public static void initSmartDashboard(){

    } 

    public static void updateSmartDashBoard(){

    }
}