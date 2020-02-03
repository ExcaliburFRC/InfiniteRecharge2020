package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.GeneralCommands.ShootProccess;

public class OI{
    //Joysticks
    public static Joystick driverJoystick = new Joystick(1);;
    public static Joystick armJoystick = new Joystick(0);;

    public static JoystickButton shootSetupButton;
    
    //Driver Joystick Constants
    public static final int xSpeedAxis = 1;
    public static final int zRotationAxis = 3;
    public static final int quickTurnButton = 7;

    //Arm Joystick Constants
    public static final int shootSetupButtonPort = 1;
    public static final int shootButtonPort = 1;
    public static final int collectorUpButton = 3;
    public static final int collectorDownButton = 4;
    public static final int collectorTakeInBallButton = 2;

    
    public static void init(){
        shootSetupButton = new JoystickButton(armJoystick, 11);
        shootSetupButton.toggleWhenPressed(new ShootProccess(false));

        initSmartDashboard();
    }

    public static void initSmartDashboard(){

    } 

    public static void updateSmartDashBoard(){

    }
}