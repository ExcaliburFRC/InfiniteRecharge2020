package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.GeneralCommands.ManualShootProccess;

public class OI{
    //Joysticks
    public static Joystick driverJoystick;
    public static Joystick armJoystick;

    public static JoystickButton shootSetupButton;
    
    //Button Constants
    public static final int xSpeedAxis = 1;
    public static final int zRotationAxis = 3;
    public static final int quickTurnButton = 7;
    public static final int shootSetupButtonPort = 1;
    public static final int shootButtonPort = 1;

    
    public static void init(){
        armJoystick = new Joystick(0);
        driverJoystick = new Joystick(1);

        shootSetupButton = new JoystickButton(armJoystick, 11);
        shootSetupButton.toggleWhenPressed(new ManualShootProccess());

        initSmartDashboard();
    }

    public static void initSmartDashboard(){

    } 

    public static void updateSmartDashBoard(){

    }
}