package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI{
    //Joysticks
    public static Joystick driverJoystick = new Joystick(1);
    public static Joystick armJoystick = new Joystick(0);


    //Button Constants
    public static int xSpeedAxis = 1;
    public static int zRotationAxis = 3;
    public static int quickTurnButton = 7;
    
    public static void init(){
        initSmartDashboard();
    }

    public static void initSmartDashboard(){

    } 

    public static void updateSmartDashBoard(){

    }
}