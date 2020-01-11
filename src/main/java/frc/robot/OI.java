package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OI{
    //Joysticks
    public static Joystick driverJoystick = new Joystick(1);
    public static Joystick armJoystick = new Joystick(0);
    
    //Button Constants
    public static final int xSpeedAxis = 1;
    public static final int zRotationAxis = 3;
    public static final int quickTurnButton = 7;
    
    public static void init(){
        initSmartDashboard();
    }

    public static void initSmartDashboard(){

    } 

    public static void updateSmartDashBoard(){

    }
}