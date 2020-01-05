package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI{
    //Joysticks
    public static XboxController mainDrive = new XboxController(1);

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