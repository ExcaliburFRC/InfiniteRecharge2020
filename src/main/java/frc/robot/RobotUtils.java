package frc.robot;

public class RobotUtils{
    public static double clip(double val, double max){
        if (val > max) return max;
        else if (val < -max) return -max;
        return val;
    }
}