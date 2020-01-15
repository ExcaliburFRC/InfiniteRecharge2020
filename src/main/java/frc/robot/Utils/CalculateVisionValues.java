package frc.robot.Utils;

import frc.robot.RobotConstants;

public class CalculateVisionValues{ //TODO : TUNE EVERYTHING
    public static double calculateDistance(double ty){
        return ty*1; //TODO needs to be tuned
    }

    public static double getOptimalShooterAngle(double dist){// gets the optimal shooter angle
        return 0;
    }

    public static double getOptimalShooterSpeed(double dist){// gets the optimal shooter speed
        return 0;
    }

    public static double getAngleWhenSetSpeed(double dist){ //gets the angle when shooter at 85% speed
        return 0;
    }

    /*
        @param tx is the tx value from the camera in degress
        @param ty is the ty value from the camera in degress
        @returns the x^2 value for offset calculations
    */
    public static double getX2Value(double tx, double ty){
        double d = calculateDistance(ty);
        double z = RobotConstants.ImageProccessingConstants.CAMERA_OFFSET_FROM_SHOOTER;
        return Math.pow(d,2) + Math.pow(z,2) - 2 * d * z * Math.sin(Math.toRadians(tx));
    }

    /*
        @param cameraTx is the angle offSet of the target from the camera in the x axis
        @param cameraTx is the angle offSet of the target from the camera in the y axis
        @returns the angle 
    */
    public static double getShooterTX(double cameraTx, double cameraTy){
        double x2 = getX2Value(cameraTx, cameraTy);
        double d = calculateDistance(cameraTy);
        double z = RobotConstants.ImageProccessingConstants.CAMERA_OFFSET_FROM_SHOOTER;

        double inCosine = (-Math.pow(d,2) + x2 + Math.pow(z,2)) / (2 * Math.sqrt(x2) * z);
        return 90 - Math.toDegrees(Math.acos(inCosine));
    }
}