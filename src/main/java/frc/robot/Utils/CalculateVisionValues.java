package frc.robot.Utils;

import frc.robot.RobotConstants;

public class CalculateVisionValues{ 
    public static double calculateDistanceFeeder(double ta){
        return ta*1; //TODO needs to be tuned
    }

    public static double calculateDistanceShooter(double ty){
        return 0.0293 * ty * ty - 1.29 * ty + 17.9;
    }

    public static double getOptimalShooterAngle(double dist){// gets the optimal shooter angle
        // return 0.154 * dist * dist - 2.53 * dist + 31.7;
        return 0.229 * dist * dist - 2.93 * dist + 44.5;
    }

    public static double getOptimalShooterSpeed(double dist){// gets the optimal shooter speed
        // return 15000;
        // return 91.3 * dist * dist - 704 * dist + 13275;
        return 91.3 * dist * dist - 704 * dist + 13100;
    }

    /*
        @param tx is the tx value from the camera in degress
        @param ty is the ty value from the camera in degress
        @returns the x^2 value for offset calculations
    */
    public static double getX2Value(double tx, double ty){
        double d = calculateDistanceShooter(ty);
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
        double d = calculateDistanceShooter(cameraTy);
        double z = RobotConstants.ImageProccessingConstants.CAMERA_OFFSET_FROM_SHOOTER;

        double inCosine = (-Math.pow(d,2) + x2 + Math.pow(z,2)) / (2 * Math.sqrt(x2) * z);
        return 90 - Math.toDegrees(Math.acos(inCosine));
    }   

    public static double getShooterTX2(double tx, double ty){
        double offset = RobotConstants.ImageProccessingConstants.CAMERA_OFFSET_FROM_SHOOTER;
        double dist = calculateDistanceShooter(ty);
        double hortx = Math.PI / 2 - Math.toRadians(tx);
        double f = Math.sqrt(dist * dist + Math.pow(offset, 2) - 2 * dist * offset * Math.cos(hortx));
        double c = Math.asin(offset * Math.sin(hortx) / f);
        double b = Math.PI - hortx - c;
        double a = (Math.PI / 2 - b) * 180.0 / Math.PI;
        return a;
    }
}