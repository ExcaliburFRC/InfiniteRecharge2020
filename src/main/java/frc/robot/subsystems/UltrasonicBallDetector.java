package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.RobotConstants.TransporterConstants;

public class UltrasonicBallDetector implements BallDetector{
    private Ultrasonic distanceSensor;
    private double MINIMUM_DISTANCE;

    public UltrasonicBallDetector(int ping, int echo){
        distanceSensor = new Ultrasonic(ping, echo);
        MINIMUM_DISTANCE =  TransporterConstants.BALL_DETECTION_TOLERANCE;
    }

    public UltrasonicBallDetector(int ping, int echo, double minimum){
        distanceSensor = new Ultrasonic(ping, echo);
        MINIMUM_DISTANCE =  minimum;
    }

    private double getMeasuredDistance() {
        return distanceSensor.getRangeMM() / 10;
    }

    @Override
    public boolean isBallDetected() {
        return getMeasuredDistance() < MINIMUM_DISTANCE;
    }
}