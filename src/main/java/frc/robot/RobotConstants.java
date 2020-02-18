package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class RobotConstants{
    public static class DriveConstants{
        public static final double MANUAL_TURN_MAX = 0.85; //TODO : tune
        public static final double WHEELDIAMETER = 0.1524;
        public static final double ENCODER_TICKS_PER_REVOLUTION = 1024; //TODO : find
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEELDIAMETER * Math.PI) / ENCODER_TICKS_PER_REVOLUTION;

        public static final boolean isRightEncoderReversed = false;
        public static final boolean isLeftEncoderReversed = false;

        //These values are tuned for large rotations
        public static final double TURN_KP = 0.0375; //TODO : tune
        public static final double TURN_AFF = 0.2825; //TODO : tune
        public static final double ANGLE_TOLERACE = 0.75; //TODO : tune
    }

    public static class ClimbConstants{
        public static final double DISTANCE_PER_TICKS = 100;
        public static final double MAX_HEIGHT = 50000;
        public static final double KP = 0.04; //TODO : tune
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double TOLERANCE = 10;
        public static final double AFF = 0;
    }

    public static class ImageProccessingConstants{
        //These values are tuned for small, precise rotations
        public static final double TURN_KP = 0.0375; //TODO : tune
        public static final double TURN_AFF = 0.2825; //TODO : tune
        public static final double VISION_TURN_MAX = 0.85;// TODO : tune
        public static final double TX_TOLERANCE = 0.2; // TODO : tune

        public static final double CAMERA_OFFSET_FROM_SHOOTER = 0.5; //TODO: messure

        public static final double FORWARD_KP = 0.2;
        public static final double FORWARD_AFF = 0.2;
    }
    
    public static class ShooterConstants{
        public static final double SPEED_TOLERANCE = 150.0;

        public static final double RIGHT_TOP_SPEED = 24000.0; 
        public static final double LEFT_TOP_SPEED = 26000.0;
        public static final double VOLTAGE_AT_TOP_SPEED = 11; 
        public static final double RIGHT_KV = VOLTAGE_AT_TOP_SPEED/RIGHT_TOP_SPEED;
        public static final double LEFT_KV = VOLTAGE_AT_TOP_SPEED/LEFT_TOP_SPEED;
        public static final double SPEED_KP = 0.000145;
        
        public static final double ABSOLUTE_FEEDFORWARD = 0;//TODO : tune
        public static final double TICKS_TO_ANGLES = 0.021176;
        public static final double MAX_ANGLE = 45;
        public static final double ANGLE_TOLERANCE = 0.15;//TODO : tune
        public static final double ANGLE_KP = 0.0275; //TODO : tune
        public static final double ANGLE_KI = 0; //TODO : tune
        public static final double ANGLE_KD = 0; //TODO : tune
        public static final double ANGLE_MOTOR_EXTRA = 0.0175;
    }

    public static class TransporterConstants{
        public static final double BALL_DETECTION_TOLERANCE = 0; //TODO: tune
        public static final double TRANSPORT_STEP = 1550; //TODO: tune
        public static final double TRANSPORT_TOLERANCE = 150; //TODO: tune
        public static final double IN_BETWEEN_TIME = 35;
    }

    public static class MotionProfilingConstants{
        public static final double ksVolts = -1; //TODO: needs to be tuned with Robot Char Suite
        public static final double kvVoltSecondsPerMeter = -1; //TODO: needs to be tuned with Robot Char Suite
        public static final double kaVoltSecondsSquaredPerMeter = -1; //TODO: needs to be tuned with Robot Char Suite

        public static final double trackWidthMeters = 0.6; //TODO: needs to be messured
        public static final DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(trackWidthMeters);

        public static final double maxSpeed = -1; //TODO: needs to be tuned - is in meters per second
        public static final double maxAccelaration = -1; //TODO: needs to be tuned - is in meters per second ^ 2

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kPDrive = 1; //TODO: needs to be tuned

        public static TrajectoryConfig getTrajectoryConfig(){
            // Create a voltage constraint to ensure we don't accelerate too fast
            var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(MotionProfilingConstants.ksVolts,
                                        MotionProfilingConstants.kvVoltSecondsPerMeter,
                                        MotionProfilingConstants.kaVoltSecondsSquaredPerMeter),
                                        MotionProfilingConstants.DriveKinematics,
                10);


            TrajectoryConfig config = new TrajectoryConfig(MotionProfilingConstants.maxSpeed,
                        MotionProfilingConstants.maxAccelaration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(MotionProfilingConstants.DriveKinematics)
                        // Dont let the voltage go about the voltage constraint
                        .addConstraint(autoVoltageConstraint);
            
            return config;
        } 
    }
}