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
    }

    public static class ImageProccessingConstants{
        public static final double TURN_KP = 0.0375; //TODO : tune
        public static final double TURN_AFF = 0.2825; //TODO : tune
        public static final double VISION_TURN_MAX = 0.85;// TODO : tune
        public static final double TX_TOLERANCE = 0.5; // TODO : tune
    }
    
    public static class ShooterConstants{
        public static final double POTENTIOMETER_FULL_RANGE = 270.0; //TODO: tune
        public static final double ZERO_ANGLE = 20.0;//TODO : tune
        public static final double SPEED_TOLERANCE = 500.0;//TODO : tune
        public static final double ANGLE_TOLERANCE = 1.5;//TODO : tune
		public static final double CONSTANT_SHOOT_SPEED = 10000;//TODO : decide
        public static final double ABSOLUTE_FEEDFORWARD = 0.1;//TODO : tune
    }

    public static class TransporterConstants{
        public static final double BALL_DETECTION_TOLERANCE = 5.0; //TODO: tune
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