package frc.robot;

public class RobotConstants{
    public static class DriveConstants{
        public static final double MANUAL_TURN_MAX = 0.85;
        public static final double WHEELDIAMETER = 0.1524;
        public static final double ENCODER_TICKS_PER_REVOLUTION = 1024;
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEELDIAMETER * Math.PI) / ENCODER_TICKS_PER_REVOLUTION;
    }

    public static class ImageProccessingConstants{
        public static double TURN_KP = 0.0375;
        public static double TURN_AFF = 0.2825;
        public static double VISION_TURN_MAX = 0.85;
    }
    
    public static class ShooterConstants{
        public static final double POTENTIOMETER_FULL_RANGE = 270.0;
        public static final double ZERO_ANGLE = 20.0;//TODO : tune
        public static final double SPEED_TOLERANCE = 500.0;//TODO : tune
        public static final double ANGLE_TOLERANCE = 1.5;//TODO : tune
		public static final double CONSTANT_SHOOT_SPEED = 10000;//TODO : tune

    }

    public static class TransporterConstants{
        public static final double BALL_DETECTION_TOLARANCE = 5.0;
    }
}