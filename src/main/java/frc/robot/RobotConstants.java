package frc.robot;

public class RobotConstants{
    public static class DriveConstants{
        public static final double MANUAL_TURN_MAX = 0.85; //TODO : tune
        public static final double WHEELDIAMETER = 0.1524;
        public static final double ENCODER_TICKS_PER_REVOLUTION = 1024; //TODO : find
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEELDIAMETER * Math.PI) / ENCODER_TICKS_PER_REVOLUTION;
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
}