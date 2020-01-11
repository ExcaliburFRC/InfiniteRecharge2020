package frc.robot;

public class RobotConstants{
    public static class Drive{
        public static final double MANUAL_TURN_MAX = 0.85;
        public static final double WHEELDIAMETER = 0.1524;
        public static final double ENCODER_TICKS_PER_REVOLUTION = 1024;
        public static final double ENCODER_DISTANCE_PER_PULSE = (WHEELDIAMETER * Math.PI) / ENCODER_TICKS_PER_REVOLUTION;
    }
    public static class ImageProccessing{
        public static double TURN_KP = 0.0375;
        public static double TURN_AFF = 0.2825;
        public static double VISION_TURN_MAX = 0.85;
    }
    
    public static class Shooter{
        public static final double POTENTIOMETER_FULL_RANGE = 270;
        public static final double ZERO_ANGLE = 20;
    }

    public static class TransporterConstants{
        public static final double TOLERANCE = 10.0;
    }
}