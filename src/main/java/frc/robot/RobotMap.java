package frc.robot;

public class RobotMap{
    //Chassi
    public static final int LEFT_BACK_MOTOR_PORT = 0;
    public static final int LEFT_FRONT_MOTOR_PORT = 1;
    public static final int RIGHT_BACK_MOTOR_PORT = 2;
    public static final int RIGHT_FRONT_MOTOR_PORT = 3;
    public static final int[] RIGHT_ENCODER_P = {0,1};
    public static final int[] LEFT_ENCODER_P = {2,3};

    //LED
    public static final int LED_PWM_PORT = 4;

    //Collector
    public static final int ROLLER_MOTOR_PORT = 5;
    public static final int[] LIFTER_PORTS = {0, 1};

    //Shooter
    public static final int LEFT_SHOOTER_MOTOR_PORT = 6;
    public static final int RIGHT_SHOOTER_MOTOR_PORT = 7;
    public static final int SHOOTER_ANGLER_MOTOR = 8;
    public static final int[] ANGLE_ENCODER_PORTS = {4,5};
    public static final int ZERO_ANGLE_LIMIT_SWITCH = 6;
    //Transporter
    public static final int TOWER_MOTOR_PORT = 0;
    public static final int LOADING_MOTOR_PORT = 0;
    public static final int DIAGONAL_MOTOR_PORT = 0;
    public static final int DIAGONAL_PING_PORT = 1;
    public static final int DIAGONAL_ECHO_PORT = 2;
    public static final int OUT_PING_PORT = 3;
    public static final int OUT_ECHO_PORT = 4;
    public static final int UNDERTIMEING_SENSOR_PORT = 8;
    public static final int TIMING_ENCODER_PORT1 = 5;
    public static final int TIMING_ENCODER_PORT2 = 6;
    public static final int ENTRANCE_SENSOR_PORT = 7; 
    
    // Climber ports
    public static final int CLIMBER_LIFTER_MOTOR_PORT = -1;
    public static final int ROBOT_LIFTER_MOTOR_PORT = -1;
    public static final int HEIGHT_ENCODER_PORT1 = -1;
    public static final int HEIGHT_ENCODER_PORT2 = -1;   

    //Limelight
    public static final int[] LIMELIGHT_SOLENOID_PORTS = {2, 3};
}