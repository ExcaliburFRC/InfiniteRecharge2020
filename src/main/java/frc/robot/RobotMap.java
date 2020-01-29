package frc.robot;

public class RobotMap{
    //Chassi
    public static int LEFT_BACK_MOTOR_PORT = 0;
    public static int LEFT_FRONT_MOTOR_PORT = 1;
    public static int RIGHT_BACK_MOTOR_PORT = 2;
    public static int RIGHT_FRONT_MOTOR_PORT = 3;
    public static int[] RIGHT_ENCODER_P = {0,1};
    public static int[] LEFT_ENCODER_P = {2,3};

    //LED
    public static int LED_PWM_PORT = 4;

    //Collector
    public static int ROLLER_MOTOR_PORT = 5;
    public static int[] LIFTER_PORTS = {0, 1};

    //Shooter
    public static final int SHOOTER_MOTOR_1 = 6;
    public static final int SHOOTER_MOTOR_2 = 7;
    public static final int SHOOTER_ANGLER_MOTOR = 8;
    public static final int[] ANGLE_ENCODER_PORTS = {4,5};
    public static final int ZERO_ANGLE_LIMIT_SWITCH = 6;
    //Transporter
    public static final int TOWER_MOTOR_PORT = 0;
    public static final int LOADING_MOTOR_PORT = 0;
    public static final int IN_PING_PORT = 1;
    public static final int IN_ECHO_PORT = 2;
    public static final int OUT_PING_PORT = 3;
    public static final int OUT_ECHO_PORT = 4;
    public static final int TIMING_ENCODER_PORT1 = 5;
    public static final int TIMING_ENCODER_PORT2 = 6;
    public static final int ENTRANCE_SENSOR_PORT = 7; 
    
    // Climber ports
    public static int CLIMBER_LIFTER_MOTOR_PORT = -1;
    public static int ROBOT_LIFTER_MOTOR_PORT1 = -1;
    public static int ROBOT_LIFTER_MOTOR_PORT2 = -1;
    public static int HEIGHT_ENCODER_PORT1 = -1;
    public static int HEIGHT_ENCODER_PORT2 = -1;   
}