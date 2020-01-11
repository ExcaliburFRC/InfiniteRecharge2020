package frc.robot;

public class RobotMap{
    public static int LEFT_BACK_MOTOR_PORT = 0;
    public static int LEFT_FRONT_MOTOR_PORT = 1;
    public static int RIGHT_BACK_MOTOR_PORT = 2;
    public static int RIGHT_FRONT_MOTOR_PORT = 3;

    public static int LED_PWM_PORT = 4;

    public static int[] RIGHT_ENCODER_P = {0,1};
    public static int[] LEFT_ENCODER_P = {2,3};

    public static int ROLLER_MOTOR_PORT = 5;
    public static int[] LIFTER_PORTS = {0, 1};

    public static final int SHOOTER_MOTOR_1 = 6;
    public static final int SHOOTER_MOTOR_2 = 7;
    public static final int SHOOTER_ANGLER_MOTOR = 8;
    public static final int ANGLE_POTENTIOMETER = 4;
    
    //Transporter
    public static final int MOTOR_PORT = 0;
    public static final int IN_PING_PORT = 1;
    public static final int IN_ECHO_PORT = 2;
    public static final int OUT_PING_PORT = 3;
    public static final int OUT_ECHO_PORT = 4;

    // Climber ports
    public static int CLIMBER_LIFTER_MOTOR_PORT;
    public static int ROBOT_LIFTER_MOTOR_PORT1;
    public static int ROBOT_LIFTER_MOTOR_PORT2;   
}