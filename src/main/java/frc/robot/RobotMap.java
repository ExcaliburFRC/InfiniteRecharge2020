package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RobotMap{
    public static int LBMP = 0;
    public static int LFMP = 1;
    public static int RBMP = 2;
    public static int RFMP = 3;

    public static int LEDPWMP = 4;

    public static int[] RIGHT_ENCODER_P = {0,1};
    public static int[] LEFT_ENCODER_P = {2,3};

    public static int ROLLER_MOTOR_PORT = 5;
    public static int[] LIFTER_PORTS = {0, 1};

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