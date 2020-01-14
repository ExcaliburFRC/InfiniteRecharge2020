package frc.robot.Pathfinding;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Robot;
import frc.robot.RobotConstants.MotionProfilingConstants;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.PIDController;

public class FollowerCommandGenerator{
    public static Command getRamseteCommandFromTrajectory(Trajectory trajectory){
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            Robot.m_chassi::getPose,
            new RamseteController(MotionProfilingConstants.kRamseteB, MotionProfilingConstants.kRamseteZeta),
            new SimpleMotorFeedforward(MotionProfilingConstants.ksVolts,
                                        MotionProfilingConstants.kvVoltSecondsPerMeter,
                                        MotionProfilingConstants.kaVoltSecondsSquaredPerMeter),
            MotionProfilingConstants.DriveKinematics,
            Robot.m_chassi::getWheelSpeeds,
            new PIDController(MotionProfilingConstants.kPDrive, 0, 0),
            new PIDController(MotionProfilingConstants.kPDrive, 0, 0),
            // RamseteCommand passes volts to the callback
            Robot.m_chassi::tankDriveVolts,
            Robot.m_chassi
        );
        return ramseteCommand.andThen(()->Robot.m_chassi.tankDrive(0, 0));
    }

    public static Command getRamseteCommandFromTrajectory(Path p){
        return getRamseteCommandFromTrajectory(p.getTrajectory());
    }

    private FollowerCommandGenerator(){}
}