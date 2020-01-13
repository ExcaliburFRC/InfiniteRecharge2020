package frc.robot.Pathfinding;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.RobotConstants.MotionProfilingConstants;

import java.util.Arrays;


public class Path{
    Pose2d[] poses;
    TrajectoryConfig config;

    public Path(boolean isReversed, Pose2d... poses){
        this.poses = poses;
        config = MotionProfilingConstants.getTrajectoryConfig().setReversed(isReversed);                         
    }

    public Path(Pose2d... poses){
        this(false, poses);
    }

    public Trajectory getTrajectory(){
        return TrajectoryGenerator.generateTrajectory(Arrays.asList(poses), config);
    }


}