package frc.robot.Pathfinding;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import java.util.List;
import frc.robot.Pathfinding.Path;
import frc.robot.RobotConstants.MotionProfilingConstants;

public class TestingPaths{
    public static Trajectory sPatternPath = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        MotionProfilingConstants.getTrajectoryConfig()
    );

    public static Trajectory strightThreeMetersPath = new Path(
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(3, 0, new Rotation2d(0))
    ).getTrajectory();
}