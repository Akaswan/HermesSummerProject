package frc.robot.utilities;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

import static frc.robot.utilities.Constants.*;

public class CreateTrajectory {
    private Pose2d start;
    private Pose2d end;

    private ArrayList<Translation2d> interiorWaypoints;

    private Trajectory trajectory;

    public Trajectory generateTrajectory(double startVelocity) {

        start = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0),
            Rotation2d.fromDegrees(0));
        end = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(6.0),
            Rotation2d.fromDegrees(90));
    
        interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(-2.0), Units.feetToMeters(2.0)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(2.0), Units.feetToMeters(4.0)));
    
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(8));
        config.setKinematics(SWERVE_KINEMATICS);
        config.setStartVelocity(startVelocity);
    
        trajectory = TrajectoryGenerator.generateTrajectory(
            start,
            interiorWaypoints,
            end,
            config);

        return trajectory;
      }
}
