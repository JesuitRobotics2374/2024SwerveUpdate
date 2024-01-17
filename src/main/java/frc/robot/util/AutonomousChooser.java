package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.HolonomicControl.FollowCommand;
import frc.robot.HolonomicControl.HolonomicPathBuilder;
import frc.robot.HolonomicControl.Splines.Line;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private final Constraints XYconstraints = new Constraints(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 4,
            1);
    private final Constraints Rconstraints = new Constraints(
            DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 4, 4);

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser() {
        this.trajectories = new AutonomousTrajectories(XYconstraints, Rconstraints);

        autonomousModeChooser.setDefaultOption("Test thing", AutonomousMode.ONE_METER_F);
    }

    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getOneMeterFAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(resetRobotPose(container), new FollowCommand(container.getDrivetrain(),
                new HolonomicPathBuilder()
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(1, 0, new Rotation2d(0)), false))
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(0, 1, new Rotation2d(0)), false))
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(-1, 0, new Rotation2d(0)), false))
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(0, -1, new Rotation2d(0)), false))));

        return command;
    }

    public Command resetRobotPose(RobotContainer container, Pose2d pose2d) {
        return new InstantCommand(() -> container.getDrivetrain().setPose(pose2d));
    }

    public Command resetRobotPose(RobotContainer container) {
        return new InstantCommand(
                () -> container.getDrivetrain().setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
    }

    // Handler to determine what command was requested for the autonmous routine to
    // execute
    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case ONE_METER_F:
                return getOneMeterFAuto(container);
            default:
                break;
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        ONE_METER_F
    }
}