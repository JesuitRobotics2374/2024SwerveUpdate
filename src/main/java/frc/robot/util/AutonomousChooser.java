package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.FollowCommand;
import frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.HolonomicPathBuilder;
import frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.Splines.Line;

public class AutonomousChooser {

    private final Constraints XYconstraints = new Constraints(1,
            0.4);
    private final Constraints Rconstraints = new Constraints(
            Math.PI * .5, .5);

    private final Constraints SlowXYconstraints = new Constraints(0.5,
            0.25);
    private final Constraints SlowRconstraints = new Constraints(
            Math.PI * .5 / 2, 5);
    TrajectoryConfig config = new TrajectoryConfig(XYconstraints.maxVelocity, XYconstraints.maxAcceleration)
            .setKinematics(CommandSwerveDrivetrain.getInstance().getKinematics());
    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser() {
        autonomousModeChooser.setDefaultOption("Test thing", AutonomousMode.ONE_METER_F);
        autonomousModeChooser.addOption("Field Test", AutonomousMode.FIELD_TEST);
        autonomousModeChooser.addOption("Shooter Travel Test", AutonomousMode.SHOOTER_TRAVEL);
        autonomousModeChooser.addOption("Amp Travel Test", AutonomousMode.AMP_TRAVEL);

    }

    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getOneMeterFAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addRequirements(container.getDrivetrain());

        command.addCommands(resetRobotPose(container), new FollowCommand(container.getDrivetrain(),
                new HolonomicPathBuilder()
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(0, 0, new Rotation2d(Math.PI / 2)),
                                false))));

        return command;
    }

    public Command getFieldTestAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        System.out.println("yes");
        command.addCommands(
                resetToVision(container),
                followTrajectory(container, TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(new Translation2d(1, 1)), new Pose2d(2, 0, new Rotation2d()), config)));

        return command;
    }

    public Command getAmpTravelAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        System.out.println("yipee");
        command.addCommands(
                resetToVision(container),
                new FollowCommand(container.getDrivetrain(), new HolonomicPathBuilder().andThen(
                        new Line(XYconstraints, Rconstraints, new Pose2d(14.78, 6.7, new Rotation2d(Math.PI / 2)), true,
                                .5,
                                .4))
                        .andThen(new Line(SlowXYconstraints, SlowRconstraints,
                                new Pose2d(14.78, 7.28, new Rotation2d(Math.PI / 2)), true, 0.03, 0.05))));

        return command;
    }

    public Command getShooterTravelAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        System.out.println("yipee");
        command.addCommands(
                resetToVision(container),
                new FollowCommand(container.getDrivetrain(), new HolonomicPathBuilder().andThen(
                        new Line(XYconstraints, Rconstraints, new Pose2d(14.5, 5.57, new Rotation2d(0)), true, .8,
                                .6))
                        .andThen(new Line(SlowXYconstraints, SlowRconstraints,
                                new Pose2d(15.15, 5.57, new Rotation2d(0)), true, 0.03, 0.05))));

        return command;
    }

    public Command resetRobotPose(RobotContainer container, Pose2d pose2d) {
        return new InstantCommand(() -> container.getDrivetrain().seedFieldRelative(pose2d));
    }

    public Command resetRobotPose(RobotContainer container) {
        return new InstantCommand(
                () -> container.getDrivetrain().runOnce(() -> container.getDrivetrain()
                        .seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
    }

    public Command resetToVision(RobotContainer container) {
        System.out.println(container.getDrivetrain().getState().Pose);
        return new InstantCommand(() -> container.getDrivetrain().alignToVision());
    }

    public Command followTrajectory(RobotContainer container, Trajectory trajectory) {
        PIDController Xcontroller = new PIDController(1.5, 0, 0);
        PIDController Ycontroller = new PIDController(1.5, 0, 0);
        ProfiledPIDController profiledPIDController = new ProfiledPIDController(3, 0, 0, Rconstraints);
        return new SwerveControllerCommand(trajectory,
                () -> CommandSwerveDrivetrain.getInstance().getState().Pose,
                CommandSwerveDrivetrain.getInstance().getKinematics(), Xcontroller, Ycontroller, profiledPIDController,
                (x) -> CommandSwerveDrivetrain.getInstance().setStates(x), CommandSwerveDrivetrain.getInstance());

    }

    // Handler to determine what command was requested for the autonmous routine to
    // execute
    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case ONE_METER_F:
                return getOneMeterFAuto(container);
            case FIELD_TEST:
                return getFieldTestAuto(container);
            case SHOOTER_TRAVEL:
                return getShooterTravelAuto(container);
            case AMP_TRAVEL:
                return getAmpTravelAuto(container);
            default:
                break;
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        ONE_METER_F, FIELD_TEST, SHOOTER_TRAVEL, AMP_TRAVEL
    }
}