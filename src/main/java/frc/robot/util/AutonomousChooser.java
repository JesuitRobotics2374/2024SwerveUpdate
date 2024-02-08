package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.FollowCommand;
import frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.HolonomicPathBuilder;
import frc.robot.subsystems.DrivetrainSubsystem.HolonomicControl.Splines.Line;

public class AutonomousChooser {

    private final Constraints XYconstraints = new Constraints(1,
            0.5);
    private final Constraints Rconstraints = new Constraints(
            Math.PI * .5 / 4, 1);

    private final Constraints SlowXYconstraints = new Constraints(0.5,
            0.25);
    private final Constraints SlowRconstraints = new Constraints(
            Math.PI * .5 / 8, 5);

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
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(1, 0, new Rotation2d(0)), false))
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(0, 1, new Rotation2d(0)), false))
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(-1, 0, new Rotation2d(0)), false))
                        .andThen(new Line(XYconstraints, Rconstraints, new Pose2d(0, -1, new Rotation2d(0)), false))));

        return command;
    }

    public Command getFieldTestAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        System.out.println("yes");
        command.addCommands(
                resetToVision(container),
                new FollowCommand(container.getDrivetrain(), new HolonomicPathBuilder().andThen(
                        new Line(XYconstraints, Rconstraints, new Pose2d(13.4, 5, new Rotation2d(0)), true, .1,
                                .5))));

        return command;
    }

    public Command getAmpTravelAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        System.out.println("yipee");
        command.addCommands(
                resetToVision(container),
                new FollowCommand(container.getDrivetrain(), new HolonomicPathBuilder().andThen(
                        new Line(XYconstraints, Rconstraints, new Pose2d(14.58, 7.1, new Rotation2d(Math.PI / 2)), true,
                                .6,
                                .7))
                        .andThen(new Line(SlowXYconstraints, SlowRconstraints,
                                new Pose2d(14.8, 7.6, new Rotation2d(Math.PI / 2)), true, 0.07, 0.2))));

        return command;
    }

    public Command getShooterTravelAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        System.out.println("yipee");
        command.addCommands(
                resetToVision(container),
                new FollowCommand(container.getDrivetrain(), new HolonomicPathBuilder().andThen(
                        new Line(XYconstraints, Rconstraints, new Pose2d(14.58, 5.7, new Rotation2d(0)), true, .6,
                                .7))
                        .andThen(new Line(SlowXYconstraints, SlowRconstraints,
                                new Pose2d(15.28, 5.7, new Rotation2d(0)), true, 0.07, 0.2))));

        return command;
    }

    public Command resetRobotPose(RobotContainer container, Pose2d pose2d) {
        return new InstantCommand(() -> container.getDrivetrain().seedFieldRelative(pose2d));
    }

    public Command resetRobotPose(RobotContainer container) {
        return new InstantCommand(
                () -> container.getDrivetrain().seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
    }

    public Command resetToVision(RobotContainer container) {
        System.out.println(container.getDrivetrain().getState().Pose);
        return new InstantCommand(() -> container.getDrivetrain().alignToVision());
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