package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem.CommandSwerveDrivetrain;

/**
 * AlignToSpeakerCommand
 */
public class AlignToSpeakerCommand extends Command {

    CommandSwerveDrivetrain subsystem;
    ProfiledPIDController controller;
    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric().withRotationalDeadband(0.01);

    public AlignToSpeakerCommand(CommandSwerveDrivetrain subDrivetrain) {
        System.out.println("ALIGN TO THE TING");
        subsystem = subDrivetrain;
        addRequirements(subDrivetrain);
        controller = new ProfiledPIDController(2, 0, 0.1, new Constraints(Math.PI * 4, Math.PI * 3));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(0.12, 0.4);
    }

    @Override
    public void initialize() {
        boolean flag = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            flag = alliance.get() == Alliance.Red;
        }
        Translation2d offset = (flag ? new Translation2d(16.3, 5.55) : new Translation2d(0.3, 5.55))
                .minus(subsystem.getState().Pose.getTranslation());
        controller.setGoal(offset.getAngle().getRadians());
        System.out.println(Math.toDegrees(controller.getGoal().position));
    }

    @Override
    public void execute() {
        double rate = controller.calculate(subsystem.getState().Pose.getRotation().getRadians());
        if (rate < 0) {
            rate = Math.min(rate, -1);
        } else {
            rate = Math.max(rate, 1);
        }
        System.out.println(rate);
        subsystem.setControl(request.withRotationalRate(rate));
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal();
    }
}