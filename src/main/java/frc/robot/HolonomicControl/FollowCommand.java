package frc.robot.HolonomicControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowCommand extends Command {

    HolonomicPathBuilder builder;
    DrivetrainSubsystem subsystem;

    public FollowCommand(DrivetrainSubsystem subsystem, HolonomicPathBuilder builder) {
        this.builder = builder;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        builder.pathList.peekFirst().initialize(subsystem.getPose());
    }

    @Override
    public void execute() {
        subsystem.drive(builder.pathList.peekFirst().getMovement(subsystem.getPose()));
        builder.clense(subsystem.getPose());
    }

    @Override
    public boolean isFinished() {
        return builder.isFinished();
    }
}
