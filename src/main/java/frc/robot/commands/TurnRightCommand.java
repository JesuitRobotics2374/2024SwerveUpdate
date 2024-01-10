
package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnRightCommand extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final double targetDegrees;
    private final double initialDegrees;

    public TurnRightCommand(DrivetrainSubsystem drivetrain, double degrees) {
        this.drivetrain = drivetrain;
        this.initialDegrees = drivetrain.getYaw();
        this.targetDegrees = initialDegrees - degrees;

        addRequirements(drivetrain);
    }

    
    @Override
    public boolean isFinished() {
        ChassisSpeeds turnInstruction = new ChassisSpeeds(0.0, 0.0, -2); // turn speed = RadiansPerSecond
        
        if (targetDegrees*.99 < drivetrain.getYaw()){  // account for execution delay
            //  System.out.println("NOT DONE="+targetDegrees+" "+drivetrain.getYaw());
            drivetrain.drive(turnInstruction);
            return false;
        }
        drivetrain.drive(new ChassisSpeeds(0.0,0.0,0.0));
        
        return true;
    }
}
