package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.common.control.*;
import frc.common.util.DrivetrainFeedforwardConstants;
import frc.common.util.HolonomicFeedforward;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    public static double SPEED_MULTIPLIER = .65;
    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0)
            * 0.1017
            * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_LENGTH_METERS / 2.0, DRIVETRAIN_WIDTH_METERS / 2.0);

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
            0.891,
            0.15, 0.13592);

    public static final double DRIVETRAIN_CURRENT_LIMIT = 50.0;

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(1.5, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                    FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(3.0), new CentripetalAccelerationConstraint(1.0) };

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(1, 0.02, .06), new PidConstants(1, 0.02, .06),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    private final Pigeon2 pigeon = new Pigeon2(DRIVETRAIN_PIGEON_CAN_ID, Constants.CAN_BUS_NAME_DRIVETRAIN);

    private final SwerveDrivetrainConstants swerveDrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANbusName(Constants.CAN_BUS_NAME_DRIVETRAIN)
            .withPigeon2Id(DRIVETRAIN_PIGEON_CAN_ID);

    private final SwerveModuleConstantsFactory swerveModuleConstantsFactory = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio((16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0))
            .withSteerMotorGearRatio((15.0 / 32.0) * (10.0 / 60.0))
            .withWheelRadius(2)
            .withSteerMotorGains(new Slot0Configs().withKP(0.2).withKI(0).withKD(0.1))
            .withDriveMotorGains(new Slot0Configs().withKP(0.2).withKI(0).withKD(0.1))
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withSpeedAt12VoltsMps(16.5)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withSteerMotorInverted(true);

    private final SwerveModuleConstants frontLeftConstants = swerveModuleConstantsFactory.createModuleConstants(
            FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID, FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
            FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID, 0, Constants.DRIVETRAIN_LENGTH_METERS / 2,
            Constants.DRIVETRAIN_WIDTH_METERS / 2, true);
    private final SwerveModuleConstants frontRightConstants = swerveModuleConstantsFactory.createModuleConstants(
            FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID, FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
            FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID, 0, Constants.DRIVETRAIN_LENGTH_METERS / 2,
            Constants.DRIVETRAIN_WIDTH_METERS / 2, true);
    private final SwerveModuleConstants backLeftConstants = swerveModuleConstantsFactory.createModuleConstants(
            BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID, BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
            BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID, 0, Constants.DRIVETRAIN_LENGTH_METERS / 2,
            Constants.DRIVETRAIN_WIDTH_METERS / 2, true);
    private final SwerveModuleConstants backRightConstants = swerveModuleConstantsFactory.createModuleConstants(
            BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID, BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
            BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID, 0, Constants.DRIVETRAIN_LENGTH_METERS / 2,
            Constants.DRIVETRAIN_WIDTH_METERS / 2, true);

    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(swerveDrivetrainConstants,
            frontLeftConstants, frontRightConstants, backLeftConstants, backRightConstants);

    private final SwerveRequest.FieldCentric m_request = new SwerveRequest.FieldCentric();

    private final Field2d m_field = new Field2d();

    public DrivetrainSubsystem(RobotContainer container) {
        pigeon.getConfigurator().apply(new MountPoseConfigs().withMountPosePitch(180));
        Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME).addNumber("yaw",
                () -> pigeon.getYaw().getValueAsDouble());
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        tab.addNumber("Odometry X", () -> Units.metersToFeet(getPose().getX()));
        tab.addNumber("Odometry Y", () -> Units.metersToFeet(getPose().getY()));
        tab.addNumber("Odometry Angle", () -> getPose().getRotation().getDegrees());
        tab.addNumber("Trajectory Position X", () -> {
            var lastState = follower.getLastState();
            if (lastState == null)
                return 0;

            return Units.metersToFeet(lastState.getPathState().getPosition().x);
        });

        tab.addNumber("Trajectory Velocity X", () -> {
            var lastState = follower.getLastState();
            if (lastState == null)
                return 0;

            return Units.metersToFeet(lastState.getVelocity());
        });
        tab.addNumber("Gyroscope Angle", () -> getGyroscopeRotation().getDegrees());
    }

    public Field2d getField() {
        return m_field;
    }

    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    /**
     * Resets the rotation of the drivetrain to zero.
     */
    public void zeroRotation() {
        System.out.println("reset");
        swerveDrivetrain.getState().Pose = new Pose2d(swerveDrivetrain.getState().Pose.getX(),
                swerveDrivetrain.getState().Pose.getY(), new Rotation2d(0));
    }

    /**
     * Returns the position of the robot
     */
    public Pose2d getPose() {
        return swerveDrivetrain.getState().Pose;
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

    /**
     * Sets the position of the robot to the position passed in with the current
     * gyroscope rotation.
     */
    public void setPose(Pose2d pose) {
        swerveDrivetrain.getState().Pose = pose;
    }

    /**
     * Sets the desired chassis speed of the drivetrain.
     * 
     * @param chassisSpeeds chassisSpeeds
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrivetrain.setControl(
                m_request.withVelocityX(chassisSpeeds.vxMetersPerSecond).withVelocityY(chassisSpeeds.vyMetersPerSecond)
                        .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
    }

    public void periodic() {
        m_field.setRobotPose(swerveDrivetrain.getState().Pose);
    }

    // public void autoBalenceTick() {
    // double theta;
    // double pitch = pigeon.getPitch();
    // double roll = pigeon.getRoll();
    // if (Math.abs(roll) <= 2)
    // if (Math.abs(pitch) <= 2)
    // theta = Double.NaN;
    // else
    // theta = 0;
    // else if (Math.abs(pitch) <= 2)
    // theta = 90;
    // else
    // theta = Math.toDegrees(Math
    // .atan(Math.sin(Math.toRadians(roll)) / Math.sin(Math.toRadians(pitch)))) +
    // 90;
    // System.out.println("Pitch - " + pitch);
    // System.out.println("Roll - " + roll);
    // System.out.println("Theta - " + theta);
    // SmartDashboard.putNumber("Theta", theta);
    // if (theta != Double.NaN)
    // drive(new ChassisSpeeds(Math.sin(Math.toRadians(theta)) * SPEED_MULTIPLIER,
    // Math.cos(Math.toRadians(theta)) * SPEED_MULTIPLIER, 0));
    // }

    public void autoBalenceTick() {
        double pitch = pigeon.getPitch().getValueAsDouble();
        if (pitch > 2.5) {
            drive(new ChassisSpeeds(Math.min(0.4, pitch / 30), 0, 0));
        } else if (pitch < -2.5) {
            drive(new ChassisSpeeds(Math.max(-0.4, pitch / 30), 0, 0));
        } else {
            drive(new ChassisSpeeds(0, 0, 0));
        }
    }

    public void printAngles() {
        System.out.println("Pitch - " + pigeon.getPitch());
        System.out.println("Roll - " + pigeon.getRoll());
    }

    public double getPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }
}