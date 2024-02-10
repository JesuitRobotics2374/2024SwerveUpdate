package frc.robot.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    ShuffleboardTab tab = Shuffleboard.getTab(Constants.DRIVER_READOUT_TAB_NAME);
    Field2d field = new Field2d();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry pose = table.getEntry("botpose");
    ArrayList<Double> xList = new ArrayList<>();
    ArrayList<Double> yList = new ArrayList<>();
    ArrayList<Double> rList = new ArrayList<>();
    ArrayList<Double> tList = new ArrayList<>();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        tab.add(field);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        tab.add(field);
        tab.addDouble("Offset", () -> this.m_fieldRelativeOffset.getDegrees());
        Matrix<N3, N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
        matrix.set(0, 0, 4);
        matrix.set(1, 0, 4);
        matrix.set(2, 0, .9);
        setVisionMeasurementStdDevs(matrix);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        double[] array = pose.getDoubleArray(new double[0]);
        boolean flag = isTargetValid(array);
        if (array.length > 0) {
            field.getObject("Vision").setPose(
                    new Pose2d(array[0] + 8.308975, array[1] + 4.098925, new Rotation2d(Math.toRadians(array[5]))));
            if (field.getObject("Vision").getPose().getX() != 0 || field.getObject("Vision").getPose().getX() != 0) {
                double offset = Math
                        .sqrt(Math.pow(field.getObject("Vision").getPose().relativeTo(getState().Pose).getX(), 2)
                                + Math.pow(field.getObject("Vision").getPose().relativeTo(getState().Pose).getY(), 2));
                if (flag && offset < 1 && Math.abs(field.getObject("Vision").getPose().getRotation().getDegrees()
                        - getState().Pose.getRotation().getDegrees()) < 30) {
                    addVisionMeasurement(field.getObject("Vision").getPose(),
                            Timer.getFPGATimestamp() - (array[6] / 1000.0));
                } // Timer.getFPGATimestamp() - (botpose[6]/1000.0)
            }
        }
        field.setRobotPose(getState().Pose);
    }

    public boolean isTargetValid(double[] array) {
        xList.add(array[0]);
        yList.add(array[1]);
        rList.add(array[5]);
        tList.add(array[6]);
        if (xList.size() < 20) {
            return false;
        }
        double[] x = new double[xList.size() - 1];
        double[] y = new double[yList.size() - 1];
        double[] r = new double[rList.size() - 1];

        for (int i = 0; i < x.length; i++) {
            x[i] = (xList.get(i) - xList.get(i + 1)) / (tList.get(i + 1) / 1000);
            y[i] = (yList.get(i) - yList.get(i + 1)) / (tList.get(i + 1) / 1000);
            r[i] = (rList.get(i) - rList.get(i + 1)) / (tList.get(i + 1) / 1000);
        }
        for (int i = 0; i < x.length - 1; i++) {
            if (Math.abs(x[i] - x[i + 1]) > 1.6 || Math.abs(y[i] - y[i + 1]) > 1.6 || Math.abs(x[i] - x[i + 1]) > 135) {
                xList.remove(0);
                yList.remove(0);
                rList.remove(0);
                tList.remove(0);
                return false;
            }
        }

        // System.out.println(x[0] - x[1]);
        xList.remove(0);
        yList.remove(0);
        rList.remove(0);
        tList.remove(0);
        return true;
    }

    public void alignToVision() {
        System.out.println(field.getObject("Vision").getPose());
        // getState().Pose = field.getObject("Vision").getPose();
        seedFieldRelative(new Pose2d(field.getObject("Vision").getPose().getTranslation(), new Rotation2d()));
        seedFieldRelative();
        seedFieldRelative(field.getObject("Vision").getPose());
    }
}
