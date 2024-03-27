package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.RingBuffer;

public class NoteLocalizer extends Command {
    private final Vision vision;
    private final Supplier<Pose2d> odometry;

    private final RingBuffer<Translation2d> buffer = new RingBuffer<>(128);

    // x' = Ax + Bu
    // y = Cx + Du

    // 1 0
    // 0 1
    private final Matrix<N2, N2> A = new Matrix<>(Nat.N2(), Nat.N2());
    private final Matrix<N2, N0> B = new Matrix<>(Nat.N2(), Nat.N0());
    private final Matrix<N2, N2> C = new Matrix<>(Nat.N2(), Nat.N2());
    private final Matrix<N2, N0> D = new Matrix<>(Nat.N2(), Nat.N0());

    private final LinearSystem<N2, N0, N2> system = new LinearSystem<>(A, B, C, D);

    private final Matrix<N2, N2> stateStdDevs = new Matrix<>(Nat.N2(), Nat.N2());
    private final Matrix<N2, N2> measurementStdDevs = new Matrix<>(Nat.N2(), Nat.N2());

    private final KalmanFilter<N2, N0, N2> filter = new KalmanFilter<>(Nat.N2(), Nat.N2(), system, null, null, 0.02);

    public NoteLocalizer(Vision vision, Supplier<Pose2d> odometry) {
        this.vision = vision;
        this.odometry = odometry;

        A.set(0, 0, 1);
        A.set(1, 1, 1);

        C.set(0, 0, 1);
        C.set(1, 1, 1);

        stateStdDevs.set(0, 0, 0.006);
        stateStdDevs.set(1, 1, 0.006);

        measurementStdDevs.set(0, 0, 0.1);
        measurementStdDevs.set(1, 1, 0.1);
    }

    @Override
    public void execute() {
        
    }
}
