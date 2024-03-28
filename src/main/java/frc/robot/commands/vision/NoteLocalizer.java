package frc.robot.commands.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.RingBuffer;

public class NoteLocalizer extends Command {
    private final Vision vision;
    private final Supplier<Pose2d> odometry;

    private final RingBuffer<Pose2d> buffer = new RingBuffer<>(128);

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

    private Translation2d lastVisionReading = null;

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
        buffer.add(odometry.get());

        if (vision.hasTarget()) {
            Translation2d newVisionReading = vision.getTarget();
            if (newVisionReading != lastVisionReading && newVisionReading != null) {
                Optional<Pose2d> oldPosition = buffer.get((int) (vision.latency() / 0.02));

                if (oldPosition.isPresent()) {
                    Translation2d notePosition = oldPosition.get().getTranslation().plus(newVisionReading.rotateBy(oldPosition.get().getRotation()));
                    Matrix<N2, N1> y = new Matrix<>(Nat.N2(), Nat.N1());
                    y.set(0, 0, notePosition.getX());
                    y.set(1, 0, notePosition.getY());
                    filter.correct(new Matrix<N0, N1>(Nat.N0(), Nat.N1()), y);
                }
            }
        }

        filter.predict(new Matrix<>(Nat.N0(), Nat.N1()), 0.02);
    }

    public Translation2d getNotePosition() {
        Matrix state = filter.getXhat();

        return new Translation2d(state.get(0, 0), state.get(1, 0));
    }
}
