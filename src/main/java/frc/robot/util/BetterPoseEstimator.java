package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.Comparator;

public class BetterPoseEstimator {

  private SwerveDrivePoseEstimator poseEstimator;
  private Pose2d pose;
  private Rotation2d lastGyroAngle;
  private SwerveModulePosition[] lastStates = new SwerveModulePosition[4];

  private ArrayList<VisionUpdate> visionUpdates = new ArrayList<>();

  public BetterPoseEstimator(SwerveDrivePoseEstimator poseEstimator, Pose2d p) {
    this.poseEstimator = poseEstimator;
    this.pose = p;
    this.lastGyroAngle = new Rotation2d();
    for (int i = 0; i < 4; i++) {
      lastStates[i] = new SwerveModulePosition();
    }
  }

  public void updateOdometry(
      double timestamp, Rotation2d gyroAngle, SwerveModulePosition[] positions) {
    this.pose = poseEstimator.updateWithTime(timestamp, gyroAngle, positions);
    this.lastGyroAngle = gyroAngle;
    this.lastStates = positions;
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    this.poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);

    // VisionUpdate visionUpdate = new VisionUpdate(pose, stdDevs, timestamp);
    // Calculate Kalman gains based on std devs
    // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
    // Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    // var r = new double[3];
    // for (int i = 0; i < 3; ++i) {
    //   r[i] = visionUpdate.stdDevs().get(i, 0) * visionUpdate.stdDevs().get(i, 0);
    // }
    // for (int row = 0; row < 3; ++row) {
    //   if (visionK.get(row, 0) == 0.0) {
    //     visionK.set(row, row, 0.0);
    //   } else {
    //     visionK.set(
    //         row,
    //         row,
    //         visionK.get(row, 0) / (visionK.get(row, 0) + Math.sqrt(visionK.get(row, 0) *
    // r[row])));
    //   }
    // }
    // Calculate twist between current and vision pose
    // var visionTwist = this.pose.log(visionUpdate.pose());

    // // Multiply by Kalman gain matrix
    // var twistMatrix =
    //     visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));
    // // Apply twist

    // this.pose =
    //     this.pose.exp(
    //         new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    this.poseEstimator.resetPosition(pose.getRotation(), lastStates, pose);
  }

  public static record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
    public static final Comparator<VisionUpdate> compareDescStdDev =
        (VisionUpdate a, VisionUpdate b) -> {
          return -Double.compare(
              a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
              b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
        };
  }
}
