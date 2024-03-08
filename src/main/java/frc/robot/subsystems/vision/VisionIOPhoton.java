package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOPhoton implements ApriltagIO {

  public final PhotonCamera camera;
  public final PhotonPoseEstimator poseEstimator;
  public AprilTagFieldLayout fieldLayout = null;
  public final Transform3d transform;
  private final VisionProcessingThread thread;

  public final Lock visionLock = new ReentrantLock();
  public final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);
  public final Queue<Pose3d> poses = new ArrayBlockingQueue<>(100);
  public final Queue<Integer[]> tids = new ArrayBlockingQueue<>(100);
  public final Queue<Double> ambiguity = new ArrayBlockingQueue<>(100);

  public VisionIOPhoton(String camname, Transform3d camToRobot) {
    this.camera = new PhotonCamera(camname);

    // this is a listener for any changes on the photonvision networktable. might need to change
    // this later. this is async
    // maybe change this to just a while true loop in a thread
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("exception");
      e.printStackTrace();
    }
    this.poseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camToRobot);
    this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    this.transform = camToRobot;
    thread = new VisionProcessingThread(this);
  }

  @Override
  public void updateInputs(ApriltagIOInputs inputs) {

    double[] timestampsArray =
        timestamps.stream()
            .mapToDouble(Double::doubleValue) // Convert Double to double
            .toArray();
    inputs.timestamps = timestampsArray;
    inputs.poses = poses.toArray(new Pose3d[0]);

    int[][] result = new int[tids.size()][];
    int index = 0;

    double[] ambiguityArray =
        ambiguity.stream()
            .mapToDouble(Double::doubleValue) // Convert Double to double
            .toArray();
    inputs.ambiguity = ambiguityArray;

    while (!tids.isEmpty()) {
      if (index >= result.length) {
        // Handle the case where the result array is not large enough
        break;
      }

      Integer[] currentArray = tids.poll();
      int[] intArray = new int[currentArray.length];
      for (int i = 0; i < currentArray.length; i++) {
        intArray[i] = currentArray[i]; // Auto-unboxing from Integer to int
      }
      result[index++] = intArray;
    }
    inputs.tids = result;
    inputs.pipelineID = camera.getPipelineIndex();
  }

  @Override
  public void setPipeline(int pipelineID) {
    camera.setPipelineIndex(pipelineID);
  }

  @Override
  public Transform3d getTransform() {
    return transform;
  }
}
