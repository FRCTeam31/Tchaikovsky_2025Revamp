package prime.physics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class LimelightPose {

  public Pose3d Pose;
  public double Timestamp;
  public double TagCount;
  public double TagSpan;
  public double AvgTagDistanceMeters;
  public double AvgTagArea;
  public Matrix<N3, N1> StdDeviations;

  public LimelightPose(double[] data, Matrix<N3, N1> stdDeviations) {
    if (data.length < 6) {
      System.err.println("Bad LL 3D Pose Data!");
      return;
    }

    Pose =
      new Pose3d(
        new Translation3d(data[0], data[1], data[2]),
        new Rotation3d(
          Units.degreesToRadians(data[3]),
          Units.degreesToRadians(data[4]),
          Units.degreesToRadians(data[5])
        )
      );

    var latencyMs = data[6];
    Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    TagCount = data[7];
    TagSpan = data[8];
    AvgTagDistanceMeters = data[9];
    AvgTagArea = data[10];
    StdDeviations = stdDeviations;
  }
}
