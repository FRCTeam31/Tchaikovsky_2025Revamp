package prime.physics;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;

@Logged
public class LimelightPose implements StructSerializable {

  public Pose3d Pose = new Pose3d();
  public double Timestamp = 0.0;
  public double TagCount = 0.0;
  public double TagSpan = 0.0;
  public double AvgTagDistanceMeters = 0.0;
  public double AvgTagArea = 0.0;
  public double[] StdDeviations = new double[3];

  public LimelightPose() {}

  public LimelightPose(double[] data, double[] stdDeviations) {
    if (data.length < 11 || data.length > 11) {
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

  public LimelightPose(Pose3d pose, double[] data, double[] stdDeviations) {
    if (data.length < 5 || data.length > 5) {
      System.err.println("Bad LL 3D Pose Data!");
      return;
    }

    Pose = pose;

    var latencyMs = data[0];
    Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    TagCount = data[1];
    TagSpan = data[2];
    AvgTagDistanceMeters = data[3];
    AvgTagArea = data[4];
    StdDeviations = stdDeviations;
  }

  public LimelightPose(Pose3d pose, double[] data) {
    if (data.length < 5 || data.length > 5) {
      System.err.println("Bad LL 3D Pose Data!");
      return;
    }

    Pose = pose;

    var latencyMs = data[0];
    Timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);
    TagCount = data[1];
    TagSpan = data[2];
    AvgTagDistanceMeters = data[3];
    AvgTagArea = data[4];
  }

  @NotLogged
  public Matrix<N3, N1> getStdDeviations() {
    return new Matrix<N3, N1>(new SimpleMatrix(StdDeviations));
  }

  /** Struct for serialization. */
  public static final LimelightPoseStruct struct = new LimelightPoseStruct();
}
