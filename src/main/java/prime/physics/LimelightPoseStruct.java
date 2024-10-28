package prime.physics;

import java.nio.ByteBuffer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;

public class LimelightPoseStruct implements Struct<LimelightPose> {
    @Override
    public Class<LimelightPose> getTypeClass() {
        return LimelightPose.class;
    }

    @Override
    public String getTypeName() {
        return "LimelightPose";
    }

    @Override
    public int getSize() {
        var size = Pose3d.struct.getSize() + (kSizeDouble * 5) + Matrix.getStruct(Nat.N3(), Nat.N1()).getSize();

        return size;
    }

    @Override
    public String getSchema() {
        return "limelight pose";
    }

    @Override
    public LimelightPose unpack(ByteBuffer bb) {
        var pose = Pose3d.struct.unpack(bb);

        var data = new double[5];
        for (int i = 0; i < 5; i++) {
        data[i] = bb.getDouble(i * kSizeDouble);
        }

        var stdDeviations = Matrix.getStruct(Nat.N3(), Nat.N1()).unpack(bb.position(kSizeDouble * 11));
        
        return new LimelightPose(pose, data, stdDeviations);
    }

    @Override
    public void pack(ByteBuffer bb, LimelightPose value) {
        Pose3d.struct.pack(bb, value.Pose); 
        bb.putDouble(value.Timestamp);
        bb.putDouble(value.TagCount);
        bb.putDouble(value.TagSpan);
        bb.putDouble(value.AvgTagDistanceMeters);
        bb.putDouble(value.AvgTagArea);
        Matrix.getStruct(Nat.N3(), Nat.N1()).pack(bb, value.StdDeviations);
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
}
