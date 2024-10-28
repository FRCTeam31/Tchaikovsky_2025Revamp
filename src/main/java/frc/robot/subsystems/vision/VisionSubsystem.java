package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import prime.vision.LimelightInputs;

@Logged(strategy = Strategy.OPT_IN)
public class VisionSubsystem extends SubsystemBase {
    private LimeLightNT[] m_limelights;

    @Logged(name = "LimelightInputs", importance = Logged.Importance.CRITICAL)
    private LimelightInputs[] m_limelightInputs;

    public VisionSubsystem(String[] tableNames) {
        setName("VisionSubsystem");
        var defaultInstance = NetworkTableInstance.getDefault();
        m_limelights = new LimeLightNT[tableNames.length];
        for (int i = 0; i < tableNames.length; i++) {
            m_limelights[i] = new LimeLightNT(defaultInstance, tableNames[i]);
        }

        m_limelightInputs = new LimelightInputs[tableNames.length];
    }

    /**
     * Gets the inputs for the specified limelight.
     * @param llIndex The index of the limelight to get inputs from.
     * @return
     */
    public LimelightInputs getLimelightInputs(int llIndex) {
        return m_limelightInputs[llIndex];
    }

    /**
     * Sets limelight’s LED state.
     *    0 = use the LED Mode set in the current pipeline.
     *    1 = force off.
     *    2 = force blink.
     *    3 = force on.
     * @param llIndex The index of the desired limelight
     * @param mode The LED mode to set
     */
    public void setLedMode(int llIndex, int mode) {
        m_limelights[llIndex].setLedMode(mode);
    }

    /**
     * Forces the LED to blink a specified number of times, then returns to pipeline control.
     * @param llIndex The index of the desired limelight
     * @param blinkCount The number of times to blink the LED
     */
    public void blinkLed(int llIndex, int blinkCount) {
        m_limelights[llIndex].blinkLed(blinkCount);
    }

    /**
     * Sets limelight’s operation mode.
     *    0 = Vision processor.
     *    1 = Driver Camera (Increases exposure, disables vision processing).
     * @param llIndex The index of the desired limelight
     * @param mode The camera mode to set
     */
    public void setCameraMode(int llIndex, int mode) {
        m_limelights[llIndex].setCameraMode(mode);
    }

    /**
     * Sets limelight’s active vision pipeline.
     * @param llIndex The index of the desired limelight
     * @param pipeline The pipeline to set active
     */
    public void setPipeline(int llIndex, int pipeline) {
        m_limelights[llIndex].setPipeline(pipeline);
    }

    /**
     * Sets limelight’s streaming mode.
     *    0 = Standard - Side-by-side streams if a webcam is attached to Limelight
     *    1 = PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
     *    2 = PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
     * @param llIndex The index of the desired limelight
     * @param mode The streaming mode to set
     */
    public void setPiPStreamingMode(int llIndex, int mode) {
        m_limelights[llIndex].setPiPStreamingMode(mode);
    }

    /**
     * Takes an instantaneous snapshot of the limelight's camera feed.
     * @param llIndex The index of the desired limelight
     */
    public void takeSnapshot(int llIndex) {
        m_limelights[llIndex].takeSnapshot();
    }

    /**
     * Set the camera's pose in the coordinate system of the robot.
     * @param llIndex The index of the desired limelight
     * @param pose The Camera's pose to set in Robot space
     */
    public void setCameraPose(int llIndex, Pose3d pose) {
        m_limelights[llIndex].setCameraPose(pose);
    }

    public void periodic() {
        // Update all limelight inputs
        for (int i = 0; i < m_limelights.length; i++) {
            m_limelightInputs[i] = m_limelights[i].getInputs();
        }
    }

    public boolean isSpeakerCenterTarget(int apriltagId) {
        return apriltagId == 4 || apriltagId == 7;
    }

    public boolean isValidApriltag(int apriltagId) {
        return apriltagId >= 1 && apriltagId <= 16;
    }
}