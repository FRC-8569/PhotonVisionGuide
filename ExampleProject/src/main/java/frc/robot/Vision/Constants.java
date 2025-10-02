package frc.robot.Vision;

import static edu.wpi.first.units.Units.Centimeters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static final AprilTagFieldLayout Field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final PoseStrategy strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public class LocalizationCamera {
        public static final String name = "LocalizationCamera";
        public static final Pose3d Place = new Pose3d(Centimeters.of(0),Centimeters.of(0),Centimeters.of(0),Rotation3d.kZero); 
    }
}