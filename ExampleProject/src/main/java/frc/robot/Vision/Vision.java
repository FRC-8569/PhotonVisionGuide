package frc.robot.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    public PhotonCamera LocalizationCamera;
    public PhotonPoseEstimator PoseEstimator;

    public StructPublisher<Pose3d> RawPose3d;
    public StructArrayPublisher<Pose3d> VisionTargets;
    public StructSubscriber<Pose2d> RobotPose;
    public BooleanPublisher isPhotonUpdated;
    public static Vision vision;

    public PhotonCameraSim LocalizationCamraSim;
    public VisionSystemSim SimSystem;
    public Notifier SimNotifer;

    private Vision(){
        LocalizationCamera = new PhotonCamera(Constants.LocalizationCamera.name);
        PoseEstimator = new PhotonPoseEstimator(Constants.Field, Constants.strategy, new Transform3d(Constants.LocalizationCamera.Place.toMatrix()));
        RawPose3d = NetworkTableInstance.getDefault().getStructTopic("Vision/RawPose", Pose3d.struct).publish();
        VisionTargets = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Targets", Pose3d.struct).publish();
        isPhotonUpdated = NetworkTableInstance.getDefault().getBooleanTopic("Vision/isAvaliable").publish();
        RobotPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotPose", Pose2d.struct).subscribe(null);
        if(RobotBase.isSimulation()) simInit();
    }

    public Pose2d getPose(){
        if(!LocalizationCamera.isConnected()){
            isPhotonUpdated.accept(false);
            return null;
        }
        else{
            try{
                var results = LocalizationCamera.getAllUnreadResults();
                if(results.isEmpty()) return null;
                var pose = PoseEstimator.update(results.get(results.size() - 1));
                List<Pose3d> t = new ArrayList<Pose3d>();
                RawPose3d.accept(pose.orElseThrow().estimatedPose);

                for(var tar : results.get(results.size() - 1).targets){
                    t.add(pose.orElseThrow().estimatedPose.plus(new Transform3d(Constants.LocalizationCamera.Place.toMatrix())).transformBy(tar.getBestCameraToTarget()));
                }
                VisionTargets.accept(t.toArray(Pose3d[]::new));
                isPhotonUpdated.accept(true);
                return pose.orElseThrow().estimatedPose.toPose2d();
            }catch(Exception e){
                isPhotonUpdated.accept(false);
                DriverStation.reportWarning("Error estmating pose", e.getStackTrace());
                return null;
            }
        }
    }

    private void simInit(){
        LocalizationCamraSim = new PhotonCameraSim(LocalizationCamera,SimCameraProperties.LL2_1280_720());
        SimSystem = new VisionSystemSim("SimVision");
        SimSystem.addCamera(LocalizationCamraSim, new Transform3d(Constants.LocalizationCamera.Place.toMatrix()));
        SimSystem.addAprilTags(Constants.Field);
    }

    @Override
    public void simulationPeriodic(){
        SimSystem.update(RobotPose.get());
    }

    public static Vision getInstance(){
        if(vision == null) vision = new Vision();
        return vision;
    }
}
