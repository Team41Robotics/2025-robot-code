package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.drive;
import frc.robot.util.Util;

public class PhotonVision extends SubsystemBase{
	private final PhotonCamera reef_cam;
	private final PhotonCamera station_cam;
	AprilTagFieldLayout layout = Util.fieldLayout;

	Transform3d robotToCamReef = new Transform3d(
			new Translation3d(Units.inchesToMeters(-10.4085), Units.inchesToMeters(7.063), Units.inchesToMeters(10.413)),
			new Rotation3d(0, 0, Math.PI));

	Transform3d robotToCamStation = new Transform3d(
			new Translation3d(Units.inchesToMeters(12.275), Units.inchesToMeters(6.455),Units.inchesToMeters(7.589)),
			new Rotation3d(0, 0, 0));

	PhotonPoseEstimator photonPoseEstimatorReef;
	PhotonPoseEstimator photonPoseEstimatorStation;


	public PhotonVision() {
		
		reef_cam = new PhotonCamera("ReefCam");
		station_cam = new PhotonCamera("StationCam");

		photonPoseEstimatorReef =
				new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamReef);
		photonPoseEstimatorReef.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		photonPoseEstimatorStation =
				new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamStation);
		photonPoseEstimatorStation.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	@Override
	public void periodic(){
		getMeasurements(reef_cam, photonPoseEstimatorReef);
		getMeasurements(station_cam, photonPoseEstimatorStation);
	}

	public void updateLogging(){
		
	}

	public void getMeasurements(PhotonCamera cam, PhotonPoseEstimator estimator){
		List<PhotonPipelineResult> camPipeline = cam.getAllUnreadResults();
		for(int i = 0; i < camPipeline.size(); i++){
			Optional<EstimatedRobotPose> camPose = estimator.update(camPipeline.get(i));
			if(camPose.isPresent()){
				double distToTag  = camPipeline.get(i).getBestTarget().bestCameraToTarget.getTranslation().getNorm();
				double tagAmbiguity = camPipeline.get(i).getBestTarget().getPoseAmbiguity();
				double bearingToTag = camPipeline.get(i).getBestTarget().getYaw();
				Logger.recordOutput("Photon/Dist " + cam.getName(), distToTag);
				Logger.recordOutput("Photon/Bearing " + cam.getName(), bearingToTag);
				Logger.recordOutput("Photon/Estimated Pose", camPose.get().estimatedPose.toPose2d());		

				if(distToTag < 5 && tagAmbiguity < 0.05){
					drive.addLimelightMeasurement(camPose.get().estimatedPose.toPose2d(), Timer.getFPGATimestamp(), getStdDevs(distToTag));
				}
			}
		}
	}

	public Matrix<N3, N1> getStdDevs(double dist){ // TODO
		return VecBuilder.fill(0.7,0.7,0.7);
	}


}