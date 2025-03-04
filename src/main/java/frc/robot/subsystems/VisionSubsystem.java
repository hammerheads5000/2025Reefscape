// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.INST;
import static frc.robot.Constants.SwerveConstants.VISION_STD_DEV_0M;
import static frc.robot.Constants.SwerveConstants.VISION_STD_DEV_5M;
import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

@Logged
public class VisionSubsystem extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout;

    private PhotonPoseEstimator poseEstimatorFL; // front left
    private PhotonPoseEstimator poseEstimatorFR; // front right
    //private PhotonPoseEstimator poseEstimatorB; // back

    private PhotonCamera camFL = new PhotonCamera(FRONT_LEFT_CAM_NAME); // front left
    private PhotonCamera camFR = new PhotonCamera(FRONT_RIGHT_CAM_NAME); // front right
    //private PhotonCamera camB = new PhotonCamera(BACK_CAM_NAME); // back
    
    private boolean hasTarget = false;

    private StructPublisher<Pose2d> fieldFL = INST.getStructTopic("Vision/FL Pose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> fieldFR = INST.getStructTopic("Vision/FR Pose", Pose2d.struct).publish();

    private Swerve swerve;

    public Distance lastPoseChange = Meters.of(-1);
    private Time lastPoseChangeTime = Seconds.zero();

    // Simulation
    VisionSystemSim visionSim = new VisionSystemSim("main");
    SimCameraProperties camProperties = new SimCameraProperties();
    PhotonCameraSim camFLSim = new PhotonCameraSim(camFL, camProperties);
    PhotonCameraSim camFRSim = new PhotonCameraSim(camFR, camProperties);
    //PhotonCameraSim camBSim = new PhotonCameraSim(camB, camProperties);

    private double X_DEV_SLOPE = (VISION_STD_DEV_0M.get(0, 0) - VISION_STD_DEV_5M.get(0, 0)) / 5;
    private double Y_DEV_SLOPE = (VISION_STD_DEV_0M.get(1, 0) - VISION_STD_DEV_5M.get(1, 0)) / 5;
    private double ROT_DEV_SLOPE = (VISION_STD_DEV_0M.get(2, 0) - VISION_STD_DEV_5M.get(2, 0)) / 5;

    /** Creates a new AprilTagSubsystem. */
    public VisionSubsystem(Swerve swerve) {
        this.swerve = swerve;
        
        fieldLayout = FieldConstants.APRIL_TAGS;
        poseEstimatorFL = new PhotonPoseEstimator(fieldLayout, POSE_STRATEGY, FRONT_LEFT_CAM_POS);
        poseEstimatorFR = new PhotonPoseEstimator(fieldLayout, POSE_STRATEGY, FRONT_RIGHT_CAM_POS);
        //poseEstimatorB = new PhotonPoseEstimator(fieldLayout, POSE_STRATEGY, BACK_CAM_POS);
    
        visionSim.addAprilTags(fieldLayout);
        camProperties.setCalibration(800, 600, Rotation2d.fromDegrees(82.4));
        camProperties.setCalibError(0.25, 0.08);
        camProperties.setFPS(30);
        camProperties.setAvgLatencyMs(35);
        camProperties.setLatencyStdDevMs(5);
        visionSim.addCamera(camFLSim, FRONT_LEFT_CAM_POS);
        visionSim.addCamera(camFRSim, FRONT_RIGHT_CAM_POS);
        //visionSim.addCamera(camBSim, BACK_CAM_POS);
        
        camFLSim.enableDrawWireframe(true);
        camFRSim.enableDrawWireframe(true);
        //camBSim.enableDrawWireframe(true);
    }

    public boolean camerasConnected() {
        return camFL.isConnected() && camFR.isConnected();// && camB.isConnected();
    }

    public Time timeSinceHadTarget() {
        return RobotController.getMeasureTime().minus(lastPoseChangeTime);
    }

    private EstimatedRobotPose estimatedPoseFromResult(PhotonPipelineResult result, PhotonPoseEstimator poseEstimator) {
        Optional<EstimatedRobotPose> optionalPose = poseEstimator.update(result);
        if (optionalPose.isEmpty()) {
            return null;
        }

        EstimatedRobotPose estimatedRobotPose = optionalPose.get();
        return estimatedRobotPose;
    }

    private Matrix<N3, N1> calculateStdDevs(Distance distance) {
        double meters = distance.in(Meters);
        double xDev = VISION_STD_DEV_0M.get(0, 0) + X_DEV_SLOPE*meters;
        double yDev = VISION_STD_DEV_0M.get(1, 0) + Y_DEV_SLOPE*meters;
        double rotDev = VISION_STD_DEV_0M.get(2, 0) + ROT_DEV_SLOPE*meters;

        return VecBuilder.fill(xDev, yDev, rotDev);
    }

    private Distance avgTargetDistance(List<PhotonTrackedTarget> targets) {
        double total = 0;
        for (PhotonTrackedTarget target : targets) {
            total += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        return Meters.of(total/targets.size());
    }

    // Check if estimated pose ambiguity exceeds MAX_AMBIGUITY
    private boolean isResultAmbiguous(EstimatedRobotPose estimatedRobotPose) {
        for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
            if (target.getPoseAmbiguity() > MAX_AMBIGUITY) {
                return true;
            }
        }
        
        return false;
    }

    private boolean updatePoseEstimator(PhotonPoseEstimator poseEstimator, PhotonCamera cam) {
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        
        hasTarget = false;

        for (PhotonPipelineResult result : results) {
            EstimatedRobotPose estimatedRobotPose = estimatedPoseFromResult(result, poseEstimator);
            if (estimatedRobotPose == null || isResultAmbiguous(estimatedRobotPose)) {
                continue;
            }
            
            Matrix<N3, N1> stdDevs = calculateStdDevs(avgTargetDistance(estimatedRobotPose.targetsUsed));
            
            hasTarget = true;
            if (poseEstimator == poseEstimatorFL) fieldFL.set(estimatedRobotPose.estimatedPose.toPose2d());
            if (poseEstimator == poseEstimatorFR) fieldFR.set(estimatedRobotPose.estimatedPose.toPose2d());

            Pose2d poseBefore = swerve.getPose();
            swerve.addVisionMeasurement(estimatedRobotPose, stdDevs);
            Pose2d poseAfter = swerve.getPose();

            lastPoseChange = Meters.of(poseAfter.getTranslation().getDistance(poseBefore.getTranslation()));
            lastPoseChangeTime = RobotController.getMeasureTime();
        }

        return hasTarget;
    }

    public boolean hasAprilTag() {
        return hasTarget;
    }

    @Override
    public void periodic() {
        if(Utils.isSimulation()) return;
        
        hasTarget = updatePoseEstimator(poseEstimatorFL, camFL) ||
                updatePoseEstimator(poseEstimatorFR, camFR);// ||
                //updatePoseEstimator(poseEstimatorB, camB);
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(swerve.getPose());
    }
}
