// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.INST;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SwerveConstants.*;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
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
    
    public boolean hasTargetFL = false;
    public boolean hasTargetFR = false;
    public boolean hasHadTargetFL = false;
    public boolean hasHadTargetFR = false;
    public boolean hasTarget = false;

    private Swerve swerve;

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

    public boolean flConnected() {
        return camFL.isConnected();
    }
    public boolean frConnected() {
        return camFR.isConnected();
    }

    public Distance getDistanceToEstimatedFL() {
        return Meters.of(swerve.getPose().getTranslation().getDistance(FL_FIELD_OBJECT.getPose().getTranslation()));
    }

    public Distance getDistanceToEstimatedFR() {
        return Meters.of(swerve.getPose().getTranslation().getDistance(FR_FIELD_OBJECT.getPose().getTranslation()));
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
        meters = meters*meters;
        double xDev = VISION_STD_DEV_0M.get(0, 0) + X_DEV_SLOPE*meters;
        double yDev = VISION_STD_DEV_0M.get(1, 0) + Y_DEV_SLOPE*meters;
        double rotDev = VISION_STD_DEV_0M.get(2, 0) + ROT_DEV_SLOPE*meters;

        return VecBuilder.fill(xDev, yDev, rotDev);
    }

    private Distance targetDistance(PhotonTrackedTarget target) {
        return Meters.of(target.getBestCameraToTarget().getTranslation().getNorm());
    }

    private Distance avgTargetDistance(List<PhotonTrackedTarget> targets) {
        double total = 0;
        for (PhotonTrackedTarget target : targets) {
            total += targetDistance(target).in(Meters);
        }
        return Meters.of(total/targets.size());
    }

    // Check if estimated pose ambiguity exceeds MAX_AMBIGUITY
    private boolean isResultAmbiguous(EstimatedRobotPose estimatedRobotPose) {
        Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
        
        if (pose.getX() < 0 || pose.getX() > APRIL_TAGS.getFieldLength()
                || pose.getY() < 0 || pose.getY() > APRIL_TAGS.getFieldWidth()) {
            return true;
        }

        for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
            if (target.getPoseAmbiguity() > MAX_AMBIGUITY) {
                return true;
            }
        }
        
        return false;
    }

    private boolean isPoseInField(Pose2d pose) {
        return !(pose.getX() < 0 || pose.getX() > APRIL_TAGS.getFieldLength()
                || pose.getY() < 0 || pose.getY() > APRIL_TAGS.getFieldWidth());
    }

    private List<Pose2d> posesFromTargets(List<PhotonTrackedTarget> targets) {
        List<Pose2d> poses = new ArrayList<Pose2d>();
        for (PhotonTrackedTarget target : targets) {
            poses.add(fieldLayout.getTagPose(target.fiducialId).get().toPose2d());
        }

        return poses;
    }

    private boolean processResult(PhotonPipelineResult result, PhotonPoseEstimator poseEstimator) {
        Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update(result);
        if (estimatedPoseOptional.isEmpty()) return false;

        EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
        Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();

        Matrix<N3, N1> stdDevs;
        if (result.getMultiTagResult().isPresent()) { // multi-tag result
            stdDevs = VISION_STD_DEV_MULTITAG;
        } else { // single tag result
            PhotonTrackedTarget bestTarget = result.getBestTarget();

            // check that target isn't ambiguous and pose is in field
            if (bestTarget.poseAmbiguity > MAX_AMBIGUITY || !isPoseInField(pose2d)) return false;

            stdDevs = calculateStdDevs(targetDistance(bestTarget));
        }

        swerve.addVisionMeasurement(estimatedPose, stdDevs);

        if (poseEstimator == poseEstimatorFL) {
            FL_FIELD_OBJECT.setPose(pose2d);
            FL_TARGETS_FIELD_OBJECT.setPoses(posesFromTargets(result.targets));
        };
        if (poseEstimator == poseEstimatorFR) {
            FR_FIELD_OBJECT.setPose(pose2d);
            FR_TARGETS_FIELD_OBJECT.setPoses(posesFromTargets(result.targets));
        };

        return true;
    }

    private boolean updatePoseEstimator(PhotonPoseEstimator poseEstimator, PhotonCamera cam) {
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        
        boolean camHasTarget = false;

        for (PhotonPipelineResult result : results) {
            // EstimatedRobotPose estimatedRobotPose = estimatedPoseFromResult(result, poseEstimator);
            // if (estimatedRobotPose == null || isResultAmbiguous(estimatedRobotPose)) {
            //     continue;
            // }
            
            // Matrix<N3, N1> stdDevs = calculateStdDevs(avgTargetDistance(estimatedRobotPose.targetsUsed));
            
            // hasTarget = true;
            // if (poseEstimator == poseEstimatorFL) fieldFL.set(estimatedRobotPose.estimatedPose.toPose2d());
            // if (poseEstimator == poseEstimatorFR) fieldFR.set(estimatedRobotPose.estimatedPose.toPose2d());
            // swerve.addVisionMeasurement(estimatedRobotPose, stdDevs);
            camHasTarget = camHasTarget || processResult(result, poseEstimator);
        }

        return camHasTarget;
    }

    public boolean hasAprilTag() {
        return hasTarget;
    }

    @Override
    public void periodic() {
        if(Utils.isSimulation()) return;
        
        hasTargetFL = updatePoseEstimator(poseEstimatorFL, camFL);
        hasTargetFR = updatePoseEstimator(poseEstimatorFR, camFR);

        hasHadTargetFL = hasTargetFL || hasHadTargetFL;
        hasHadTargetFR = hasTargetFR || hasHadTargetFR;

        hasTarget = hasTargetFL || hasTargetFR;
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(swerve.getPose());
    }
}
