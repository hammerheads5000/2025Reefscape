// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private Swerve swerve;

    // Simulation
    VisionSystemSim visionSim = new VisionSystemSim("main");
    SimCameraProperties camProperties = new SimCameraProperties();
    PhotonCameraSim camFLSim = new PhotonCameraSim(camFL, camProperties);
    PhotonCameraSim camFRSim = new PhotonCameraSim(camFR, camProperties);
    //PhotonCameraSim camBSim = new PhotonCameraSim(camB, camProperties);

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

    private EstimatedRobotPose estimatedPoseFromResult(PhotonPipelineResult result, PhotonPoseEstimator poseEstimator) {
        Optional<EstimatedRobotPose> optionalPose = poseEstimator.update(result);
        if (optionalPose.isEmpty()) {
            return null;
        }

        EstimatedRobotPose estimatedRobotPose = optionalPose.get();
        return estimatedRobotPose;
    }

    private boolean updatePoseEstimator(PhotonPoseEstimator poseEstimator, PhotonCamera cam) {
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        hasTarget = false;

        for (PhotonPipelineResult result : results) {
            EstimatedRobotPose estimatedRobotPose = estimatedPoseFromResult(result, poseEstimator);
            if (estimatedRobotPose == null) {
                continue;
            }
            
            hasTarget = true;
            swerve.addVisionMeasurement(estimatedRobotPose);
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
