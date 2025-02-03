// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

@Logged
public class VisionSubsystem extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout;

    private PhotonPoseEstimator poseEstimatorFL; // front left
    private PhotonPoseEstimator poseEstimatorFR; // front right
    private PhotonPoseEstimator poseEstimatorB; // back

    private PhotonCamera camFL = new PhotonCamera(FRONT_LEFT_CAM_NAME); // front left
    private PhotonCamera camFR = new PhotonCamera(FRONT_RIGHT_CAM_NAME); // front right
    private PhotonCamera camB = new PhotonCamera(BACK_CAM_NAME); // back
    
    private DoubleArrayPublisher publisher;

    private boolean hasTarget = false;

    /** Creates a new AprilTagSubsystem. */
    public VisionSubsystem() {
        fieldLayout = FieldConstants.APRIL_TAGS;
        poseEstimatorFL = new PhotonPoseEstimator(fieldLayout, POSE_STRATEGY, FRONT_LEFT_CAM_POS);
        poseEstimatorFR = new PhotonPoseEstimator(fieldLayout, POSE_STRATEGY, FRONT_RIGHT_CAM_POS);
        poseEstimatorB = new PhotonPoseEstimator(fieldLayout, POSE_STRATEGY, BACK_CAM_POS);
        publisher = POSE_TOPIC.publish();
    }

    public boolean updatePoseEstimator(PhotonPoseEstimator poseEstimator, PhotonCamera cam) {
        Optional<EstimatedRobotPose> optionalPose;
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();

        
        for (PhotonPipelineResult result : results) {
            optionalPose = poseEstimator.update(result);
        }

        if (optionalPose.isEmpty()) {
            hasTarget = false;
            return false;
        }

        hasTarget = true;
        EstimatedRobotPose estimatedRobotPose = optionalPose.get();
        Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();

        // represents the pose as a double array with {x (meters), y (meters), rotation
        // (radians)}
        double[] poseArray = new double[] {
                pose.getX(), pose.getY(),
                pose.getRotation().getRadians()
        };
        long timeMicroseconds = (long) (Seconds.of(estimatedRobotPose.timestampSeconds).in(Microseconds));

        publisher.set(poseArray, timeMicroseconds);
    }

    public boolean hasAprilTag() {
        return hasTarget;
    }

    @Override
    public void periodic() {
        updatePoseEstimator(poseEstimatorFL, camFL);
        updatePoseEstimator(poseEstimatorFR, camFR);
        updatePoseEstimator(poseEstimatorB, camB);
    }
}
