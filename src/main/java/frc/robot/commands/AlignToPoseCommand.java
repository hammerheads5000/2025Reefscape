// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.AutoConstants.APPROACH_DISTANCE;
import static frc.robot.Constants.FieldConstants.ALIGNMENT_FIELD_OBJECT_NAME;
import static frc.robot.Constants.FieldConstants.FIELD;
import static frc.robot.Constants.SwerveConstants.ALIGN_TIME;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlConstants;
import frc.robot.subsystems.Swerve;

/* Aligns robot to given pose (translation + rotation)  */
@Logged
public class AlignToPoseCommand extends Command {
    public Pose2d targetPose;
    public ProfiledPIDController pidControllerX;
    public ProfiledPIDController pidControllerY;
    public PIDController pidControllerAngle;
    private Timer alignedTimer;

    private Swerve swerve;

    /** Creates a new AlignToPoseCommand. */
    public AlignToPoseCommand(Pose2d targetPose, ControlConstants pidConstantsX,
            ControlConstants pidConstantsY, ControlConstants pidConstantsAngle, Swerve swerve) {
        this.targetPose = targetPose;
        this.swerve = swerve;

        pidControllerX = pidConstantsX.getProfiledPIDController();
        pidControllerY = pidConstantsY.getProfiledPIDController();
        pidControllerAngle = pidConstantsAngle.getPIDController();
        pidControllerAngle.enableContinuousInput(-180, 180);

        pidControllerX.setGoal(targetPose.getX());
        pidControllerY.setGoal(targetPose.getY());
        pidControllerAngle.setSetpoint(targetPose.getRotation().getDegrees());

        alignedTimer = new Timer();

        SmartDashboard.putData("Align X PID", pidControllerX);
        SmartDashboard.putData("Align Y PID", pidControllerY);
        SmartDashboard.putData("Align Angle PID", pidControllerAngle);

        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d pose = getPose();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(swerve.getChassisSpeeds(), pose.getRotation());
        pidControllerX.reset(pose.getX(), chassisSpeeds.vxMetersPerSecond);
        pidControllerY.reset(pose.getY(), chassisSpeeds.vyMetersPerSecond);
        pidControllerAngle.reset();

        pidControllerX.setGoal(targetPose.getX());
        pidControllerY.setGoal(targetPose.getY());
        pidControllerAngle.setSetpoint(targetPose.getRotation().getDegrees());

        FIELD.getObject(ALIGNMENT_FIELD_OBJECT_NAME).setPose(targetPose);

        alignedTimer.reset();
        SmartDashboard.putBoolean("Aligned", false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d pose = getPose();
        LinearVelocity xVel = MetersPerSecond.of(pidControllerX.calculate(pose.getX()));
        LinearVelocity yVel = MetersPerSecond.of(pidControllerY.calculate(pose.getY()));
        AngularVelocity angleVel = DegreesPerSecond
                .of(pidControllerAngle.calculate(pose.getRotation().getDegrees()));

        // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red && idfk.get()) {
        //     xVel = xVel.unaryMinus();
        //     yVel = yVel.unaryMinus();
        //     System.out.println("Hi");
        // }
        swerve.driveFieldCentricAbsolute(xVel, yVel, angleVel);

        SmartDashboard.putNumber("Align X Setpoint", pidControllerX.getSetpoint().position);
        SmartDashboard.putNumber("Align Y Setpoint", pidControllerY.getSetpoint().position);
        SmartDashboard.putNumber("Align Angle Setpoint", pidControllerAngle.getSetpoint());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Aligned", true);
        swerve.stop();
    }

    private Pose2d getPose() {
        return swerve.getPose();
    }

    public LinearVelocity calculateXPID(Distance error) {
        return MetersPerSecond.of(pidControllerX.calculate(error.in(Meters)));
    }

    public LinearVelocity calculateYPID(Distance error) {
        return MetersPerSecond.of(pidControllerY.calculate(error.in(Meters)));
    }

    public boolean isAligned() {
        return pidControllerX.atGoal() && pidControllerY.atGoal() && pidControllerAngle.atSetpoint();
    }

    public static boolean withinApproachRange(Pose2d pose, Pose2d target) {
        return withinRange(pose, target, APPROACH_DISTANCE);
    }

    public static boolean withinRange(Pose2d pose, Pose2d target, Distance distance) {
        return pose.getTranslation().getDistance(target.getTranslation()) <= distance.in(Meters);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!alignedTimer.isRunning() && isAligned()) {
            alignedTimer.restart();
        } else if(alignedTimer.isRunning() && !isAligned()) {
            alignedTimer.stop();
        }

        return isAligned() && alignedTimer.hasElapsed(ALIGN_TIME.in(Seconds));
    }
}
