package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.V2_5Drive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "left")
public class v1AutonLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        V2_5Drive drive = new V2_5Drive(hardwareMap);

        // Start position
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // Build the trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(90)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(-42, -6), Math.toRadians(85))
                .addTemporalMarker(0.5, () -> {
                    // Where we define peripheral movements...
                    drive.setHorizontalSlide("leftFromLeft");
                })
                .build();

        // Initialization ends, and the round starts
        waitForStart();
        if (isStopRequested()) return;

        // Movements
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);

        // Telemetry
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }
}

/*
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(1000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );


 */