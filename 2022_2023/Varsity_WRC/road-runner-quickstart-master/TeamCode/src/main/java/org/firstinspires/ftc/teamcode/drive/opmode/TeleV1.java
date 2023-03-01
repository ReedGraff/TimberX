package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how to create a teleop using just the SampleMecanumDrive class without
 * the need for an external robot class. This will allow you to do some cool things like
 * incorporating live trajectory following in your teleop. Check out TeleOpAgumentedDriving.java for
 * an example of such behavior.
 * <p>
 * This opmode is essentially just LocalizationTest.java with a few additions and comments.
 */
@TeleOp(group = "teleop")
public class v1Tele extends LinearOpMode {
    private int grabLiftCase = 0;
    private int grabCase = 0;
    private int junction = 0;
    private int horizontalDis = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set them to float
        // drive.verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // drive.horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set values to zero:
        drive.setLaunch("zero", true);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;

        // Define times for buttons...
        long last_left_bumper_press = System.currentTimeMillis();
        long last_right_bumper_press = System.currentTimeMillis();


        while (opModeIsActive() && !isStopRequested()) {
            long currentTime = System.currentTimeMillis();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Launch", drive.launch.getCurrentPosition());
            telemetry.update();


            if (gamepad1.left_bumper) {
                if (currentTime - last_left_bumper_press > 500) {
                    last_left_bumper_press = System.currentTimeMillis();
                    drive.setLaunch("down", false);
                }
            }
            if (gamepad1.right_bumper) {
                if (currentTime - last_right_bumper_press > 500) {
                    last_right_bumper_press = System.currentTimeMillis();
                    drive.setLaunch("up", false);
                }
            }
        }
    }
}