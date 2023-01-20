package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.V2_5Drive;

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
    //private int distance = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        V2_5Drive drive = new V2_5Drive(hardwareMap);

        // Set them to float
        // drive.verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // drive.horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set values to zero:
        drive.setVerticalSlide("zero", true);


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
        long last_a_press = System.currentTimeMillis();
        long last_b_press = System.currentTimeMillis();
        long last_y_press = System.currentTimeMillis();
        long last_x_press = System.currentTimeMillis();
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
            telemetry.addData("vslide", drive.verticalSlide.getCurrentPosition());
            telemetry.addData("hslide", drive.horizontalSlide.getCurrentPosition());
            telemetry.update();


            // You can make 250 equal to zero... this was just an example of how you could implement holding a button vs pressing a button...

            if (gamepad1.a) {
                if (currentTime - last_a_press > 250) {
                    last_a_press = System.currentTimeMillis();
                    drive.setVerticalSlide("lowJunction", false);
                }
            }
            if (gamepad1.b) {
                if (currentTime - last_b_press > 250) {
                    last_b_press = System.currentTimeMillis();
                    drive.setVerticalSlide("mediumJunction", false);
                }
            }
            if (gamepad1.y) {
                if (currentTime - last_y_press > 250) {
                    last_y_press = System.currentTimeMillis();
                    drive.setVerticalSlide("highJunction", false);
                }
            }
            if (gamepad1.x) {
                if (currentTime - last_x_press > 250) {
                    last_x_press = System.currentTimeMillis();
                    drive.setVerticalSlide("passing", false);
                }
            }
            if (gamepad1.right_bumper) {
                if (currentTime - last_right_bumper_press > 250) {
                    last_right_bumper_press = System.currentTimeMillis();
                    drive.setGrabber("grab");
                }
            }
            if (gamepad1.left_bumper) {
                if (currentTime - last_left_bumper_press > 250) {
                    last_left_bumper_press = System.currentTimeMillis();
                    drive.setGrabber("release");
                }
            }




            /*if (gamepad1.x==true && emt == 0 && distance>=0) {
                distance = 6000;
                drive.linearStretch(distance);
                emt = 1;
                while (emt == 1){
                    if (gamepad1.x==false){
                        emt = 0;
                    }
                }
            }

             */
            /*if (gamepad1.y==true && emt == 0 && distance>0 && distance<6001) {
                distance = 0;
                drive.linearStretch(distance);
                emt = 1;
                while (emt == 1){
                    if (gamepad1.y==false){
                        emt = 0;
                    }
                }
            }

             */
            /*
            if (gamepad1.x==true) {

                drive.release();

                while (emt1 == 1){
                    if (gamepad1.left_bumper==false){
                        emt1 = 0;
                    }
                }


            }
            */
            /*
            if (gamepad1.y==true ) {

                drive.grab();

                while (emt1 == 1){
                    if (gamepad1.right_bumper==false){
                        emt1 = 0;
                    }
                }


            }
            */


        }
    }
}
