package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.dripackage org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class V2_5Drive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    // Used when strafing is undershooting / overshooting the target...
    public static double LATERAL_MULTIPLIER = 40 / 36.5;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    // Peripherals
    public DcMotorEx horizontalSlide, verticalSlide;
    public Servo grabberLift, grabberGrab, verticalSlideGrab;

    // Autonomous to TeleOp:
    private static int parking;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public V2_5Drive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide");
        verticalSlide = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        grabberLift = hardwareMap.get(Servo.class, "grabberLift");
        grabberGrab = hardwareMap.get(Servo.class, "grabberGrab");
        verticalSlideGrab = hardwareMap.get(Servo.class, "verticalSlideGrab");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }


    // Peripherals
    public void resetEncoding(DcMotorEx generic) {
        generic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        generic.setTargetPosition(0);
        generic.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHorizontalSlide(String type, boolean reset) {
        if (reset) {
            resetEncoding(horizontalSlide);
        }

        try {
            // Stack Cones
            // TODO: Change to grid values...
            switch (type) {
                case "leftFromLeft":
                    horizontalSlide.setTargetPosition(-2000); //TODO

                    break;
                case "leftFromRight":
                    horizontalSlide.setTargetPosition(10); // TODO

                    break;
                case "rightFromLeft":
                    horizontalSlide.setTargetPosition(10); // TODO

                    break;
                case "rightFromRight":
                    horizontalSlide.setTargetPosition(10); //TODO

                    break;
                case "move100":
                    horizontalSlide.setTargetPosition(-100);
                    break;

                case "move-100":
                    horizontalSlide.setTargetPosition(100);
                    break;

                // Human Cones
                case "middleFromLeft":
                    horizontalSlide.setTargetPosition(10); // TODO
                    break;

                case "middleFromRight":
                    horizontalSlide.setTargetPosition(10); // TODO
                    break;

                case "Zero":
                    horizontalSlide.setTargetPosition(0);
                    break;
                case "Base":
                    horizontalSlide.setTargetPosition(10);
                    break;
                case "Passing":
                    horizontalSlide.setTargetPosition(10);
                    break;
                default:
                    throw new Exception("The 'type' was not found");
            }


        } catch (Exception ignored) {
        }

        horizontalSlide.setVelocity(2000);
    }


    public void setVerticalSlide(String type, boolean reset) {
        if (reset) {
            resetEncoding(verticalSlide);
        }

        try {
            switch (type) {
                case "highJunction":
                    verticalSlide.setTargetPosition(-4000);
                    break;
                case "mediumJunction":
                    verticalSlide.setTargetPosition(-3000);
                    break;
                case "lowJunction":
                    verticalSlide.setTargetPosition(-2000);
                    break;
                case "zero":
                    verticalSlide.setTargetPosition(0);
                    break;
                case "base":
                    verticalSlide.setTargetPosition(-10);
                    break;
                case "passing":
                    verticalSlide.setTargetPosition(-500);
                    break;
                default:
                    throw new Exception("The 'type' was not found");
            }


        } catch (Exception ignored) {
        }

        verticalSlide.setVelocity(1500);
    }


    public void setGrabber(String type) {

        try {
            switch (type) {
                case "topStack":
                    grabberLift.setPosition(10);
                    break;
                case "2ndStack":
                    grabberLift.setPosition(10);
                    break;
                case "3rdStack":
                    grabberLift.setPosition(10);
                    break;
                case "4thStack":
                    grabberLift.setPosition(10);
                    break;
                case "5thStack":
                    grabberLift.setPosition(10);
                    break;
                case "Zero":
                    grabberLift.setPosition(0);
                    break;
                case "base":
                    grabberGrab.setPosition(10);
                    grabberLift.setPosition(10);
                    break;
                case "passing":
                    grabberLift.setPosition(0.65);
                    break;
                case "wait":
                    grabberLift.setPosition(0.25);
                    break;

                case "grab":
                    grabberGrab.setPosition(0);
                    break;
                case "release":
                    grabberGrab.setPosition(0.25);
                    break;
                default:
                    throw new Exception("The 'type' was not found");
            }


        } catch (Exception ignored) {
        }
    }

    public void setVerticalSlideGrabber(String type) {

        try {
            switch (type) {
                case "highJunction":
                    verticalSlideGrab.setPosition(0.25);
                case "mediumJunction":
                    verticalSlideGrab.setPosition(0.25);
                    break;
                case "lowJunction":
                    verticalSlideGrab.setPosition(0.35);
                    break;
                case "zero":
                case "base":
                    verticalSlideGrab.setPosition(0);
                    break;

                //not done
                case "passing":
                    verticalSlideGrab.setPosition(0.85);
                    break;
                case "horizontal":
                    verticalSlideGrab.setPosition(0.5);
                    break;
                default:
                    throw new Exception("The 'type' was not found");
            }


        } catch (Exception ignored) {
        }
    }


    public void stackLoop() throws InterruptedException {
        sleep(1000);
        this.setVerticalSlide("highJunction", false);
        this.setHorizontalSlide("leftFromLeft", false);

        sleep(1500);
        this.setVerticalSlideGrabber("horizontal");

        sleep(1000);
        this.setVerticalSlideGrabber("highJunction");

        sleep(1000);
        this.setVerticalSlideGrabber("passing");

        sleep(500);
        this.setVerticalSlide("passing", false);

        sleep(1000);
        this.setGrabber("topStack");

        sleep(1000);
        this.setGrabber("grab");
        this.setHorizontalSlide("zero", false);

        sleep(1000);
        this.setGrabber("passing");

        sleep(1000);
        this.setGrabber("release");

                    /*
            .addTemporalMarker(7.0, () -> {
            })
            .addTemporalMarker(8.0, () -> {
                drive.setVerticalSlideGrabber("highJunction");
            })
            .addTemporalMarker(1.0, 0.5, () -> {
                drive.setGrabber("topStack");
                drive.setVerticalSlideGrabber("Passing");
                drive.setVerticalSlide("Passing", false);
            })
            .addTemporalMarker(1.0, 1, () -> {
                drive.setGrabber("grab");
            })
            .addTemporalMarker(1.0, 2.5, () -> {
                drive.setGrabber("topStack");
            })
            */
    }

    public void setHorizontalSlideDistance(int distance, boolean reset) {
        if (reset) {
            resetEncoding(horizontalSlide);
        }

        horizontalSlide.setTargetPosition(distance);

    }


    // Passing values between opmodes
    public static void changeParking(int i) {
        parking = i;
    }

    public static int getParking() {
        return parking;
    }
}ve.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class V2_5Drive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    // Used when strafing is undershooting / overshooting the target...
    public static double LATERAL_MULTIPLIER = 40/36.5;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    // Peripherals
    public DcMotorEx horizontalSlide, verticalSlide;
    public Servo grabberLift, grabberGrab, verticalSlideGrab;

    // Autonomous to TeleOp:
    private static int parking;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public V2_5Drive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide");
        verticalSlide = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        //grabberLift = hardwareMap.get(Servo.class, "grabberLift");
        grabberGrab = hardwareMap.get(Servo.class, "grabberGrab");
        verticalSlideGrab = hardwareMap.get(Servo.class, "verticalSlideGrab");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }




    // Peripherals
    public void resetEncoding(DcMotorEx generic) {
        generic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        generic.setTargetPosition(0);
        generic.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHorizontalSlide(String type, boolean reset) {
        if (reset) {
            resetEncoding(horizontalSlide);
        }

        try {
            // Stack Cones
            // TODO: Change to grid values...
            switch (type) {
                case "leftFromLeft":
                    horizontalSlide.setTargetPosition(-2000); //TODO

                    break;
                case "leftFromRight":
                    horizontalSlide.setTargetPosition(10); // TODO

                    break;
                case "rightFromLeft":
                    horizontalSlide.setTargetPosition(10); // TODO

                    break;
                case "rightFromRight":
                    horizontalSlide.setTargetPosition(10); //TODO

                    break;
                case "move100":
                    horizontalSlide.setTargetPosition(-50);
                    break;

                case "move-100":
                    horizontalSlide.setTargetPosition(50);
                    break;

                // Human Cones
                case "middleFromLeft":
                    horizontalSlide.setTargetPosition(10); // TODO
                    break;

                case "middleFromRight":
                    horizontalSlide.setTargetPosition(10); // TODO
                    break;

                case "Zero":
                    horizontalSlide.setTargetPosition(0);
                    break;
                case "Base":
                    horizontalSlide.setTargetPosition(10);
                    break;
                case "Passing":
                    horizontalSlide.setTargetPosition(10);
                    break;
                default:
                    throw new Exception("The 'type' was not found");
            }


        } catch (Exception ignored) {}

        horizontalSlide.setVelocity(500);
    }


    public void setVerticalSlide(String type, boolean reset) {
        if (reset) {
            resetEncoding(verticalSlide);
        }

        try {
            switch (type) {
                case "highJunction":
                    verticalSlide.setTargetPosition(-4000);
                    break;
                case "mediumJunction":
                    verticalSlide.setTargetPosition(-3000);
                    break;
                case "lowJunction":
                    verticalSlide.setTargetPosition(-2000);
                    break;
                case "zero":
                    verticalSlide.setTargetPosition(0);
                    break;
                case "base":
                    verticalSlide.setTargetPosition(-10);
                    break;
                case "passing":
                    verticalSlide.setTargetPosition(-500);
                    break;
                default:
                    throw new Exception("The 'type' was not found");
            }


        } catch (Exception ignored) {}

        verticalSlide.setVelocity(1500);
    }


    public void setGrabber(String type) {

        try {
            switch (type) {
                case "topStack":
                    grabberLift.setPosition(10);
                    break;
                case "2ndStack":
                    grabberLift.setPosition(10);
                    break;
                case "3rdStack":
                    grabberLift.setPosition(10);
                    break;
                case "4thStack":
                    grabberLift.setPosition(10);
                    break;
                case "5thStack":
                    grabberLift.setPosition(10);
                    break;
                case "Zero":

                    grabberLift.setPosition(0);
                    break;
                case "base":
                    grabberGrab.setPosition(10);
                    grabberLift.setPosition(10);
                    break;
                case "passing":
                    grabberLift.setPosition(0.8);
                    break;
                case "wait":
                    grabberLift.setPosition(0.25);
                    break;

                case "grab":
                    grabberGrab.setPosition(0);
                    break;
                case "release":
                    grabberGrab.setPosition(0.25);
                    break;
                default:
                    throw new Exception("The 'type' was not found");
            }


        } catch (Exception ignored) {}
    }

    public void setVerticalSlideGrabber(String type) {

        try {
            switch (type) {
                case "highJunction":
                    verticalSlideGrab.setPosition(0.25);
                case "mediumJunction":
                    verticalSlideGrab.setPosition(0.25);
                    break;
                case "lowJunction":
                    verticalSlideGrab.setPosition(0.35);
                    break;
                case "zero":
                case "base":
                    verticalSlideGrab.setPosition(0);
                    break;

                //not done
                case "passing":
                    verticalSlideGrab.setPosition(0.85);
                    break;
                case "horizontal":
                    verticalSlideGrab.setPosition(0.5);
                    break;
                default:
                    throw new Exception("The 'type' was not found");
            }


        } catch (Exception ignored) {}
    }


    public void stackLoop() throws InterruptedException {
        sleep(1000);
        this.setVerticalSlide("highJunction", false);
        this.setHorizontalSlide("leftFromLeft", false);

        sleep(1500);
        this.setVerticalSlideGrabber("horizontal");

        sleep(1000);
        this.setVerticalSlideGrabber("highJunction");

        sleep(1000);
        this.setVerticalSlideGrabber("passing");

        sleep(500);
        this.setVerticalSlide("passing", false);

        sleep(1000);
        this.setGrabber("topStack");

        sleep(1000);
        this.setGrabber("grab");
        this.setHorizontalSlide("zero", false);

        sleep(1000);
        this.setGrabber("passing");

        sleep(1000);
        this.setGrabber("release");

                    /*
            .addTemporalMarker(7.0, () -> {
            })
            .addTemporalMarker(8.0, () -> {
                drive.setVerticalSlideGrabber("highJunction");
            })
            .addTemporalMarker(1.0, 0.5, () -> {
                drive.setGrabber("topStack");

                drive.setVerticalSlideGrabber("Passing");
                drive.setVerticalSlide("Passing", false);
            })
            .addTemporalMarker(1.0, 1, () -> {
                drive.setGrabber("grab");
            })
            .addTemporalMarker(1.0, 2.5, () -> {
                drive.setGrabber("topStack");
            })
            */
    }

    public void setHorizontalSlideDistance(int distance, boolean reset){
    if (reset) {
        resetEncoding(horizontalSlide);
    }
    
    horizontalSlide.setTargetPosition(distance);

    }


    // Passing values between opmodes
    public static void changeParking(int i) {
        parking = i;
    }

    public static int getParking() {
        return parking;
    }
}
