package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import android.app.Activity;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "RED_BUCKET_SIDE", group = "Autonomous")
public class REDBUCKET extends LinearOpMode {
    public class Lift {
        private DcMotorEx ylinear;

        public Lift(HardwareMap hardwareMap) {
            ylinear = hardwareMap.get(DcMotorEx.class, "ylinear");
            ylinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ylinear.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ylinear.setPower(0.8);
                    initialized = true;
                }

                double pos = ylinear.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    ylinear.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ylinear.setPower(-0.8);
                    initialized = true;
                }

                double pos = ylinear.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    ylinear.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

            TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 5))
                //.waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .lineToXSplineHeading(-30, Math.toRadians(220))
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(6, Math.toRadians(90))
                .strafeTo(new Vector2d(0, 18))
                .setTangent(Math.toRadians(90))
                .lineToYSplineHeading(34, Math.toRadians(180))
                .strafeTo(new Vector2d(1, 30))
                //.setTangent(Math.toRadians(180))

                //testing new are
                //.turn(Math.toRadians(90))
                //.setTangent(Math.toRadians(270))
                //.lineToYSplineHeading(0, Math.toRadians(180))


                //.setTangent(Math.toRadians(90))
                //.lineToYSplineHeading(30, Math.toRadians(90))
                .waitSeconds(30);

                /*.waitSeconds(0.3)
                .turn(Math.toRadians(-40))

                .strafeTo(new Vector2d(0, 0))
                //.waitSeconds(1)
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(0, 22))
                .turn(Math.toRadians(90))
                .setTangent(Math.toRadians(180))
                .lineToXSplineHeading(-35, Math.toRadians(215))*/
                //.waitSeconds(1)
                //.setTangent(Math.toRadians(90))
                //.lineToYSplineHeading(20, Math.toRadians(180))
                //.strafeTo(new Vector2d(20, 0))
                //.setTangent(Math.toRadians(90))
                //.lineToY(48)
                //.setTangent(Math.toRadians(0))
                //.lineToX(32)
                /*.strafeTo(new Vector2d(0, 100))
                .turn(Math.toRadians(180))*/
                //.lineToX(47.5)

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                /*.lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)*/
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        /*lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),*/
                        trajectoryActionCloseOut
                )
        );
    }
}