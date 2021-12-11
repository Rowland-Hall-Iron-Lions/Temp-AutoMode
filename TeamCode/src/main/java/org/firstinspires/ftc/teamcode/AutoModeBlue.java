package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutoModeBlue extends LinearOpMode {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotor duckWheel = hardwareMap.get(DcMotor.class, "duckWheel");
        DcMotor extender = hardwareMap.get(DcMotor.class, "extender");
        CRServo intakeL = hardwareMap.get(CRServo.class, "intakeL");
        CRServo intakeR = hardwareMap.get(CRServo.class, "intakeR");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Servo intakeLiftR = hardwareMap.get(Servo.class, "intakeLiftR");
        Servo intakeLiftL = hardwareMap.get(Servo.class, "intakeLiftL");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Define trajectories to be used for most of the match
        // toDuckSpinner = go to the turntable to get duck(s)
        Trajectory toDuckSpinner = drive.trajectoryBuilder(new Pose2d(-35, 60, 0))
                .lineToSplineHeading(new Pose2d(-60, 60, 0))
                .build();
        // park = go to the warehouse to park at end of autonomous period
        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(100)
                .build();
        // toShippingHub = This trajectory is run when we need to get to the shipping hub to place a piece of fright
        Trajectory toShippingHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-11, 43, Math.toRadians(-90)))
                .build();

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Status", "Running");
            telemetry.update();
            drive.followTrajectory(toDuckSpinner);
            duckWheel.setPower(15);
            sleep(6000);
            duckWheel.setPower(0);
            drive.followTrajectory(toShippingHub);
            extender.setPower(3);
            sleep(100);
            extender.setPower(0);
            clawServo.setPosition(10);
            sleep(1500);
            extender.setPower(-3);
            sleep(500);
            extender.setPower(0);

        }
    }
}
