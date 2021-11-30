package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutoModeBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeL = null;
    private DcMotor intakeR = null;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private Servo clawServo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SampleMecanumDrive bc1 = new SampleMecanumDrive(hardwareMap);
        SampleMecanumDrive bc2 = new SampleMecanumDrive(hardwareMap);
        SampleMecanumDrive bc3 = new SampleMecanumDrive(hardwareMap);
        
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR= hardwareMap.get(DcMotor.class, "intakeR");
        // Define trajectories to be used for most of the match
        // toDuckSpinner = go to the turntable to get duck(s)
        Trajectory toDuckSpinner = drive.trajectoryBuilder(new Pose2d(-35, 60, 0))
                .lineToSplineHeading(new Pose2d(-60, 60, 0))
                .build();
        // park = go to the warehouse to park at end of autonomous period
        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(100)
                .build();
        // bc1 = drive to middle barcode spot, to check for duck
        Trajectory barcode1 = bc1.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(15)
                .build();
        // bc2 = drive to first barcode spot, to check for duck
        Trajectory barcode2 = bc2.trajectoryBuilder(drive.getPoseEstimate())
                .back(9)
                .build();
        // bc3 = drive to third barcode spot, to check for duck
        Trajectory barcode3 = bc3.trajectoryBuilder(drive.getPoseEstimate())
                .forward(17)
                .build();
        // forwardForPickup = This trajectory is run when the robot needs to drive forward to pickup the barcode duck.
        Trajectory forwardForPickup = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(10)
                .build();
        Trajectory pickupStartBox = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-60, 60, Math.toRadians(270)))
                .build();

//        intakeMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeMotorR.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Status", "Running");
            telemetry.update();
            // Check Barcode, for which layer of alliance shipping hub
            bc1.followTrajectory(barcode1);
            sleep(1000);
            bc2.followTrajectory(barcode2);
            sleep(1000);
            bc3.followTrajectory(barcode3);
            sleep(1000);
            // While the robot is paused, check if duck sits on that spot.
            while (!bc1.isBusy() && !bc2.isBusy() && !bc3.isBusy()){
                double distance = distanceSensor.getDistance(DistanceUnit.CM);
                // If duck is there, run pickup code.
                if (distance <= 4){
                    drive.followTrajectory(pickupStartBox);
                    sleep(100);
//                    turntableWheel.setPower(5);
                    sleep(100);
//                    turntableWheel.setPower(0);
                    intakeL.setPower(5);
                    intakeR.setPower(-5);
                    drive.followTrajectory(forwardForPickup);
                }
            }
        }
    }
}
