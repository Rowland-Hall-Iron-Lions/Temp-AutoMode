package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name="autoBlueLeft")

public class autoBlueLeft extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        DcMotor duckWheel = hardwareMap.get(DcMotor.class, "duckWheel");
        CRServo intakeL = hardwareMap.get(CRServo.class, "intakeL");
        CRServo intakeR = hardwareMap.get(CRServo.class, "intakeR");
        DcMotor extender = hardwareMap.get(DcMotor.class, "extender");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        // Run using encoders. This allows us to count how many rotations
        // each motor does and use that data accordingly
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Make it so that the motors brake when they have zero power
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Directions of Motors
        duckWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        //Declare Trajectories
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Trajectory to get to the shipping hub to drop our preload box
        Trajectory toShippingHub = drive.trajectoryBuilder(new Pose2d(12, 58, Math.toRadians(180)))
                .forward(15)
                .build();
        // Trajectory to get to the entrance for the warehouse(edge)
        Trajectory readyForPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(12, 62, Math.toRadians(0)))
                .build();
        // Trajectory to drive forward and park inside the warehouse
        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(15)
                .build();

        // Run While the Autonomous Mode is Active
        waitForStart();
        while (opModeIsActive()){
            // Update telemetry status to show that it is running
            telemetry.addData("Status", "Running");
            telemetry.update();

            // Follow trajectories. This is what the robot will actually do
            drive.followTrajectory(toShippingHub);
            // TODO: NEW INTAKE MOVEMENT
            drive.followTrajectory(readyForPark);
            drive.followTrajectory(park);

            // Stop the Autonomous Mode after we finish parking
            return;
        }
    }
}
