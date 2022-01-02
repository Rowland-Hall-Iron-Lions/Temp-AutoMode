package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class autoRedLeft extends LinearOpMode {
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

        // Declare Trajectories
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Run while the Autonomous mode is active
        waitForStart();
        while (opModeIsActive()) {
            // Update telemetry status to show that it is Running
            telemetry.addData("Status", "Running");
            telemetry.update();

            //Stop the Autonomous mode after we finish parking
            return;
        }
    }
}
