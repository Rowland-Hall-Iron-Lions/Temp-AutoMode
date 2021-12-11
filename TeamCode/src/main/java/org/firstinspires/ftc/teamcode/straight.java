package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class straight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory straight = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(10)
                .build();

        waitForStart();
        while(opModeIsActive()){
            drive.followTrajectory(straight);
        }
    }
}
