package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // System.setProperty("sun.java2d.opengl", "true");
        MeepMeep mm = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, 0))
                                .lineToSplineHeading(new Pose2d(10, -60, 0))

                                .build()
                )
                .start();
    }
}
