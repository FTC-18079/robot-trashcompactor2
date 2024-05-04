package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRun {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12.0, -61.34, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(40, -0, 0), Math.PI / 2.0)
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(40, -40), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60, -58, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToConstantHeading(new Vector2d(35, -58))
                .splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(180)), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}