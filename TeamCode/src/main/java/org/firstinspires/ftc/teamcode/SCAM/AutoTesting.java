package org.firstinspires.ftc.teamcode.SCAM;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AutoTesting", group = "SCAM")

public class AutoTesting extends LinearOpMode {

    private Limelight3A limelight;


    @Override
    public void runOpMode() throws InterruptedException
    {
        final Pose2d init_close = new Pose2d(-64, -40, Math.toRadians(-90));
        final Pose2d init_far = new Pose2d(64, -16, Math.toRadians(180));
        final Pose2d init_test = new Pose2d(0, 0, Math.toRadians(0));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        MecanumDrive drive = new MecanumDrive(hardwareMap, init_test);
        waitForStart();
        Action moveTest = drive.actionBuilder(init_test)
                .strafeToLinearHeading(new Vector2d(24,0), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(24,24), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,0), Math.toRadians(180))
                .build();


        Actions.runBlocking(
                new SequentialAction(
                       // driveToFirstLaunch,
                        moveTest
                )
        );
        telemetry.addData("Test", "Complete");
        limelight.stop();




    }
}

