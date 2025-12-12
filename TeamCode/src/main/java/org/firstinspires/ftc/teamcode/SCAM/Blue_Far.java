package org.firstinspires.ftc.teamcode.SCAM;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous(name = "Blue_Far", group = "SCAM")

public class Blue_Far extends LinearOpMode {
    // Create an instance of the sensor
    GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private DcMotorEx flywheel;
    private CRServo Gecko1;
    private CRServo Gecko2;


    @Override
    public void runOpMode() throws InterruptedException
    {
        final Pose2d init_close = new Pose2d(-64, -40, Math.toRadians(-90));
        final Pose2d blue_far_init= new Pose2d(64, -16, Math.toRadians(180));
        final Pose2d blue_far_launch = new Pose2d(52,-15, Math.toRadians(-159));
        final Pose2d blue_far_park = new Pose2d(48,-25, Math.toRadians(180));
        final double flywheelVel_far= 1600;
        //final Pose2d init_test = new Pose2d(24, 0, Math.toRadians(0));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.resetPosAndIMU();

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients customCoeffs = new PIDFCoefficients(50.0, 0.05, 2.5, 13.5);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customCoeffs);
        Gecko1 = hardwareMap.get(CRServo.class, "Gecko1");
        Gecko2 = hardwareMap.get(CRServo.class, "Gecko2");

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        MecanumDrive drive = new MecanumDrive(hardwareMap, blue_far_init);

        //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, init_test.position.x, init_test.position.y, AngleUnit.RADIANS, init_test.heading.toDouble()));
        double headingError = 0;
        double rangeError = 0;
        double rx;
        while (!isStarted()) {
            pinpoint.update();
            Pose2D pinpointPose = pinpoint.getPosition();

            telemetry.addData("Pinpoint X (IN)", pinpointPose.getX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Y (IN)", pinpointPose.getY(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Heading (DEG)", pinpointPose.getHeading(AngleUnit.DEGREES));

            LLResult result = limelight.getLatestResult(); // get the latest result from the limelight
            if (result != null) {
                if (result.isValid()) {
                    headingError = result.getTx() + 3.4;
                    rangeError = result.getBotposeAvgDist();
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("AvgDist", rangeError);
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // if (gamepad1.left_trigger > 0) {
                    //   y  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    // }
//                    if (gamepad1.left_trigger > 0) {
//                        rx   = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//                    }
                }
                else {
                    telemetry.addData("ll", "is invalid");
                    headingError = 0;
                    rangeError = 0;
                }
            }
            else {
                telemetry.addData("ll", "null");
            }

            telemetry.update();
        }
        waitForStart();
        Action setFlywheelFar = new InstantAction( () -> {
            flywheel.setVelocity(flywheelVel_far);
        });
        Action stopFlywheel = new InstantAction( () -> {
            flywheel.setVelocity(0);
        });
        //TODO addjust times and locations as needed
        Action moveToFarLaunch = drive.actionBuilder(blue_far_init)
                .strafeToLinearHeading(blue_far_launch.position, blue_far_launch.heading.toDouble())
                .build();
        Action driveToBlueFarPark = drive.actionBuilder(blue_far_launch)
                .strafeToLinearHeading(blue_far_park.position, blue_far_park.heading.toDouble())
                .build();
        Action waitForFlywheelSpinup = drive.actionBuilder(blue_far_init)
                .waitSeconds(3)
                .build();
        Action waitForLaunch = drive.actionBuilder(blue_far_launch)
                .waitSeconds(1)
                .build();
        Action waitForLaunchRecovery = drive.actionBuilder(blue_far_launch)
                .waitSeconds(1)
                .build();
        Action startGeckoWheels = new InstantAction( () -> {
            Gecko1.setPower(1);
            Gecko2.setPower(-1);
        });
        Action stopGeckoWheels = new InstantAction( () -> {
            Gecko1.setPower(0);
            Gecko2.setPower(0);
        });
        //TODO this sequence is a collection of others. You can use the individual actions elsewhere if needed
        Action shootSequence = new SequentialAction(
                startGeckoWheels,
                waitForLaunch,
                stopGeckoWheels,
                waitForLaunchRecovery
        );
        pinpoint.update();
        Pose2D pinpointPose = pinpoint.getPosition();

        telemetry.addData("Pinpoint X (IN)", pinpointPose.getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y (IN)", pinpointPose.getY(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Heading (DEG)", pinpointPose.getHeading(AngleUnit.DEGREES));
        Actions.runBlocking(
                new ParallelAction(
                        //add inline action here for telemetry
                        new InstantAction( () -> {
                            pinpoint.update();
                            Pose2D pose = pinpoint.getPosition();

                            telemetry.addData("Pinpoint X (IN)", pose.getX(DistanceUnit.INCH));
                            telemetry.addData("Pinpoint Y (IN)", pose.getY(DistanceUnit.INCH));
                            telemetry.addData("Pinpoint Heading (DEG)", pose.getHeading(AngleUnit.DEGREES));
                            telemetry.update();
                        }),
                        new SequentialAction(
                                setFlywheelFar,
                                // driveToFirstLaunch,
                                moveToFarLaunch,
                                waitForFlywheelSpinup,
                                //First shot
                                shootSequence,
                                //Second shot
                                shootSequence,
                                //Third shot
                                shootSequence,
                                //park
                                stopFlywheel,
                                driveToBlueFarPark
                        )
                )

        );
        telemetry.addData("Test", "Complete");
        limelight.stop();
    }
}

