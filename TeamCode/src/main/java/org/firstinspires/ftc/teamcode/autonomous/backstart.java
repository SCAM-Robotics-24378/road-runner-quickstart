package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Autonomous(name = "backstart")
public class backstart extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private DcMotorEx flywheel;
    private CRServo Gecko1;
    private CRServo Gecko2;

    @Override
    public void runOpMode() throws InterruptedException {
        final Pose2d init_close = new Pose2d(-64, -40, Math.toRadians(-90));
        final Pose2d blue_far_init = new Pose2d(64, -16, Math.toRadians(180));
        final Pose2d blue_far_launch = new Pose2d(55, -15, Math.toRadians(-156));
        final Pose2d blue_far_park = new Pose2d(48, -25, Math.toRadians(180));
        final double flywheelVel_far = -1470;
        double waitForLaunchTime = 1;
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
                    headingError = result.getTx();
                    rangeError = result.getBotposeAvgDist();
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("AvgDist", rangeError);
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                } else {
                    telemetry.addData("ll", "is invalid");
                    headingError = 0;
                    rangeError = 0;
                }
            } else {
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
        //1
        Action waitForFlywheelSpinup1 = drive.actionBuilder(blue_far_init)
                .waitSeconds(4)
                .build();
        Action waitForLaunch1 = drive.actionBuilder(blue_far_launch)
                .waitSeconds(0.35)
                .build();
        Action waitForLaunchRecovery1 = drive.actionBuilder(blue_far_launch)
                .waitSeconds(0.45)
                .build();

        Action startGeckoWheels1 = new InstantAction( () -> {
            Gecko1.setPower(1);
            Gecko2.setPower(-1);
        });
        Action stopGeckoWheels1 = new InstantAction( () -> {
            Gecko1.setPower(0);
            Gecko2.setPower(0);
        });
        //2
        Action waitForFlywheelSpinup2 = drive.actionBuilder(blue_far_init)
                .waitSeconds(3)
                .build();
        Action waitForLaunch2 = drive.actionBuilder(blue_far_launch)
                .waitSeconds(0.35)
                .build();
        Action waitForLaunchRecovery2 = drive.actionBuilder(blue_far_launch)
                .waitSeconds(0.45)
                .build();

        Action startGeckoWheels2 = new InstantAction( () -> {
            Gecko1.setPower(1);
            Gecko2.setPower(-1);
        });
        Action stopGeckoWheels2 = new InstantAction( () -> {
            Gecko1.setPower(0);
            Gecko2.setPower(0);
        });
        //3
        Action waitForFlywheelSpinup3 = drive.actionBuilder(blue_far_init)
                .waitSeconds(3)
                .build();
        Action waitForLaunch3 = drive.actionBuilder(blue_far_launch)
                .waitSeconds(1)
                .build();
        Action waitForLaunchRecovery3 = drive.actionBuilder(blue_far_launch)
                .waitSeconds(0.1)
                .build();

        Action startGeckoWheels3 = new InstantAction( () -> {
            Gecko1.setPower(1);
            Gecko2.setPower(-1);
        });
        Action stopGeckoWheels3 = new InstantAction( () -> {
            Gecko1.setPower(0);
            Gecko2.setPower(0);
        });

        //TODO this sequence is a collection of others. You can use the individual actions elsewhere if needed
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
                                //1
                                waitForFlywheelSpinup1,
                                startGeckoWheels1,
                                waitForLaunch1,
                                stopGeckoWheels1,
                                waitForLaunchRecovery1,

                                //2
                                waitForFlywheelSpinup2,
                                startGeckoWheels2,
                                waitForLaunch2,
                                stopGeckoWheels2,
                                waitForLaunchRecovery2,
                                //3
                                waitForFlywheelSpinup3,
                                startGeckoWheels3,
                                waitForLaunch3,
                                stopGeckoWheels3,
                                waitForLaunchRecovery3,

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
