/*
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous (name = "limelightautotest")
public class limelightautotest extends LinearOpMode{
    GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    boolean purpleFound;
    boolean greenFound;
    boolean objectFound;

    public void runOpMode() throws InterruptedException{
        final Pose2d readMotif = new Pose2d(-64, 40, Math.toRadians(90));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        purpleFound = false;
        greenFound = true;
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        double headingError = 0;
        double rangeError = 0;
        double rx;
        while (!isStarted()) {
            pinpoint.update();
            Pose2D pinpointPose = pinpoint.getPosition();

            telemetry.addData("Pinpoint X (IN)", pinpointPose.getX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Y (IN)", pinpointPose.getY(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Heading (DEG)", pinpointPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Purple Found? ", purpleFound);
            telemetry.addData("Green Found?", greenFound);
    }
}
*/