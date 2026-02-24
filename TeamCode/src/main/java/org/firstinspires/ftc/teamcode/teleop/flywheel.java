package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "flywheel")
@Disabled
public class flywheel extends LinearOpMode {

    private DcMotorEx Flywheel;
    private DcMotor FrontRight;
    private DcMotor BackRight;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private CRServo Gecko1;
    private CRServo Gecko2;
    private Limelight3A limelight;


    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        limelight.start();

        double denominator;
        double y;
        double x;
        double rx;
        double speed = 0.5;
        boolean arm_down = false;
        double flywheelVel = 0;
        double drive;
        double turn;
        double strafe;

        final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
        final double TURN_GAIN   =  0.065  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 1;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 1;   //  Clip the strafing speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 1;   //  Clip the turn speed to this max value (adjust for your robot)



        Flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        FrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        BackRight = hardwareMap.get(DcMotor.class, "backRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "backLeft");
        Gecko1 = hardwareMap.get(CRServo.class, "Gecko1");
        Gecko2 = hardwareMap.get(CRServo.class, "Gecko2");


        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        Flywheel.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients customCoeffs = new PIDFCoefficients(50.0, 0.05, 2.5, 13.5);
        Flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customCoeffs);


        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.



                LLResult result = limelight.getLatestResult();


                boolean slow_bot;
                boolean reg_bot;
                boolean talha;
                boolean speed_bot;
                slow_bot = gamepad1.a;
                reg_bot = gamepad1.b;
                speed_bot = gamepad1.y;
                talha = gamepad1.x;
                y = gamepad1.left_stick_y;
                x = -(gamepad1.left_stick_x * 1.1);
                rx = -gamepad1.right_stick_x;
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
                double headingError = 0;
                double rangeError = 0;


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
                            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        }

                        // if (gamepad1.left_trigger > 0) {
                        //   y  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        // }
                        if (gamepad1.left_trigger > 0) {
                            rx   = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                        }
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

                //Changes speed modes based on controller 1 buttons
                if (slow_bot){
                    speed = 0.3;
                }
                if (reg_bot){
                    speed = 0.5;
                }
                if (speed_bot){
                    speed = 0.7;
                }
                if (talha){
                    speed = 1.0;
                }

                //Reports current speed constant

                telemetry.addData("speed value:", speed);
                telemetry.addData("Set Ticks:", flywheelVel);
                telemetry.addData("Ticks:", Flywheel.getVelocity());
                telemetry.addData("RPM:", (Flywheel.getVelocity()/28)*60);

                //double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                //Sets motor speed to motorspeed variables
                frontLeftPower = frontLeftPower*speed;
                frontRightPower = frontRightPower*speed;
                backLeftPower = backLeftPower*speed;
                backRightPower = backRightPower*speed;

                FrontLeft.setPower(frontLeftPower);
                BackLeft.setPower(backLeftPower);
                FrontRight.setPower(frontRightPower);
                BackRight.setPower(backRightPower);

                telemetry.update();

                // if (gamepad1.dpad_up) {
                //   flywheelVel = 1000;
                // }
                // else if (gamepad1.dpad_down) {
                //   flywheelVel = 0;
                // }

                if (gamepad1.leftBumperWasPressed()) {
                    flywheelVel -= 25;
                }

                if (gamepad1.rightBumperWasPressed()) {
                    flywheelVel += 25;
                }

                if (gamepad1.dpad_right) {
                    flywheelVel = 1625;
                }
                else if (gamepad1.dpad_left) {
                    flywheelVel = 1450;
                }

                if (gamepad1.right_trigger > 0) {
                    Gecko1.setPower(1);
                    Gecko2.setPower(-1);
                }
                if (gamepad1.right_trigger == 0) {
                    Gecko1.setPower(0);
                    Gecko2.setPower(0);
                }

                Flywheel.setVelocity(flywheelVel);
            }
        }
    }
}
