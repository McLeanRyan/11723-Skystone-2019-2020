package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;

@Autonomous
public class AdvancedPark extends LinearOpMode {
    DcMotor LF, LB, RF, RB, LIFT, FI;
    Servo ARM2;
    CRServo BOOM;

    double ticksPerMotorRev = 383.6;
    double driveGearReduction = 0.5;
    double wheelDiameterInches = 4;
    double ticksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    boolean far = false;
    double distance = 3;
    int delay = 0;
    boolean left = true;

    boolean a = false;
    int Where =0;
    BNO055IMU imu;
    private Orientation angles;

    double kP = 0.005;
    double kD = 0.01;
    double kI = 0.00008;

    double totalError = 0;
    double lastAngle = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode", "Initialized");
        telemetry.update();

        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        FI = hardwareMap.dcMotor.get("FI");
        LIFT = hardwareMap.dcMotor.get("LIFT");

        ARM2 = hardwareMap.servo.get("ARM2");
        BOOM = hardwareMap.crservo.get("BOOM");

        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        FI.setPower(0);
        LIFT.setPower(0);

        BOOM.setPower(0);

        telemetry.addData("LF Starting Pos", LF.getCurrentPosition());
        telemetry.addData("LB Starting Pos", LB.getCurrentPosition());
        telemetry.addData("RF Starting Pos", RF.getCurrentPosition());
        telemetry.addData("RB Starting Pos", RB.getCurrentPosition());
        telemetry.update();

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();       //sets up IMU
        parameters1.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "REVHub1IMUCalibration.json";
        parameters1.loggingEnabled      = true;
        parameters1.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addLine("ready to go!");
        telemetry.update();


        while (!a) {
            if (gamepad1.dpad_left) {
                --distance;
            }
            if (gamepad1.dpad_right) {
                ++distance;
            }
            if (gamepad1.dpad_up) {
                far = true;
            }
            if (gamepad1.dpad_down) {
                far = false;
            }
            if (gamepad1.right_bumper) {
                ++delay;
            }
            if (gamepad1.left_bumper) {
                --delay;
            }
            if (gamepad1.a) {
                a = true;
            }
            if (gamepad1.b) {
                left = false;
            }
            telemetry.addData("Going to far side?", far);
            telemetry.addData("Distance", distance);
            telemetry.addData("Delay", delay);
            telemetry.addData("Starting on left side?", left);
            telemetry.addData("LF Starting Pos", LF.getCurrentPosition());
            telemetry.addData("LB Starting Pos", LB.getCurrentPosition());
            telemetry.addData("RF Starting Pos", RF.getCurrentPosition());
            telemetry.addData("RB Starting Pos", RB.getCurrentPosition());
            telemetry.addLine("D Pad Left/Right for Distance");
            telemetry.addLine("D Pad Up/Down for Far");
            telemetry.addLine("Bumpers for delay");
            telemetry.addLine("B for left");
            telemetry.addLine("A to finish and move on");
            telemetry.update();
        }

        waitForStart();
        if (distance < 0) {
            distance = 0;
        }
        if (!left) {
            distance = -distance;
        }
        sleep(delay);
        if (far) {
            encoderDrive(.25, 36, 31, true);
            sleep(500);
        }
        encoderDrive(.25, distance, 31, false);
    }

    private void encoderDrive(double speed, double inches, double timeoutS, boolean strafe) {

        telemetry.addLine("Encoder Drive");

        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
        int lFPos = LF.getCurrentPosition();
        int rFPos = RF.getCurrentPosition();
        int lBPos = LB.getCurrentPosition();
        int rBPos = RB.getCurrentPosition();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;

        if (opModeIsActive()) {
            if (strafe) {
                newLFTarget = lFPos + (int) (inches * ticksPerInch);
                newRFTarget = rFPos - (int) (inches * ticksPerInch);
                newLBTarget = lBPos - (int) (inches * ticksPerInch);
                newRBTarget = rBPos + (int) (inches * ticksPerInch);
            } else {
                newLFTarget = lFPos + (int) (inches * ticksPerInch);
                newRFTarget = rFPos + (int) (inches * ticksPerInch);
                newLBTarget = lBPos + (int) (inches * ticksPerInch);
                newRBTarget = rBPos + (int) (inches * ticksPerInch);
            }

            telemetry.addData("speed", speed);
            telemetry.addData("inches", inches);
            telemetry.addData("newLFTarget", newLFTarget);
            telemetry.addData("newRFTarget", newRFTarget);
            telemetry.addData("newLBTarget", newLBTarget);
            telemetry.addData("newRBTarget", newRBTarget);

            LF.setTargetPosition(newLFTarget);
            RF.setTargetPosition(newRFTarget);
            LB.setTargetPosition(newLBTarget);
            RB.setTargetPosition(newRBTarget);

            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LF.setPower(Math.abs(speed));
            RF.setPower(Math.abs(speed));
            LB.setPower(Math.abs(speed));
            RB.setPower(Math.abs(speed));

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < timeoutS && (LF.isBusy() && RF.isBusy() && LB.isBusy() && RB.isBusy())) {
                if (!strafe) {
                    double error = kP * (startAngle - angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
                    telemetry.addData("Start Angle", startAngle);
                    telemetry.addData("Current Angle", angles.firstAngle);
                    telemetry.addData("error", error);
                    telemetry.update();
                    if (error > 0) {
                        LF.setPower(Math.abs(speed) - error);
                        RF.setPower(Math.abs(speed) + error);
                        LB.setPower(Math.abs(speed) - error);
                        RB.setPower(Math.abs(speed) + error);
                    } else if (error < 0) {
                        LF.setPower(Math.abs(speed) + error);
                        RF.setPower(Math.abs(speed) - error);
                        LB.setPower(Math.abs(speed) + error);
                        RB.setPower(Math.abs(speed) - error);
                    }
                }
                telemetry.addData("LF Current Position", LF.getCurrentPosition());
                telemetry.addData("RF Current Position", RF.getCurrentPosition());
                telemetry.addData("LB Current Position", LB.getCurrentPosition());
                telemetry.addData("RB Current Position", RB.getCurrentPosition());
                telemetry.addData("LF Current Power", LF.getPower());
                telemetry.addData("RF Current Power", RF.getPower());
                telemetry.addData("LB Current Power", LB.getPower());
                telemetry.addData("RB Current Power", RB.getPower());
                telemetry.update();
            }
            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);

            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);
        }
    }

    private void gyroTurn(double targetAngle) {
        //+ is counter-clockwise
        //- is clockwise
        boolean finished = false;
        while (!finished) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
            double currentAngle = angles.firstAngle;
            double e = (targetAngle - currentAngle);
            totalError += e;
            double error = (kP * e) - (kD * (currentAngle - lastAngle)) + (kI) * (totalError);
            lastAngle = currentAngle;
            LF.setPower(-error);
            RF.setPower(error);
            LB.setPower(-error);
            RB.setPower(error);
            telemetry.addData("targetAngle", targetAngle);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.addData("error", error);
            telemetry.addData("targetAngle - currentAngle", targetAngle - currentAngle);
            telemetry.addData("finished", finished);



            telemetry.addData("LFM Current Power", LF.getPower());
            telemetry.addData("RFM Current Power", RF.getPower());
            telemetry.addData("LBM Current Power", LB.getPower());
            telemetry.addData("RBM Current Power", RB.getPower());

            telemetry.update();
            if (Math.abs(targetAngle - currentAngle) < 4) {
                finished = true;
                telemetry.addData("Finished", finished);
                RF.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                LB.setPower(0);
                sleep(500);
            }
        }
    }
}