package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
            encoderDrive(.25, 36, true);
            sleep(500);
        }
        encoderDrive(.25, distance, false);
    }

    private void encoderDrive(double speed, double inches, boolean strafe) {

        telemetry.addLine("Encoder Drive");

        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
        int lFPos = LF.getCurrentPosition();
        int rFPos = RF.getCurrentPosition();
        int lBPos = LB.getCurrentPosition();
        int rBPos = RB.getCurrentPosition();

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
            telemetry.update();


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

            while (opModeIsActive() && (LF.isBusy() && RF.isBusy() && LB.isBusy() && RB.isBusy())) {
                telemetry.addData("newLFTarget", newLFTarget);
                telemetry.addData("newRFTarget", newRFTarget);
                telemetry.addData("newLBTarget", newLBTarget);
                telemetry.addData("newRBTarget", newRBTarget);
                telemetry.addData("LF Current Pos", LF.getCurrentPosition());
                telemetry.addData("RF Current Pos", RF.getCurrentPosition());
                telemetry.addData("LB Current Pos", LB.getCurrentPosition());
                telemetry.addData("RB Current Pos", RB.getCurrentPosition());
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

            sleep(10000);
        }
    }
}
