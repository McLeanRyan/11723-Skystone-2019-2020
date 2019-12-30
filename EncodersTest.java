package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class EncodersTest extends LinearOpMode {

    DcMotor RF, RB, LF, LB = null;
    DcMotor FI, LIFT;
    CRServo ARM2, BOOM;

    double ticksPerMotorRev = 383.6;
    double driveGearReduction = 2;
    double wheelDiameterInches = 4;
    double ticksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    double driveSpeed = .2;

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

        ARM2 = hardwareMap.crservo.get("ARM2");
        BOOM = hardwareMap.crservo.get("BOOM");

        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.getCurrentPosition();
        LB.getCurrentPosition();
        RF.getCurrentPosition();
        RB.getCurrentPosition();

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        FI.setPower(0);
        LIFT.setPower(0);

        ARM2.setPower(0);
        BOOM.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addLine("RUN USING ENCODERS");
            telemetry.update();

            int LFStartingPosition = LF.getCurrentPosition();
            int LBStartingPosition = LB.getCurrentPosition();
            int RFStartingPosition = RF.getCurrentPosition();
            int RBStartingPosition = RB.getCurrentPosition();

            telemetry.addLine("Found current position");
            telemetry.update();

            LF.setPower(driveSpeed);
            LB.setPower(driveSpeed);
            RF.setPower(driveSpeed);
            RB.setPower(driveSpeed);

            LF.setTargetPosition((int)(12 * ticksPerInch) + LFStartingPosition);
            LB.setTargetPosition((int)(12 * ticksPerInch) + LBStartingPosition);
            RF.setTargetPosition((int)(12 * ticksPerInch) + RFStartingPosition);
            RB.setTargetPosition((int)(12 * ticksPerInch) + RBStartingPosition);

            telemetry.addLine("Set Target Position");
            telemetry.update();

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
