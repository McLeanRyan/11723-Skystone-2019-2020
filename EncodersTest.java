package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class EncodersTest extends LinearOpMode {

    DcMotor RF,RB, LF, LB, FI;

    private static double ticksPerMotorRev = 383.6;
    private static double driveGearReduction = 2;
    private static double wheelDiameterInches = 4;
    private static double ticksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Mode", "Initialized");
        telemetry.update();

        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        FI = hardwareMap.dcMotor.get("FI");

        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        FI.setPower(0);

        waitForStart();

        while (opModeIsActive()) {
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LF.setTargetPosition((int) ticksPerInch * 12);
            LB.setTargetPosition((int) ticksPerInch * 12);
            RF.setTargetPosition((int) ticksPerInch * 12);
            RB.setTargetPosition((int) ticksPerInch * 12);

            LF.setPower(0.5);
            LB.setPower(0.5);
            RF.setPower(0.5);
            RB.setPower(0.5);

            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
