package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeEncoderTest extends OpMode {
    DcMotor FI;

    double ticksPerMotorRev = 383.6;
    double driveGearReduction = 0.5;
    double wheelDiameterInches = 4;
    double driveTicksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    @Override
    public void init() {
        FI = hardwareMap.dcMotor.get("FI");
        FI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FI.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("FI Starting Pos", FI.getCurrentPosition());
    }

    @Override
    public void loop() {
        telemetry.addLine ("Running");
        if (gamepad1.a) {
            telemetry.addLine ("pulling");
            FIEncoder(true);
        }
        if (gamepad1.b) {
            telemetry.addLine("pushing");
            FIEncoder(false);
        }
    }


    private void FIEncoder(boolean pull) {
        int FICurrentPos = FI.getCurrentPosition();
        FI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (pull) {
            FI.setTargetPosition(-383);
        } else {
            FI.setTargetPosition(383);
        }
        FI.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FI.setPower(.2);
        while (FI.isBusy()) {
            telemetry.addData("FI Current Position", FI.getCurrentPosition());
        }
        FI.setPower(0);
        FI.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


