package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class IntakeEncoderTest extends OpMode {
    boolean a_past = false;
    boolean b_past = false;
    DcMotor FI;

    double ticksPerMotorRev = 383.6;
    double driveGearReduction = 0.5;
    double wheelDiameterInches = 4;
    double ticksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    @Override
    public void init() {
        FI = hardwareMap.dcMotor.get("FI");
        FI.setDirection(DcMotorSimple.Direction.REVERSE);
        FI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FI.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("FI Starting Pos", FI.getCurrentPosition());
    }

    @Override
    public void loop() {
        telemetry.addLine("Running");
        telemetry.addData("FI Current Pos", FI.getCurrentPosition());
        telemetry.update();
        FI.setPower(gamepad1.right_stick_y / 3);
        if (gamepad1.a && !a_past) {
            telemetry.addLine("Pulling");
            telemetry.update();
            FIEncoder(true);
            a_past = true;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!gamepad1.a) {
            a_past = false;
        }
        if (gamepad1.b && !b_past) {
            telemetry.addLine("Pushing");
            telemetry.update();
            FIEncoder(false);
            b_past = true;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if (!gamepad1.b) {
            b_past = false;
        }
    }


    public void FIEncoder(boolean pull) {
        for (int i = 0; i < 4; i++) {
            int newFITarget = 0;
            int fIPos = FI.getCurrentPosition();
            if (pull) {
                newFITarget = fIPos + 383;
                telemetry.addLine("Pulling");
                telemetry.update();
            } else if (!pull) {
                newFITarget = fIPos - 383;
                telemetry.addLine("Pushing");
                telemetry.update();
            }
            FI.setTargetPosition(newFITarget);
            FI.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FI.setPower(Math.abs(.5));
            while (FI.isBusy()) {
                telemetry.addData("Current Pos", FI.getCurrentPosition());
                telemetry.addData("Target", newFITarget);
                telemetry.addData("Power", FI.getPower());
                telemetry.update();
            }
            FI.setPower(0);
            FI.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}


