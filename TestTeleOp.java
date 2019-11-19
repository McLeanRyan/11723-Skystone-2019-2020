package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestTeleOp extends OpMode {
   DcMotor RF = null, RB = null, LF = null, LB = null, FI = null;

    @Override
    public void init() {
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
    }

    @Override
    public void loop() {
        LF.setPower(-gamepad1.left_stick_y);
        LB.setPower(-gamepad1.left_stick_y);
        RF.setPower(-gamepad1.right_stick_y);
        RB.setPower(-gamepad1.right_stick_y);

        while (gamepad1.left_trigger > 0) {
            LF.setPower(-1);
            LB.setPower(1);
            RF.setPower(1);
            RB.setPower(-1);
        }

        while (gamepad1.right_trigger > 0) {
            LF.setPower(1);
            LB.setPower(-1);
            RF.setPower(-1);
            RB.setPower(1);
        }

        if (gamepad2.a) {
            ElapsedTime elapsedTime = new ElapsedTime();
            FI.setPower(.5);
            elapsedTime.reset();

            while (elapsedTime.seconds() < 0.2) {
            }
            FI.setPower(0);
        }

        if (gamepad2.b) {
                ElapsedTime elapsedTime = new ElapsedTime();
                FI.setPower(-0.5);
                elapsedTime.reset();

                while (elapsedTime.seconds() < 0.2) {
           }
            FI.setPower(0);
        }
    }
}