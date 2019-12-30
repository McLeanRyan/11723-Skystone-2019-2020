package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestTeleOp extends OpMode {

    DcMotor RF, LF, LB, RB, FI, LIFT;
    CRServo ARM2, BOOM;

    @Override
    public void init() {
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        FI = hardwareMap.dcMotor.get("FI");
        ARM2 = hardwareMap.crservo.get("ARM2");
        LIFT = hardwareMap.dcMotor.get("LIFT");
        BOOM = hardwareMap.crservo.get("BOOM");

        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

        FI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        FI.setPower(0);
        LIFT.setPower(0);

        ARM2.setPower(0);
        BOOM.setPower(0);
    }

    @Override
    public void loop() {
        LF.setPower(-gamepad1.left_stick_y);
        LB.setPower(-gamepad1.left_stick_y);
        RF.setPower(-gamepad1.right_stick_y);
        RB.setPower(-gamepad1.right_stick_y);

        ARM2.setPower(gamepad2.right_stick_y);
        BOOM.setPower(gamepad2.left_stick_y);

        if (gamepad1.left_trigger > 0) {
            LF.setPower(-1);
            LB.setPower(1);
            RF.setPower(-1);
            RB.setPower(1);
        }

        if (gamepad1.right_trigger > 0) {
            LF.setPower(1);
            LB.setPower(-1);
            RF.setPower(1);
            RB.setPower(-1);
        }

        if (gamepad2.b) {
            ElapsedTime elapsedTime = new ElapsedTime();
            FI.setPower(.5);
            elapsedTime.reset();

            while (elapsedTime.seconds() < 2) {
            }
            FI.setPower(0);
        }

        if (gamepad2.a) {
            ElapsedTime elapsedTime = new ElapsedTime();
            FI.setPower(-0.5);
            elapsedTime.reset();

            while (elapsedTime.seconds() < 2) {
            }
            FI.setPower(0);
        }

        if (gamepad2.right_stick_button) {
            LIFT.setPower(0);
        }

        if (gamepad2.left_trigger > 0) {
            LIFT.setPower(-0.5);
        }
        else LIFT.setPower(0);

        if (gamepad2.right_trigger > 0) {
            LIFT.setPower(0.5);
        }
        else LIFT.setPower(0);
    }
}