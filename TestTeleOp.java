package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestTeleOp extends OpMode {

    DcMotor RF = null, RB = null, LF = null, LB = null, FI = null;
    Servo ARM2;
    double servoPos;

    @Override
    public void init() {
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        FI = hardwareMap.dcMotor.get("FI");
        ARM2 = hardwareMap.servo.get("ARM");

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

        servoPos = ARM2.getPosition();
        Telemetry();

        while (gamepad1.left_trigger > 0) {
            LF.setPower(-1);
            LB.setPower(1);
            RF.setPower(-1);
            RB.setPower(1);
        }

        while (gamepad1.right_trigger > 0) {
            LF.setPower(1);
            LB.setPower(-1);
            RF.setPower(1);
            RB.setPower(-1);
        }

        if (gamepad2.b) {
            ElapsedTime elapsedTime = new ElapsedTime();
            FI.setPower(.5);
            elapsedTime.reset();

            while (elapsedTime.seconds() < 0.2) {
            }
            FI.setPower(0);
        }

        if (gamepad2.a) {
            ElapsedTime elapsedTime = new ElapsedTime();
            FI.setPower(-0.5);
            elapsedTime.reset();

            while (elapsedTime.seconds() < 0.2) {
            }
            FI.setPower(0);
        }


        while (gamepad1.right_stick_x > 0.2) {
            RF.setPower(1);
            LB.setPower(1);
        }

        while (gamepad1.right_stick_x < -0.2) {
            RF.setPower(-1);
            LB.setPower(-1);
        }

        while (gamepad1.left_stick_x < -0.2) {
            LF.setPower(1);
            RB.setPower(1);
        }

        while (gamepad1.left_stick_x > 0.2) {
            LF.setPower(-1);
            RB.setPower(-1);
        }

        while (gamepad2.right_bumper) {
            ARM2.setPosition(servoPos);
        }

        while (gamepad2.left_bumper) {
            ARM2.setPosition(servoPos);
        }
    }

    public void Telemetry() {
        telemetry.addData("ARM Position:", + servoPos);
    }
}
