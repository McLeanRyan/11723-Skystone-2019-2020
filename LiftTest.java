package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LiftTest extends OpMode {
    DcMotor LIFT, FI;
    CRServo BOOM;
    @Override
    public void init() {
        telemetry.addLine("Initialized");
        telemetry.update();
        LIFT = hardwareMap.dcMotor.get("LIFT");
        FI = hardwareMap.dcMotor.get("FI");
        FI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BOOM = hardwareMap.crservo.get("BOOM");
    }

    @Override
    public void loop() {
        telemetry.addData("Lift Power", LIFT.getPower());
        telemetry.addData("Boom Power", BOOM.getPower());
        telemetry.update();
        FI.setPower(gamepad2.left_stick_y / 3);
        if (gamepad2.right_trigger > .3) {
            LIFT.setPower(1);
        } else {
            LIFT.setPower(0);
        }
        if (gamepad2.left_trigger > .3) {
            LIFT.setPower(-1);
        } else {
            LIFT.setPower(0);
        }
        if (gamepad2.right_stick_button) {
            LIFT.setPower(0);
        }
        BOOM.setPower(gamepad2.right_stick_y);
    }
}
