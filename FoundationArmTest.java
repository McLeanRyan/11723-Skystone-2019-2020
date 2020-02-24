package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class FoundationArmTest extends LinearOpMode {
    Servo ARM;

    @Override
    public void runOpMode() throws InterruptedException {
        ARM = hardwareMap.servo.get("ARM2");
        telemetry.addData("Foundation Arm Pos", ARM.getPosition());
        telemetry.update();
        double ARMStartingPos = ARM.getPosition();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Foundation Arm Pos", ARM.getPosition());
            telemetry.update();
            ARM.setPosition(.1);
        }
    }
}
