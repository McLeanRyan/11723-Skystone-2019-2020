package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;

@Autonomous
public class BlueFoundation extends LinearOpMode {

    DcMotor LF, RF, LB, RB, FI, LIFT;
    CRServo ARM2, BOOM;

    BNO055IMU imu;
    private Orientation angles;

    double ticksPerMotorRev = 383.6;
    double driveGearReduction = 2;
    double wheelDiameterInches = 4;
    double ticksPerInch = (ticksPerMotorRev * driveGearReduction) / (wheelDiameterInches * 3.14159265359);

    //about 30" to foundation
    // 45" to line

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

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        FI.setPower(0);
        LIFT.setPower(0);

        double drivePower = .4;
        double turnPower = .2;

        ARM2.setPower(0);
        BOOM.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "REVHub1IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        waitForStart();

        while (opModeIsActive()) {
            encoderDrive(.15,-21,false);

            ARM2.setPower(1);
            //sleep();

            encoderDrive(.15,20, false);

            ARM2.setPower(-1);
            sleep(800);
            ARM2.setPower(0);

            turn(-0.1, 90);

            encoderDrive(.15,30,false);
        }
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

                telemetry.addData("LFM Current Pos", LF.getCurrentPosition());
                telemetry.addData("RFM Current Pos", RF.getCurrentPosition());
                telemetry.addData("LBM Current Pos", LB.getCurrentPosition());
                telemetry.addData("RBM Current Pos", RB.getCurrentPosition());
                telemetry.addData("LFM Current Power", LF.getPower());
                telemetry.addData("RFM Current Power", RF.getPower());
                telemetry.addData("LBM Current Power", LB.getPower());
                telemetry.addData("RBM Current Power", RB.getPower());
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

            sleep(100);
        }
    }

    private void turn(double speed, double difference) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
        double startAngle = angles.firstAngle;
        double targetAngle = angles.firstAngle + difference;
        boolean turned = false;
        while (!turned) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES);
            double error = (angles.firstAngle - startAngle);
            LF.setPower(speed - error);
            LB.setPower(speed - error);
            RF.setPower(speed + error);
            RB.setPower(speed + error);
            if((Math.abs(angles.firstAngle - targetAngle)) < 5) {
                turned = true;
            }
        }
    }
}