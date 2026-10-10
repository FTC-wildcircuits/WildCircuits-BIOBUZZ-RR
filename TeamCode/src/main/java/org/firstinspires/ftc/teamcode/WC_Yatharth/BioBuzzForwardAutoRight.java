package org.firstinspires.ftc.teamcode.WC_Yatharth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BIOBUZZ Oct10 Auton Right", group = "BIOBUZZ")
public class BioBuzzForwardAutoRight extends LinearOpMode {

    private DcMotor lf_drive;
    private DcMotor rf_drive;
    private DcMotor lb_drive;
    private DcMotor rb_drive;

    private double FORWARD_POWER = 0.25;
    private double REST_POWER = 0.0;
    private double LEFT_POWER = 0.25;
    private int HALF_SECOND = 500;
    private int SEVEN_SECONDS = 7000;

    @Override
    public void runOpMode() {

        // Initialize drive motors
        lf_drive = hardwareMap.get(DcMotor.class, "leftFront");
        rf_drive = hardwareMap.get(DcMotor.class, "rightFront");
        lb_drive = hardwareMap.get(DcMotor.class, "leftBack");
        rb_drive = hardwareMap.get(DcMotor.class, "rightBack");

        // Match the directions from your TeleOp
        lf_drive.setDirection(DcMotor.Direction.REVERSE);
        rf_drive.setDirection(DcMotor.Direction.FORWARD);
        lb_drive.setDirection(DcMotor.Direction.REVERSE);
        rb_drive.setDirection(DcMotor.Direction.FORWARD);

        // Stop safely when power is set to zero
        lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the START button
        telemetry.addLine("BIOBUZZ Forward Auto Ready");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Drive forward at 25% power
            lf_drive.setPower(FORWARD_POWER);
            rf_drive.setPower(FORWARD_POWER);
            lb_drive.setPower(FORWARD_POWER);
            rb_drive.setPower(FORWARD_POWER);

            telemetry.addLine("Driving Forward!");
            telemetry.update();

            // Drive for 1/2 a second
            sleep(HALF_SECOND);

            // Strafe left at 25% power
            lf_drive.setPower(-LEFT_POWER);
            rf_drive.setPower(LEFT_POWER);
            lb_drive.setPower(LEFT_POWER);
            rb_drive.setPower(-LEFT_POWER);

            telemetry.addLine("Strafing Right!");
            telemetry.update();

            // Strafe for 4 seconds
            sleep(SEVEN_SECONDS);

            // Stop all drive motors
            lf_drive.setPower(REST_POWER);
            rf_drive.setPower(REST_POWER);
            lb_drive.setPower(REST_POWER);
            rb_drive.setPower(REST_POWER);
        }

        // Ensure the robot is stopped
        lf_drive.setPower(REST_POWER);
        rf_drive.setPower(REST_POWER);
        lb_drive.setPower(REST_POWER);
        rb_drive.setPower(REST_POWER);
    }
}