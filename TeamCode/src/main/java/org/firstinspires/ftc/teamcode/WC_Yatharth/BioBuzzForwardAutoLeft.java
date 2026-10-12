
package org.firstinspires.ftc.teamcode.WC_Yatharth;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BIOBUZZ Oct10 Auton Left", group = "BIOBUZZ")
public class BioBuzzForwardAutoLeft extends LinearOpMode {

    private DcMotor lf_drive;
    private DcMotor rf_drive;
    private DcMotor lb_drive;
    private DcMotor rb_drive;

    private double FORWARD_POWER = 0.25;
    private double REST_POWER = 0.0;
    private double RIGHT_POWER = 0.25;

    private int HALF_SECOND = 500;
    private int TWO_SECONDS = 2000;

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

        // Brake when stopped
        lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for start
        telemetry.addLine("BIOBUZZ Left Auto Ready");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Drive forward
            lf_drive.setPower(FORWARD_POWER);
            rf_drive.setPower(FORWARD_POWER);
            lb_drive.setPower(FORWARD_POWER);
            rb_drive.setPower(FORWARD_POWER);

            telemetry.addLine("Driving Forward!");
            telemetry.update();

            // Drive forward for half a second
            sleep(HALF_SECOND);

            // Strafe left
            lf_drive.setPower(-RIGHT_POWER);
            rf_drive.setPower(RIGHT_POWER);
            lb_drive.setPower(RIGHT_POWER);
            rb_drive.setPower(-RIGHT_POWER);

            telemetry.addLine("Strafing Left!");
            telemetry.update();

            // Strafe for two seconds
            sleep(TWO_SECONDS);

            // Stop all drive motors
            lf_drive.setPower(REST_POWER);
            rf_drive.setPower(REST_POWER);
            lb_drive.setPower(REST_POWER);
            rb_drive.setPower(REST_POWER);
        }

        // Ensure all drive motors are stopped
        lf_drive.setPower(REST_POWER);
        rf_drive.setPower(REST_POWER);
        lb_drive.setPower(REST_POWER);
        rb_drive.setPower(REST_POWER);
    }
}