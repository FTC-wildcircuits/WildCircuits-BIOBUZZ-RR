package org.firstinspires.ftc.teamcode.WC_Yatharth;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tele-Test", group = "Linear OpMode")
public class Tele_Test extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf_drive = null;
    private DcMotor rf_drive = null;
    private DcMotor lb_drive = null;
    private DcMotor rb_drive = null;
    private DcMotor Shooter = null;
    private DcMotor Intake = null;
    private CRServo GrabberL = null;
    private CRServo GrabberR = null;
    private Servo Launcher = null;

    // Servo Variables
    public double servoPosition = 0.0;
    public final double MIN_POSITION = 0.0;
    public final double MAX_POSITION = 1.0;
    public double positionAdjustment = 0.1;
    public double servoTargetPosition = 0.5;
    public double GRABBERL_IN_POWER = 1.0;
    public double GRABBERR_IN_POWER = 1.0;
    public double GRABBERL_REST_POWER = 0.0;
    public double GRABBERR_REST_POWER = 0.0;
    public double LAUNCHER_READY_POSITION = 0.0;
    public double LAUNCHER_SHOOT_POSITION = 0.5;

    // Motor Variables
    public double SHOOTER_SHOOT_POWER = 0.01;
    public double SHOOTER_REST_POWER = 0.0;
    public double INTAKE_IN_POWER = 1.0;
    public double INTAKE_REST_POWER = 0.0;
    double shooterclip;


    @Override
    public void runOpMode() {

        // Initialize the hardware map - Motors
        lf_drive = hardwareMap.get(DcMotor.class, "leftFront");
        rf_drive = hardwareMap.get(DcMotor.class, "rightFront");
        lb_drive = hardwareMap.get(DcMotor.class, "leftBack");
        rb_drive = hardwareMap.get(DcMotor.class, "rightBack");

        //Non-drive motors
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        // Initialize the hardware map - Servos
        GrabberL = hardwareMap.get(CRServo.class, "GrabberL");
        GrabberR = hardwareMap.get(CRServo.class, "GrabberR");
        Launcher = hardwareMap.get(Servo.class, "Launcher");

        // Setting the Drive motors Directions
        lf_drive.setDirection(DcMotor.Direction.REVERSE);
        lb_drive.setDirection(DcMotor.Direction.REVERSE);
        rf_drive.setDirection(DcMotor.Direction.FORWARD);
        rb_drive.setDirection(DcMotor.Direction.FORWARD);

        // Servo Directions
        Launcher.setDirection(Servo.Direction.REVERSE);

        // Setting Zero Power Brake to the Motors
        lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Non Drive Motors Encoder Modes
        Shooter.setDirection(DcMotor.Direction.FORWARD);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Launcher Position", Launcher.getPosition());
            telemetry.update();

            // Defining variable for the motor's power
            double lfPower;
            double rfPower;
            double rbPower;
            double lbPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            lfPower = (y + x + rx) / denominator;
            lbPower = (y - x + rx) / denominator;
            rfPower = (y - x - rx) / denominator;
            rbPower = (y + x - rx) / denominator;

            // Setting all the drive motors to their power
            lf_drive.setPower(lfPower);
            lb_drive.setPower(lbPower);
            rf_drive.setPower(rfPower);
            rb_drive.setPower(rbPower);

            if (gamepad2.a) {
                Launcher.setPosition(LAUNCHER_READY_POSITION);
            } else if (gamepad2.b) {
                Launcher.setPosition(LAUNCHER_SHOOT_POSITION);
            }

            if (gamepad2.right_bumper) {
                Intake.setPower(INTAKE_IN_POWER);
                GrabberL.setPower(GRABBERL_IN_POWER);
                GrabberR.setPower(GRABBERR_IN_POWER);
            } else if (gamepad2.left_bumper) {
                Intake.setPower(INTAKE_REST_POWER);
                GrabberL.setPower(GRABBERL_REST_POWER);
                GrabberR.setPower(GRABBERR_REST_POWER);
            }

            double shooterPower = -gamepad2.left_stick_y;
            shooterclip = Range.clip(shooterPower, -0.3, 0.3);
            Shooter.setPower(shooterclip);
        }
    }
}
