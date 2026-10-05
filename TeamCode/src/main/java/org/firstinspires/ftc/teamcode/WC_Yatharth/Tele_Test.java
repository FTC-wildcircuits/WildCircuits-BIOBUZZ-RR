package org.firstinspires.ftc.teamcode.WC_Yatharth;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tele-Test", group = "Linear OpMode")
public class Tele_Test extends LinearOpMode {

    // Declare OpMode members and variables
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lf_drive = null;
    private DcMotor rf_drive = null;
    private DcMotor lb_drive = null;
    private DcMotor rb_drive = null;

    private DcMotor Shooter = null;
    private DcMotor Intake = null;

    private CRServo GrabberL = null;
    private CRServo GrabberR = null;

    private CRServo Launcher = null;
    private ElapsedTime launchtimer = new ElapsedTime();


    // Servo Variables

    public final double MIN_POSITION = 0.0;
    public final double MAX_POSITION = 1.0;
    public double positionAdjustment = 0.01;
    public double GRABBERL_IN_POWER = 1.0;
    public double GRABBERR_IN_POWER = 1.0;
    public double GRABBERL_REST_POWER = 0.0;
    public double GRABBERR_REST_POWER = 0.0;
    public double LAUNCHER_IN_POWER = 0.5;
    public double LAUNCHER_REST_POWER = 0.0;
    public boolean launchtimerstarted = false;
    public double launchDuration = 4.0;
    public double servoPosition = 0.0;

    // Motor Variables
    public double SHOOTER_SHOOT_POWER = 0.1;
    public double SHOOTER_REST_POWER = 0.0;
    public double INTAKE_IN_POWER = 1.0;
    public double INTAKE_REST_POWER = 0.0;
    double shooterclip;


    @Override
    public void runOpMode() {

        // Initialize the hardware map - Motors
        lf_drive = hardwareMap.get(DcMotor.class, "leftFront");
        lf_drive.setDirection(DcMotor.Direction.REVERSE);
        lf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf_drive = hardwareMap.get(DcMotor.class, "rightFront");
        rf_drive.setDirection(DcMotor.Direction.FORWARD);
        rf_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lb_drive = hardwareMap.get(DcMotor.class, "leftBack");
        lb_drive.setDirection(DcMotor.Direction.REVERSE);
        lb_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rb_drive = hardwareMap.get(DcMotor.class, "rightBack");
        rb_drive.setDirection(DcMotor.Direction.FORWARD);
        rb_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setDirection(DcMotor.Direction.FORWARD);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the hardware map - Servos
        GrabberL = hardwareMap.get(CRServo.class, "GrabberL");
        GrabberL.setDirection(CRServo.Direction.FORWARD);

        GrabberR = hardwareMap.get(CRServo.class, "GrabberR");
        GrabberR.setDirection(CRServo.Direction.FORWARD);

        Launcher = hardwareMap.get(CRServo.class, "Launcher");
        Launcher.setDirection(CRServo.Direction.FORWARD);

        // Non Drive Motors Encoder Modes and Directions


        // Wait for start
        waitForStart();
        runtime.reset();

       // Launcher.setPosition(servoPosition);

        while (opModeIsActive()) {

            telemetry.addData("Launcher Power", Launcher.getPower());
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

            // Launcher servo incremental control
         /*   if (gamepad2.right_stick_y < -0.1) {
                servoPosition += positionAdjustment;
            } else if (gamepad2.right_stick_y > 0.1) {
                servoPosition -= positionAdjustment;
            }

            servoPosition = Range.clip(
                    servoPosition,
                    MIN_POSITION,
                    MAX_POSITION
            ); */

            //Launcher.setPosition(servoPosition);

            //Intake Motor and Grabber control
            if (gamepad2.right_bumper) {
                Intake.setPower(INTAKE_IN_POWER);
                GrabberL.setPower(GRABBERL_IN_POWER);
                GrabberR.setPower(GRABBERR_IN_POWER);
            } else if (gamepad2.left_bumper) {
                Intake.setPower(INTAKE_REST_POWER);
                GrabberL.setPower(GRABBERL_REST_POWER);
                GrabberR.setPower(GRABBERR_REST_POWER);
            }

            if (gamepad2.y) {
                Launcher.setPower(0.5);
            }

            //Shooter Motor control
            double shooterPower = -gamepad2.left_stick_y;
            shooterclip = Range.clip(shooterPower, -0.3, 0.3);
            Shooter.setPower(shooterclip);

            //Emergency Stop Button (create one for all non-drive motors and servos)
            if (gamepad2.x) {
                Shooter.setPower(SHOOTER_REST_POWER);
                Intake.setPower(INTAKE_REST_POWER);
                Launcher.setPower(LAUNCHER_REST_POWER);
                GrabberL.setPower(GRABBERL_REST_POWER);
                GrabberR.setPower(GRABBERR_REST_POWER);
            }


            //Time bound CRServo for Launcher

           /* if (!launchtimerstarted && gamepad2.a) {
                Launcher.setPower(LAUNCHER_IN_POWER);
                launchtimer.reset();
                launchtimerstarted = true;
            }
            if (launchtimerstarted && (launchtimer.seconds() >= launchDuration)) {
                Launcher.setPower(LAUNCHER_IN_POWER);
                launchtimerstarted = false;
            }

            if (gamepad2.b) {
                Launcher.setPower(LAUNCHER_REST_POWER);
            }*/
        }
    }
}


