package org.firstinspires.ftc.teamcode.WC_Yatharth;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "AA Oct10 Teleop")
public class Tele_Yatharth extends LinearOpMode {

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

    private IMU robotImu = null;


    // Servo Variables

    public final double MIN_POSITION = 0.0;
    public final double MAX_POSITION = 1.0;
    public double positionAdjustment = 0.01;
    public double GRABBERL_IN_POWER = 1.0;
    public double GRABBERR_IN_POWER = 1.0;
    public double GRABBERL_REST_POWER = 0.0;
    public double GRABBERR_REST_POWER = 0.0;
    public double LAUNCHER_IN_POWER = 1.0;
    public double LAUNCHER_REST_POWER = 0.0;
    public double LAUNCHER_REVERSE_POWER = -1.0;
    public boolean launchtimerstarted = false;
    public double launchDuration = 10.0;
    double launcherclip;
    public double servoPosition = 0.0;

    // Motor Variables
    public double SHOOTER_SHOOT_POWER = 0.55;
    public double SHOOTER_REST_POWER = 0.0;
    public double INTAKE_IN_POWER = 1.0;
    public double INTAKE_REST_POWER = 0.0;
    double shooterclip;

    // Drive Variables
    public double STRAFE_CORRECTION = 1.1; // Counteract imperfect strafing, tune as needed


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
        //Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotor.Direction.REVERSE);
        //Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the hardware map - Servos
        GrabberL = hardwareMap.get(CRServo.class, "GrabberL");
        GrabberL.setDirection(CRServo.Direction.FORWARD);

        GrabberR = hardwareMap.get(CRServo.class, "GrabberR");
        GrabberR.setDirection(CRServo.Direction.FORWARD);

        Launcher = hardwareMap.get(CRServo.class, "Launcher");
        Launcher.setDirection(CRServo.Direction.FORWARD);

        // Initialize the hardware map - IMU
        // Make sure the two values are set depending on how the
        // control hub is mounted on the robot
        robotImu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        robotImu.initialize(imuParameters);

        // Non Drive Motors Encoder Modes and Directions


        // Wait for start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Launcher.setPosition(servoPosition);

        while (opModeIsActive()) {

            // Let the driver reset the "forward" direction by pressing Options/Back.
            // Do this while the robot is pointed the way you want to call "forward".
            if (gamepad1.options) {
                robotImu.resetYaw();
            }

            // Get the robot's current heading from the IMU.
            double heading = robotImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("Launcher Power", Launcher.getPower());
            telemetry.update();

            // Defining variable for the motor's power
            double lfPower;
            double rfPower;
            double rbPower;
            double lbPower;

            // Field-centric drive: left stick moves the robot relative to the field,
            // and right stick turns.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Rotate the joystick's field-relative direction by the negative of the
            // robot's heading, converting it into robot-relative x/y values.
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            rotX = rotX * STRAFE_CORRECTION;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            lfPower = (rotY + rotX + rx) / denominator;
            lbPower = (rotY - rotX + rx) / denominator;
            rfPower = (rotY - rotX - rx) / denominator;
            rbPower = (rotY + rotX - rx) / denominator;

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

//            double launcherPower = gamepad2.right_stick_x;
//            launcherclip = Range.clip(launcherPower, 0.0, 1.0);
//            Launcher.setPower(launcherclip);

//            if (gamepad2.y) {
//                Launcher.setPower(0.5);
//            }

            //Shooter Motor control
            double shooterPower = -gamepad2.left_stick_y;
            shooterclip = Range.clip(shooterPower, -0.55, 0.55);
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

            if (gamepad2.a) {
                Launcher.setPower(LAUNCHER_IN_POWER);
            } else if (gamepad2.b) {
                Launcher.setPower(LAUNCHER_REVERSE_POWER);
            } else {
                Launcher.setPower(LAUNCHER_REST_POWER);
            }
            }
        }
    }