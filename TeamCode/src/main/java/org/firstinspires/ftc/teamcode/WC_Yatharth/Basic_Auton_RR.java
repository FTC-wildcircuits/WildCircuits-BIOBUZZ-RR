package org.firstinspires.ftc.teamcode.WC_Yatharth;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Basic RR Autonomous")
public class Basic_Auton_RR extends LinearOpMode {

    // Time into the Autonomous round
    public ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members and variables

    DcMotor Shooter = null;
    DcMotor Intake = null;

    CRServo GrabberL = null;
    CRServo GrabberR = null;

    CRServo Launcher = null;
   // ElapsedTime launchtimer = new ElapsedTime();
   // HuskyLens camera = null;


    // Servo Variables

    public final double MIN_POSITION = 0.0;
    public final double MAX_POSITION = 1.0;
    public double positionAdjustment = 0.01;
    public double GRABBERL_IN_POWER = 1.0;
    public double GRABBERR_IN_POWER = 1.0;
    public double GRABBERL_REST_POWER = 0.0;
    public double GRABBERR_REST_POWER = 0.0;
    public double GRABBERL_REVERSE_POWER = -1.0;
    public double GRABBERR_REVERSE_POWER = -1.0;
    public double LAUNCHER_IN_POWER = 1.0;
    public double LAUNCHER_REST_POWER = 0.0;
    public double LAUNCHER_REVERSE_POWER = -1.0;
    public boolean launchtimerstarted = false;

    // Timer Variables
    public double launchDuration = 10.0;
    public int CRGrabberTimer = 120;
    public int CRLauncherTimer = 120;
    public int IntakeMotorTimer = 10;
    public int ShooterMotorTimer = 120;
    double launcherclip;
    public double servoPosition = 0.0;

    // Motor Variables
    public double SHOOTER_SHOOT_POWER = 0.35;
    public double SHOOTER_REST_POWER = 0.0;
    public double SHOOTER_REVERSE_POWER = -0.35;
    public double INTAKE_IN_POWER = 1.0;
    public double INTAKE_REST_POWER = 0.0;
    public double INTAKE_REVERSE_POWER = -1.0;
    public double DRIVEMOTOR_POWER_ZERO = 0.0;
    double shooterclip;

    // Wait Variables
    public static double action_wait = 0.1;
    public static double pause = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d InitialPose = new Pose2d(0,0,0);     // Beginning pose

        //Importing the hardware maps for all drive motors and setting the robot position
        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        // Initialize the hardware map - Motors and Servos

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

       // camera = hardwareMap.get(HuskyLens.class, "HuskyLens");

        runtime.reset();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(0,0,0))
                                .strafeToLinearHeading(new Vector2d(20,-20), Math.toRadians(180))
                                .build(),
                        new MotorActionTimeBase(Intake, IntakeMotorTimer, INTAKE_IN_POWER)
                )
        );

    }

    // Servo action build for roadrunner to use
    public static class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer = null;

        public ServoAction(Servo srv, double pos) {
            this.servo = srv;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            return timer.seconds() < action_wait;
        }
    }

    // Double Motor Action build for roadrunner to use
    public static class DoubleMotorAction implements Action {
        DcMotor motor1;
        DcMotor motor2;
        double position_tgt1;
        double position_tgt2;
        double power1;
        double power2;
        ElapsedTime timer = null;

        public DoubleMotorAction(DcMotor mot1, DcMotor mot2, double pos1, double pos2, double pow1, double pow2) {
            this.motor1 = mot1;
            this.position_tgt1 = pos1;
            this.power1 = pow1;
            this.motor2 = mot2;
            this.position_tgt2 = -pos2;
            this.power2 = pow2;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (motor1.getCurrentPosition() < position_tgt1) {
                timer = new ElapsedTime();
                motor1.setPower(power1);
                motor2.setPower(power2);
                motor1.setTargetPosition((int) (position_tgt1));
                // motor2.setTargetPosition((int) (position_tgt2));
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            return timer.seconds() < action_wait;
        }
    }

    // Double Servo action build for roadrunner to use
    public static class DoubleServoAction implements Action {
        Servo servo1;
        double position1;
        Servo servo2;
        double position2;
        ElapsedTime timer = null;

        public DoubleServoAction(Servo srv1, Servo srv2, double pos1, double pos2) {
            this.servo1 = srv1;
            this.servo2 = srv2;
            this.position1 = pos1;
            this.position2 = pos2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo1.setPosition(position1);
                servo2.setPosition(position2);
            }

            return timer.seconds() < action_wait;
        }
    }

    // Motor Action-2 build for roadrunner to use
    public static class MotorAction2 implements Action {
        DcMotor motor;
        double position_tgt;
        double power;

        public MotorAction2(DcMotor mot, double pos, double pow) {
            this.motor = mot;
            this.position_tgt = pos;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (motor.getCurrentPosition() != position_tgt) {
                motor.setPower(power);
                motor.setTargetPosition((int) (position_tgt));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            } else {
                motor.setPower(0);
                return false;
            }
        }
    }

    // Motor Action-Time build for roadrunner to use
    public static class MotorActionTimeBase implements Action {
        DcMotor motor;
        double time;
        double power;
        ElapsedTime timer;

        public MotorActionTimeBase(DcMotor mot, double sec, double pow) {
            this.motor = mot;
            this.time = sec;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer.seconds() < time) {
                timer = new ElapsedTime();
                motor.setPower(power);
                return true;
            } else {
                motor.setPower(0);
                return false;
            }
        }
    }

    // CRServo Action build for roadrunner to use
    public class CRServo_Action implements Action {
        CRServo crServo;
        double power;
        int seconds;
        ElapsedTime timer = null;

        public CRServo_Action(CRServo c, double p, int s) {
            this.crServo = c;
            this.seconds = s;
            this.power = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer.seconds() < seconds) {
                timer = new ElapsedTime();
                crServo.setPower(power);
            }
            return false;
        }
    }

    // Continuous Motor Action build for roadrunner to use (Non-timed)
    public static class MotorActionContinuous implements Action {
        DcMotor motor;
        double power;
        boolean initialized = false;

        public MotorActionContinuous(DcMotor mot, double pow) {
            this.motor = mot;
            this.power = pow;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                motor.setPower(power);
                initialized = true;
            }
            // Return false immediately so RoadRunner moves to the next instruction
            // while the motor continues running in the background.
            return false;
        }
    }

    // CRServo Action build for roadrunner to use (Continuous / Non-timed)
    public static class CRServo_Action_Continuous implements Action {
        CRServo crServo;
        double power;
        boolean initialized = false;

        public CRServo_Action_Continuous(CRServo c, double p) {
            this.crServo = c;
            this.power = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                crServo.setPower(power);
                initialized = true;
            }
            // Return false immediately so RoadRunner moves on to the next action
            // while the servo keeps running continuously in the hardware background.
            return false;
        }
    }

}
