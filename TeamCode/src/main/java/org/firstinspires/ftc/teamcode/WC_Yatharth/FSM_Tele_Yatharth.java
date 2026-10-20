package org.firstinspires.ftc.teamcode.WC_Yatharth;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "AAA_FSM Roadrunner Teleop")
public class FSM_Tele_Yatharth extends OpMode {

    private MecanumDrive mecanumDrive;

    public enum LiftState {
        RegTele,
        Intake,
        OutTake,
        Transfer,
        RTransfer,
        HiveShoot,
        Stop,
        Begin
    }


    // Declare OpMode members and variables
    ElapsedTime runtime = new ElapsedTime();

    DcMotor lf_drive;
    DcMotor rf_drive;
    DcMotor lb_drive;
    DcMotor rb_drive;

    DcMotor Shooter = null;
    DcMotor Intake = null;

//    CRServo GrabberL = null;
//    CRServo GrabberR = null;

    CRServo Launcher = null;
    ElapsedTime launchtimer = new ElapsedTime();

    // IMU robotImu = null;

   // HuskyLens camera = null;

    private LiftState CurrentState;
    private boolean EmergencyStop;


    // Servo Variables

    public final double MIN_POSITION = 0.0;
    public final double MAX_POSITION = 1.0;
    public double positionAdjustment = 0.01;
//    public double GRABBERL_IN_POWER = 1.0;
//    public double GRABBERR_IN_POWER = 1.0;
//    public double GRABBERL_REST_POWER = 0.0;
//    public double GRABBERR_REST_POWER = 0.0;
//    public double GRABBERL_REVERSE_POWER = -1.0;
//    public double GRABBERR_REVERSE_POWER = -1.0;
    public double LAUNCHER_IN_POWER = 0.2;
    public double LAUNCHER_REST_POWER = 0.0;
    public double LAUNCHER_REVERSE_POWER = -0.2;
    public boolean launchtimerstarted = false;

    // Timer Variables
    public double launchDuration = 10.0;
    //public int CRGrabberTimer = 120;
    public int CRLauncherTimer = 120;
    public int IntakeMotorTimer = 120;
    public int ShooterMotorTimer = 120;
    double launcherclip;
    public double servoPosition = 0.0;

    // Motor Variablesn
    public double SHOOTER_SHOOT_POWER = 0.475;
    public double SHOOTER_REST_POWER = 0.0;
    public double SHOOTER_REVERSE_POWER = -0.475;
    public double INTAKE_IN_POWER = 1.0;
    public double INTAKE_SHOOT_POWER = 1.0;
    public double INTAKE_REST_POWER = 0.0;
    public double INTAKE_REVERSE_POWER = -1.0;
    public double DRIVEMOTOR_POWER_ZERO = 0.0;
    double shooterclip;
   // double v;

    // Wait Variables
    public static double action_wait = 0.1;
    public static double pause = 0.7;

    @Override
    public void init() {

        Pose2d InitialPose = new Pose2d(0, 0, 0);     // Beginning pose

        mecanumDrive = new MecanumDrive(hardwareMap, InitialPose);

        runtime.reset();

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
//        GrabberL = hardwareMap.get(CRServo.class, "GrabberL");
//        GrabberL.setDirection(CRServo.Direction.FORWARD);
//
//        GrabberR = hardwareMap.get(CRServo.class, "GrabberR");
//        GrabberR.setDirection(CRServo.Direction.REVERSE);

        Launcher = hardwareMap.get(CRServo.class, "Launcher");
        Launcher.setDirection(CRServo.Direction.REVERSE);

       // camera = hardwareMap.get(HuskyLens.class, "HuskyLens");

        CurrentState = LiftState.RegTele;
        EmergencyStop = false;
    }

    public void loop() {

        move();
        Pose2d currentpose = mecanumDrive.localizer.getPose();

        // Displaying Telemetry
        telemetry.addData("State", CurrentState);
        telemetry.addData("Current Pose", currentpose.position);
        telemetry.addData("Current Pose", currentpose.heading);
        telemetry.addData("Shooter Power", Shooter.getPower());
        telemetry.addData("Intake Power", Intake.getPower());
//        telemetry.addData("GrabberL Power", GrabberR.getPower());
//        telemetry.addData("GrabberR Power", GrabberR.getPower());
        telemetry.addData("Launcher Power", Launcher.getPower());
       // telemetry.addData("Voltage", v);
        telemetry.update();

        switch (CurrentState) {
            case RegTele:
                move();
                // On Gamepad 1 if Needed
                TeleOperations();
                //Other States
                if (gamepad2.x) {
                    CurrentState = LiftState.Intake;
                } else if (gamepad2.y) {
                    CurrentState = LiftState.OutTake;
                } else if (gamepad2.right_bumper) {
                    CurrentState = LiftState.HiveShoot;
                } else if (gamepad2.b) {
                    CurrentState = LiftState.Transfer;
                } else if (gamepad2.left_bumper) {
                    CurrentState = LiftState.RTransfer;
                } else if (gamepad2.dpad_left) {
                    CurrentState = LiftState.Stop;
                } else if (gamepad2.start) {
                    CurrentState = LiftState.Begin;
                }
                telemetry.update();
                break;

            case Intake:
                move();
                Shooter.setPower(SHOOTER_REST_POWER);
                Launcher.setPower(LAUNCHER_REST_POWER);
                Actions.runBlocking(new ParallelAction(
//                        new CRServo_Action_Continuous(GrabberR, GRABBERR_IN_POWER),
//                        new CRServo_Action_Continuous(GrabberL, GRABBERL_IN_POWER),
                        new MotorActionContinuous(Intake, INTAKE_IN_POWER)

                ));
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case OutTake:
                move();
                Actions.runBlocking(new ParallelAction(
//                        new CRServo_Action_Continuous(GrabberR, GRABBERR_REVERSE_POWER),
//                        new CRServo_Action_Continuous(GrabberL, GRABBERL_REVERSE_POWER),
                        new MotorActionContinuous(Intake, INTAKE_REVERSE_POWER)
                ));
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case HiveShoot:
                move();
//                GrabberR.setPower(GRABBERR_REST_POWER);
//                GrabberL.setPower(GRABBERL_REST_POWER);
               // v = hardwareMap.voltageSensor.iterator().next().getVoltage();
                Actions.runBlocking(new SequentialAction(
                        new MotorActionContinuous(Shooter, SHOOTER_SHOOT_POWER),
                        new SleepAction(1),
                        new ParallelAction(
                        new CRServo_Action_Continuous(Launcher, LAUNCHER_IN_POWER),
                        new MotorActionContinuous(Intake, INTAKE_SHOOT_POWER)
                )));
                /*Shooter.setPower(SHOOTER_REST_POWER);
                Intake.setPower(INTAKE_REST_POWER);
                Launcher.setPower(LAUNCHER_REST_POWER);*/
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case Transfer:
                move();
//                GrabberR.setPower(GRABBERR_REST_POWER);
//                GrabberL.setPower(GRABBERL_REST_POWER);
                Actions.runBlocking(new ParallelAction(
                        new CRServo_Action_Continuous(Launcher, LAUNCHER_IN_POWER),
                        new MotorActionContinuous(Intake, INTAKE_IN_POWER),
                        new MotorActionContinuous(Shooter, SHOOTER_SHOOT_POWER)
                ));
                /*Intake.setPower(INTAKE_REST_POWER);
                Launcher.setPower(LAUNCHER_REST_POWER);*/
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            case RTransfer:
                move();
//                GrabberR.setPower(GRABBERR_REST_POWER);
//                GrabberL.setPower(GRABBERL_REST_POWER);
                Actions.runBlocking(new SequentialAction(
                        new CRServo_Action_Continuous(Launcher, LAUNCHER_REVERSE_POWER),
                        new MotorActionContinuous(Intake, INTAKE_IN_POWER),
                        new MotorActionContinuous(Shooter, SHOOTER_SHOOT_POWER)
                ));
               /* Intake.setPower(INTAKE_REST_POWER);
                Launcher.setPower(LAUNCHER_REST_POWER);*/
                telemetry.update();
                CurrentState = LiftState.RegTele;
                break;

            // Emergency Stop Button Just in case
            case Stop:
                lf_drive.setPower(DRIVEMOTOR_POWER_ZERO);
                rf_drive.setPower(DRIVEMOTOR_POWER_ZERO);
                lb_drive.setPower(DRIVEMOTOR_POWER_ZERO);
                rb_drive.setPower(DRIVEMOTOR_POWER_ZERO);

                Shooter.setPower(SHOOTER_REST_POWER);
                Intake.setPower(INTAKE_REST_POWER);

                Launcher.setPower(LAUNCHER_REST_POWER);
//                GrabberL.setPower(GRABBERL_REST_POWER);
//                GrabberR.setPower(GRABBERR_REST_POWER);
                EmergencyStop = true;

                if (gamepad2.dpad_left) {
                    EmergencyStop = false;
                    CurrentState = LiftState.RegTele;
                }
                break;

            // Only Pressed at the Start of Teleop
            case Begin:
                move();
                Actions.runBlocking(new ParallelAction(
//                        new CRServo_Action_Continuous(GrabberR, GRABBERR_IN_POWER),
//                        new CRServo_Action_Continuous(GrabberL, GRABBERL_IN_POWER),
                        new MotorActionContinuous(Intake, INTAKE_IN_POWER)
                ));
               /* Intake.setPower(INTAKE_REST_POWER);
                GrabberR.setPower(GRABBERR_REST_POWER);
                GrabberL.setPower(GRABBERL_REST_POWER);*/
                telemetry.update();
                CurrentState = LiftState.RegTele;
        }
    }

    // Driving Using Roadrunner MecanumDrive Class
    public void move() {
        //Driving
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        PoseVelocity2d drivePower = new PoseVelocity2d(new Vector2d(x, y), rx);

        // Use MecanumDrive class to drive the robot
        mecanumDrive.setDrivePowers(drivePower);
        mecanumDrive.updatePoseEstimate(); // Update the drive to apply motor powers
    }

    // Basic Teleop Functions
    public void TeleOperations() {

        //Intake Motor and Grabber control
       /* if (gamepad1.right_bumper) {
            Intake.setPower(INTAKE_IN_POWER);
            GrabberL.setPower(GRABBERL_IN_POWER);
            GrabberR.setPower(GRABBERR_IN_POWER);
        } else if (gamepad1.left_bumper) {
            Intake.setPower(INTAKE_REST_POWER);
            GrabberL.setPower(GRABBERL_REST_POWER);
            GrabberR.setPower(GRABBERR_REST_POWER);
        }*/
        //Shooter Motor control
      /*  double shooterPower = -gamepad2.left_stick_y;
        shooterclip = Range.clip(shooterPower, -SHOOTER_SHOOT_POWER, SHOOTER_SHOOT_POWER);
        Shooter.setPower(shooterclip);*/

        //Emergency Stop Button (create one for all non-drive motors and servos)
      /*  if (gamepad1.x) {
            Shooter.setPower(SHOOTER_REST_POWER);
            Intake.setPower(INTAKE_REST_POWER);
            Launcher.setPower(LAUNCHER_REST_POWER);
            GrabberL.setPower(GRABBERL_REST_POWER);
            GrabberR.setPower(GRABBERR_REST_POWER);
        }*/
        //Time bound CRServo for Launcher

      /*  if (!launchtimerstarted && gamepad1.a) {
            Launcher.setPower(LAUNCHER_IN_POWER);
            launchtimer.reset();
            launchtimerstarted = true;
        }
        if (launchtimerstarted && (launchtimer.seconds() >= launchDuration)) {
            Launcher.setPower(LAUNCHER_IN_POWER);
            launchtimerstarted = false;
        }

        if (gamepad1.b) {
            Launcher.setPower(LAUNCHER_REST_POWER);
        }
        if (gamepad1.a) {
            Launcher.setPower(LAUNCHER_IN_POWER);
        } else if (gamepad1.b) {
            Launcher.setPower(LAUNCHER_REVERSE_POWER);
        } else {
            Launcher.setPower(LAUNCHER_REST_POWER);
        }*/

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
        ElapsedTime timer = null;

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