
package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Subsystem_Intake {

    private DcMotor Intake;

    public double INTAKE_IN_POWER = 1.0;
    public double INTAKE_SHOOT_POWER = 1.0;
    public double INTAKE_REST_POWER = 0.0;
    public double INTAKE_REVERSE_POWER = -1.0;

    public double IntakeMotorTimer = 10.0;

    public Subsystem_Intake(HardwareMap hardwareMap) {

        Intake = hardwareMap.get(DcMotor.class, "Intake");

        Intake.setDirection(DcMotor.Direction.FORWARD);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // INTAKE FORWARD
    public static class Intake_Forward implements Action {

        private final DcMotor motor;
        private final double time;
        private final double power;
        private final double restPower;
        private final ElapsedTime timer = new ElapsedTime();

        public Intake_Forward(Subsystem_Intake intake) {
            this.motor = intake.Intake;
            this.time = intake.IntakeMotorTimer;
            this.power = intake.INTAKE_IN_POWER;
            this.restPower = intake.INTAKE_REST_POWER;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (timer.seconds() < time) {
                motor.setPower(power);
                return true;
            } else {
                motor.setPower(restPower);
                return false;
            }
        }
    }

    // INTAKE REVERSE
    public static class Intake_Reverse implements Action {

        private final DcMotor motor;
        private final double time;
        private final double power;
        private final double restPower;
        private final ElapsedTime timer = new ElapsedTime();

        public Intake_Reverse(Subsystem_Intake intake) {
            this.motor = intake.Intake;
            this.time = intake.IntakeMotorTimer;
            this.power = intake.INTAKE_REVERSE_POWER;
            this.restPower = intake.INTAKE_REST_POWER;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (timer.seconds() < time) {
                motor.setPower(power);
                return true;
            } else {
                motor.setPower(restPower);
                return false;
            }
        }
    }

    // INTAKE SHOOT
    public static class Intake_Shoot implements Action {

        private final DcMotor motor;
        private final double time;
        private final double power;
        private final double restPower;
        private final ElapsedTime timer = new ElapsedTime();

        public Intake_Shoot(Subsystem_Intake intake) {
            this.motor = intake.Intake;
            this.time = intake.IntakeMotorTimer;
            this.power = intake.INTAKE_SHOOT_POWER;
            this.restPower = intake.INTAKE_REST_POWER;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (timer.seconds() < time) {
                motor.setPower(power);
                return true;
            } else {
                motor.setPower(restPower);
                return false;
            }
        }
    }

    // INTAKE STOP
    public static class Intake_Stop implements Action {

        private final DcMotor motor;
        private final double restPower;

        public Intake_Stop(Subsystem_Intake intake) {
            this.motor = intake.Intake;
            this.restPower = intake.INTAKE_REST_POWER;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setPower(restPower);
            return false;
        }
    }
}
