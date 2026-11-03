
package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Subsystem_Shooter {

    private DcMotor Shooter;

    public double SHOOTER_SHOOT_POWER = 0.55;
    public double SHOOTER_REST_POWER = 0.0;
    public double SHOOTER_REVERSE_POWER = -0.55;

    public double ShooterMotorTimer = 1.2;

    public Subsystem_Shooter(HardwareMap hardwareMap) {

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        Shooter.setDirection(DcMotor.Direction.FORWARD);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // SHOOT FORWARD
    public class Shoot_Forward implements Action {

        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (timer.seconds() < ShooterMotorTimer) {

                Shooter.setPower(SHOOTER_SHOOT_POWER);

                return true;

            } else {

                Shooter.setPower(SHOOTER_REST_POWER);

                return false;
            }
        }
    }

    // SHOOT IN REVERSE
    public class Shoot_Reverse implements Action {

        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (timer.seconds() < ShooterMotorTimer) {

                Shooter.setPower(SHOOTER_REVERSE_POWER);

                return true;

            } else {

                Shooter.setPower(SHOOTER_REST_POWER);

                return false;
            }
        }
    }

    // STOP SHOOTER
    public class Shooter_Stop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Shooter.setPower(SHOOTER_REST_POWER);

            return false;
        }
    }
}
