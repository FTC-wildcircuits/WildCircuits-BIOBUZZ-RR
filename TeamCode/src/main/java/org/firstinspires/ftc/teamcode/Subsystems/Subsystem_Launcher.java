package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Subsystem_Launcher {

    private CRServo Launcher;

    public double LAUNCHER_IN_POWER = 1.0;
    public double LAUNCHER_REST_POWER = 0.0;
    public double LAUNCHER_REVERSE_POWER = -1.0;

    public Subsystem_Launcher(HardwareMap hardwareMap) {
        Launcher = hardwareMap.get(CRServo.class, "Launcher");
    }

    // FEED LAUNCHER
    public class Launcher_Feed implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Launcher.setPower(LAUNCHER_IN_POWER);

            // Continue running the servo in the background
            return false;
        }
    }

    // REVERSE LAUNCHER
    public class Launcher_Reverse implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            Launcher.setPower(LAUNCHER_REVERSE_POWER);

            return false;
        }
    }
}
