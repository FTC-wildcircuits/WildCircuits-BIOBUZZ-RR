package org.firstinspires.ftc.teamcode.WC_Yatharth;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Servo Position Helper", group = "Concept")
public class ServoPositionHelper extends OpMode {

    // private Servo el = null;
    private Servo er = null;

   // private double servoPositionel = 0;
    private double servoPositioner = 0;

    private double positionAdjustment = 0.05;
   // private final double STEP_ADJUSTMENT = 0.05;
    private final double MIN_POSITION = -1.0;
    private final double MAX_POSITION = 1.0;

    private boolean previousGamepadY = false;
    private boolean previousGamePadA = false;
    private boolean previousGamePadUp = false;
    private boolean previousGamePadDown = false;

    @Override
    public void init() {


       // el = hardwareMap.get(Servo.class, "Twist Left");
        // el.setDirection(Servo.Direction.REVERSE);
        // el.setPosition(servoPositionaxon);

        er = hardwareMap.get(Servo.class, "Claw");
        er.setDirection(Servo.Direction.FORWARD);
        er.setPosition(servoPositioner);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        boolean currentGamepadY = gamepad1.y;
        boolean currentGamepadA = gamepad1.a;
        boolean currentGamepadUp = gamepad1.dpad_up;
        boolean currentGamepadDown = gamepad1.dpad_down;

        if (currentGamepadY && !previousGamepadY) {
            servoPositioner += positionAdjustment;
        } else if (currentGamepadA && !previousGamePadA) {
            servoPositioner -= positionAdjustment;
        }

        if (positionAdjustment < 0.01) {
            positionAdjustment = 0.01;
        } else if (positionAdjustment > 0.1) {
            positionAdjustment = 0.1;
        }

        if (servoPositioner > MAX_POSITION) {
            servoPositioner = MAX_POSITION;
        } else if (servoPositioner < MIN_POSITION) {
            servoPositioner = MIN_POSITION;
        }

        //el.setPosition(servoPositionel);
        er.setPosition(servoPositioner);

        previousGamepadY = currentGamepadY;
        previousGamePadA = currentGamepadA;
        previousGamePadUp = currentGamepadUp;
        previousGamePadDown = currentGamepadDown;

//        telemetry.addData("El Servo Position", el.getPosition());
        telemetry.addData("Er Servo Position", er.getPosition());
        telemetry.addData("Target Servo Position", servoPositioner);
        telemetry.addData("Servo Step Size", positionAdjustment);
        telemetry.update();
    }
}
