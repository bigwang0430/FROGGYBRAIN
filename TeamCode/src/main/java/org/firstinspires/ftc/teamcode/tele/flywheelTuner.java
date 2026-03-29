package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.vars.globals;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name = "flywheelTuner")
public class flywheelTuner extends OpMode {
    private Motor l1, l2;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private GamepadEx g1;
    private double RPM, lastTime, previousRPM;
    private int lastPosition;
    @Override
    public void init() {
        l1 = new Motor(hardwareMap, "l1", 28, 6000);
        l2 = new Motor(hardwareMap, "l2", 28, 6000);
        l1.setRunMode(Motor.RunMode.RawPower);
        l2.setRunMode(Motor.RunMode.RawPower);
        l2.setInverted(true);
        l1.setInverted(false);
        l1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        l2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        g1 = new GamepadEx(gamepad1);
        launchPIDF.setTolerance(50);
    }

    @Override
    public void loop() {
        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", globals.launcher.targetRPM);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
        telemetry.update();
        launchPIDF.setSetPoint(globals.launcher.targetRPM);
        double launchPower = launchPIDF.calculate(RPM);
        if (g1.getButton(GamepadKeys.Button.CROSS)) {
//            if (RPM < 400) {
//                l1.set(0.55);
//                l2.set(0.55);
//            } else {
//
//            }
            l1.set(launchPower + globals.launcher.kv * globals.launcher.targetRPM + globals.launcher.ks);
            l2.set(launchPower + globals.launcher.kv * globals.launcher.targetRPM + globals.launcher.ks);
        } else{
            l1.set(0);
            l2.set(0);
        }
        RPM();
    }

    public void RPM() {
        double currentTime = getRuntime();
        int currentPosition = l2.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        double deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.02) {
            previousRPM = RPM;
            double revs = deltaTicks / 28.0; // GoBILDA CPR
            RPM = -(revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }
}
