package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.vars.globals;

@TeleOp(name = "turretttestt")
public class turretTest extends OpMode {
    private PIDController turretPIDF = new PIDController(globals.turret.p, globals.turret.i, globals.turret.d);
    private CRServoEx t1, t2;
    private AnalogInput turretEncoder;
    private Motor intake;
    @Override
    public void init() {
        t1 = new CRServoEx(hardwareMap, "t1");
        t2 = new CRServoEx(hardwareMap, "t2");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        intake = new Motor(hardwareMap, "intake");


    }

    @Override
    public void loop() {
        turretPIDF.setPID(globals.turret.p, globals.turret.i, globals.turret.d);
        turretPIDF.setSetPoint(globals.turret.turretTarget);
        double turretPower = MathFunctions.clamp(turretPIDF.calculate(intake.getCurrentPosition()), -1, 1);
        if (turretPower < 0.00001) {
            turretPower = 0;
        }

        t1.set(turretPower);
        t2.set(turretPower);

        telemetry.addData("pwoer", turretPower);
        telemetry.addData("pos", intake.getCurrentPosition());
        telemetry.addData("voltage", turretEncoder.getVoltage());
        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("pos", intake.getCurrentPosition());

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("target", globals.turret.turretTarget);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
        telemetry.update();

    }
}
