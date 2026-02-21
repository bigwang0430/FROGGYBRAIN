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
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.vars.globals;

@TeleOp(name = "turretttestt")
public class turretTest extends OpMode {
    private PIDController turretPIDF = new PIDController(globals.turret.pFarTele, globals.turret.i, globals.turret.d);
    private CRServoEx t1, t2;
    private AnalogInput turretEncoder;
    private Motor intake;

    @Override
    public void init() {
        t1 = new CRServoEx(hardwareMap, "t1");
        t2 = new CRServoEx(hardwareMap, "t2");
        t2.setInverted(true);
        t1.setInverted(true);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        intake = new Motor(hardwareMap, "intake");
        turretPIDF.setTolerance(67);
        t1.set(0.01);
        t2.set(0.01);

    }

    @Override
    public void loop() {
//        if (Math.abs(turretPIDF.getPositionError()) > 1000) {
//            turretPIDF.setP(globals.turret.pFar);
//        } else {
//            turretPIDF.setP(globals.turret.pClose);
//        }
//        turretPIDF.setSetPoint(globals.turret.turretTarget);
//        double turretPower = MathFunctions.clamp(turretPIDF.calculate(intake.getCurrentPosition()), -1, 1);
//
//
//            t1.set(setTurret(turretPower));
//            t2.set(setTurret(turretPower));

//        if (!turretPIDF.atSetPoint()) {
//            t1.set(turretpos);
//            t2.set(turretpos);
//        } else {
//            t1.set(0);
//            t2.set(0);
//        }

        t1.set(0);
        t2.set(0);

        telemetry.addData("voltage", turretEncoder.getVoltage());
        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("pos", intake.getCurrentPosition());

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("target", globals.turret.turretTarget);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
        telemetry.update();

    }

    public double degresToTicks(double degree) {
        return (degree * 8192) / 360;
    }

    public double voltageToDegrees(double volts) {
        return (volts * 360) / 3.2;
    }

    private double setTurret(double power) {

        return Math.signum(power) * (Math.abs(power) + globals.turret.ks);
    }
}
