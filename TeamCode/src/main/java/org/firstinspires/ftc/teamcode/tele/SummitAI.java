package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vars.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vars.states;

import java.util.List;
import java.util.Objects;

@TeleOp(name = "SummitAI")
public class SummitAI extends OpMode {
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(17, 17);

    private Motor l1, l2, intake, transfer;
    private ServoEx hood, gate, tiltl, tiltr, lights;
    private ServoEx t1, t2;
    private AnalogInput turretEncoder;
    private GamepadEx g1;
    private Follower follower;
    private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
    private ElapsedTime timer = new ElapsedTime();

    private enum launchMode {
        SOTM,
        normal
    } private launchMode currentLaunchMode = launchMode.normal;
    private enum launchState {
        idle,
        intaking,
        launching

    } private launchState currentLaunchState = launchState.idle;
    private String robotLocation = "No Zone";


    private double lastTime, launchPower, RPM, previousRPM, dist, turretAng, targetRPM, hoodAngle, leftY, leftX, turretPower;
    private double turretPos = 0F;
    private int lastPosition;
    private boolean prevCross1, prevOptions2, prevTriggerR, prevTriggerL;
    private boolean autoAim = true;
    private boolean slowDrive = false;
    private boolean turretZeroed = false, turretInRange;
    private int ballsLaunched = 0;
    private double turretZeroOffset;

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style(
            "", "#3F51B5", 1
    );

    private double xEst = 0, yEst = 0, hEst = 0;   // odometry estimate

    private boolean zapLeon = false, leftBumper = false, rightBumper = false;
    private ElapsedTime relocTimer = new ElapsedTime();
    private double filteredAccelMag = 0;
    private double filteredAccelAngle = 0F;
    private Limelight3A limelight;
    private FtcDashboard dashboard;


    @Override
    public void init() {

        relocTimer.startTime();
        timer.startTime();
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        t1 = new ServoEx(hardwareMap, "t1", 360);
        t2 = new ServoEx(hardwareMap, "t2", 360);
        t2.setInverted(true);
        t1.setInverted(true);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        l1 = new Motor(hardwareMap, "l1", 28, 6000);
        l2 = new Motor(hardwareMap, "l2", 28, 6000);
        l1.setRunMode(Motor.RunMode.RawPower);
        l2.setRunMode(Motor.RunMode.RawPower);
        l2.setInverted(true);
        l1.setInverted(false);
        l1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        l2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        gate = new ServoEx(hardwareMap, "gate");
        gate.set(globals.gate.close);

        intake = new Motor(hardwareMap, "intake");
        intake.stopAndResetEncoder();
        intake.resetEncoder();

        tiltl = new ServoEx(hardwareMap, "tiltl");
        tiltr = new ServoEx(hardwareMap, "tiltr");
        lights = new ServoEx(hardwareMap, "lights");
        lights.set(0);


        transfer = new Motor(hardwareMap, "transfer");
        intake.setRunMode(Motor.RunMode.RawPower);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer.setInverted(true);
        intake.setInverted(false);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hardwareMap, "hood", 300);

        g1 = new GamepadEx(gamepad1);
        launchPIDF.setTolerance(50);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose (8, 8)); //TEMPORARY


        while (timer.seconds() < 1) {
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }

        timer.reset();
        timer.startTime();



        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(20);
        limelight.pipelineSwitch(0);
        limelight.start();
        turretZeroOffset = Math.abs(turretEncoder.getVoltage()) < 0.005 ? 0 : (degresToTicks(voltageToDegrees(turretEncoder.getVoltage() - 1.6)) * 2) + globals.turret.turretOffset;
        tiltl.set(0.83);
        tiltr.set(0.17);


        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(limelight, 30);

    }

    @Override
    public void loop() {
        follower.update();
        RPM();
        launchCalc();
        drive();
        launch();
        drawRobot(follower.getPose(), robotLook);
        turretAim();

        telemetry();
    }

    private void telemetry() {
        telemetry.addData("distance", dist);
        telemetry.addData("autoaim", autoAim);
        telemetry.addData("robotLocation", robotLocation);


        TelemetryPacket ang = new TelemetryPacket();
        ang.put("target", globals.launcher.targetRPM);

        TelemetryPacket fang = new TelemetryPacket();
        fang.put("rpm", RPM);

        TelemetryPacket mag = new TelemetryPacket();
        mag.put("mag", follower.getAcceleration().getMagnitude());

        TelemetryPacket fmag = new TelemetryPacket();
        fmag.put("fmag", filteredAccelMag);

        TelemetryPacket distance = new TelemetryPacket();
        distance.put("dist", dist);

        FtcDashboard.getInstance().sendTelemetryPacket(ang);
        FtcDashboard.getInstance().sendTelemetryPacket(fang);
        FtcDashboard.getInstance().sendTelemetryPacket(distance);

        FtcDashboard.getInstance().sendTelemetryPacket(mag);
        FtcDashboard.getInstance().sendTelemetryPacket(fmag);


        telemetry.update();
    }

    private void launch() {


        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        if (g1.getButton(GamepadKeys.Button.CROSS)) {
            currentLaunchState = launchState.launching;
        } else if (g1.getButton(GamepadKeys.Button.TRIANGLE)) {
            currentLaunchState = launchState.intaking;
        } else {
            currentLaunchState = launchState.idle;
        }

        switch (currentLaunchState) {

            case idle:
                ballsLaunched = 0;
                l1.set(0);
                l2.set(0);
                intake.set(0);
                transfer.set(0);
                gate.set(globals.gate.close);
                break;

            case launching:

                launchPIDF.setSetPoint(targetRPM);
                launchPower = launchPIDF.calculate(RPM);
                if (RPM < 400) {
                    l1.set(0.55);
                    l2.set(0.55);
                } else {
                    l1.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                    l2.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                }

                hood.set(MathFunctions.clamp(hoodAngle, 0, 300));
                if (launchPIDF.atSetPoint() && turretInRange) {
                    gate.set(globals.gate.open);
                    if (Objects.equals(robotLocation, "Far Zone")) {
                        intake.set(.6);
                        transfer.set(0.6);
                    } else {
                        intake.set(0.9);
                        transfer.set(0.9);
                    }
                }
                break;
            case intaking:
                intake.set(1.0);
                transfer.set(0.5);
                gate.set(globals.gate.close);
                break;
        }
    }

    private void launchCalc() {
        if (robotZone.isInside(closeLaunchZone) || robotZone.isInside(farLaunchZone)) {
            lights.set(0.5);
        } else {
            lights.set(0);
        }

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        Pose robot = new Pose(x, y);
        robotZone.setPosition(x, y);
        robotZone.setRotation(follower.getPose().getHeading());
        Pose goal = new Pose(globals.turret.goalX, globals.turret.goalY);
        if (follower.getVelocity().getMagnitude() < 6 || robotZone.isInside(farLaunchZone)) {
            currentLaunchMode = launchMode.normal;
        } else {
            currentLaunchMode = launchMode.SOTM;
        }

        switch (currentLaunchMode) {
            case SOTM:
                dist = goal.minus(robot).getAsVector().getMagnitude();

                double accelMag = follower.getAcceleration().getMagnitude();
                filteredAccelMag = globals.launcher.accelAlpha * accelMag + (1-globals.launcher.accelAlpha) * filteredAccelMag;
                double accelAngle = follower.getAcceleration().getTheta();
                filteredAccelAngle = globals.launcher.accelAlpha * accelAngle + (1-globals.launcher.accelAlpha) * filteredAccelAngle;
                Vector accel = new Vector(accelMag, accelAngle); // calculate acceleration rounded to nearest inch/s, nearest degree (in inch/s^2, rad)
                Vector velocity = follower.getVelocity().plus(new Vector(accel.getMagnitude() * globals.launcher.velTime, accel.getTheta())); // create a velocity vector by using v = u + at
                double distanceDiff = velocity.getMagnitude() * (0.0025 * dist + 0.3871); //calculate distance by using an expiremental relation of distance vs time
                Vector robotVelocity = new Vector(distanceDiff, velocity.getTheta());
                Pose newGoal = new Pose(-robotVelocity.getXComponent() + goal.getX(), -robotVelocity.getYComponent() + goal.getY());

                double newGoalAngle = Math.atan2(newGoal.getY() - y, newGoal.getX() - x);
                turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - newGoalAngle));
                dist = newGoal.minus(robot).getAsVector().getMagnitude();
                break;
            case normal:

                Pose target = goal.minus(robot);
                Vector robotToGoal = target.getAsVector();
                double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

                turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
                dist = robotToGoal.getMagnitude();

                break;
        }


        targetRPM = 19.879* dist + 1691.5;
        if (dist > 40) {
            hoodAngle = 156.75 * Math.log(dist) - 562.97;
        } else if (dist <= 40) {
            hoodAngle = 0;
        }



    }

    private void turretAim() {
        if (Math.abs(turretAng) > 150) {
            turretAng = 0;
            turretInRange = false;
        } else  {
            turretInRange = true;
        }


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result.getStaleness() < 500) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (!tags.isEmpty()) {
                    for (LLResultTypes.FiducialResult tag : tags) {
                        double distance = tag.getTargetXDegrees();
                        telemetry.addData("dist", distance);

                    }
                }
            }
        }


        double set = MathFunctions.clamp( 180 + (turretAng * 3)/2, globals.turret.offset, 360-globals.turret.offset);
        t1.set(set);
        t2.set(set + globals.turret.offset);
    }
    private double degresToTicks(double degree) {
        return (degree * 8192) / 360;
    }

    private double voltageToDegrees(double volts) {
        return ((volts) * 360) / 3.2 ;
    }

    private void drive() {
        leftX = g1.getLeftX();
        leftY = g1.getLeftY();
        if (Math.abs(leftX) < 0.2) {
            leftX = 0;
        }
        if (Math.abs(leftY) < 0.2) {
            leftY = 0;
        }

        if (g1.getButton(GamepadKeys.Button.DPAD_UP)) {

            follower.setPose(new Pose(142 - 8.5, 9, Math.toRadians(0)));
        }

        if (g1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            follower.setPose(new Pose(72, 9, Math.toRadians(90)));

        }
        if (g1.getButton(GamepadKeys.Button.DPAD_DOWN)) {

            follower.setPose(new Pose(142-135, 9, Math.toRadians(180)));

        }
        if (g1.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            follower.setPose(new Pose(142-15, 79, Math.toRadians(90)));

        }



        if (g1.getButton(GamepadKeys.Button.CIRCLE) && !prevCross1) {
            slowDrive = !slowDrive;
        } prevCross1 = g1.getButton(GamepadKeys.Button.CIRCLE);

        if (slowDrive) {
            follower.setMaxPower(0.5);
        } else {
            follower.setMaxPower(1);
        }

        follower.setTeleOpDrive(leftY, -leftX, 0.5 * (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), true);

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



    public void drawRobot(Pose pose, Style style) {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double heading = follower.getHeading();
        double radius = 9.0;


        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.circle(radius);


        Vector v = follower.getPose().getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9);
        double x1 = x + v.getXComponent() / 2, y1 = y + v.getYComponent() / 2;
        double x2 = x + v.getXComponent(), y2 = y + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
        panelsField.update();
    }
}
