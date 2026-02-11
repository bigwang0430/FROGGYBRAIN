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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

@TeleOp(name = "Blue")
public class Blue extends OpMode {
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(18, 18);
    
    private Motor l1, l2, intake, transfer;
    private ServoEx hood, turret, gate, tiltl, tiltr;
    
    private GamepadEx g1, g2;
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
    
    
    private double lastTime, launchPower, RPM, previousRPM, dist, turretAng, targetRPM, hoodAngle, leftY, leftX;
    private double turretPos = 180F;
    private int lastPosition;
    private boolean prevCross1, prevOptions2;
    private boolean autoAim = true, dip1 = false, dip2 = true;
    private boolean slowDrive = false;
    private int ballsLaunched = 0;

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style(
            "", "#3F51B5", 1
    );
    
    @Override
    public void init() {
        timer.startTime();
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        turret = new ServoEx(hardwareMap, "t2", 360, AngleUnit.DEGREES);
        turret.setInverted(true);
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
        transfer = new Motor(hardwareMap, "transfer");
        intake.setRunMode(Motor.RunMode.RawPower);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer.setInverted(true);
        intake.setInverted(false);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hardwareMap, "hood", 300, AngleUnit.DEGREES);

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        launchPIDF.setTolerance(50);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setStartingPose(new Pose(16, 75, Math.PI/2)); //TEMPORARY


        while (timer.seconds() < 0.5) {
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        follower.update();
        RPM();
        launchCalc();
        drive();
        launch();
        drawRobot(follower.getPose(), robotLook);

        telemetry.addData("loop time", timer.seconds());
        timer.reset();
        telemetry();
    }

    private void telemetry() {
        telemetry.addData("autoaim", autoAim);
        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", targetRPM);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
        telemetry.update();
    }

    private void launch() {
        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        if (g2.getButton(GamepadKeys.Button.CROSS)) {
            currentLaunchState = launchState.launching;
        } else if (g2.getButton(GamepadKeys.Button.TRIANGLE) || g1.getButton(GamepadKeys.Button.TRIANGLE)) {
            currentLaunchState = launchState.intaking;
        } else {
            currentLaunchState = launchState.idle;
        }

        switch (currentLaunchState) {

            case idle:
                ballsLaunched = 0;
                dip1 = false;
                dip2 = true;
                l1.set(0);
                l2.set(0);
                intake.set(0);
                transfer.set(0);
                gate.set(globals.gate.close);
                break;

            case launching:
                hood.set(MathFunctions.clamp(hoodAngle, 40, 240));
                launchPIDF.setSetPoint(targetRPM);
                launchPower = launchPIDF.calculate(RPM);
                if (RPM < 600) {
                    l1.set(0.43);
                    l2.set(0.43);
                } else {
                    l1.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                    l2.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                }

                boolean RPMdip = previousRPM - RPM > 300;
                if (RPMdip && !dip1) {
                    ballsLaunched++;
                    dip1 = true;
                    dip2 = false;
                } else if (RPMdip && !dip2) {
                    ballsLaunched++;
                    dip2 = true;
                }


                if (ballsLaunched == 0) {
                    hood.set(MathFunctions.clamp(hoodAngle, 40, 240));
                } else if (ballsLaunched == 1) {
                    hood.set(MathFunctions.clamp(hoodAngle - globals.launcher.airSortThreshold, 40, 240));
                } else {
                    hood.set(MathFunctions.clamp(hoodAngle, 40, 240));
                }

                if (launchPIDF.atSetPoint() && !robotLocation.equals("No Zone")) {
                    gate.set(globals.gate.open);
                    if (Objects.equals(robotLocation, "Far Zone")) {
                        intake.set(.65);
                        transfer.set(0.65);
                    } else {
                        intake.set(0.85);
                        transfer.set(0.85);
                    }
                }
                break;
            case intaking:
                intake.set(0.7);
                transfer.set(0.2);
                gate.set(globals.gate.close);
                break;
        }
    }

    private void launchCalc() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        Pose robot = new Pose(x, y);
        robotZone.setPosition(x, y);
        robotZone.setRotation(follower.getPose().getHeading());
        Pose goal = new Pose(globals.turret.goalX, globals.turret.goalY);
        if (follower.getVelocity().getMagnitude() < 6 || "Far Zone".equals(robotLocation)) {
            currentLaunchMode = launchMode.normal;
        } else {
            currentLaunchMode = launchMode.SOTM;
        }

        switch (currentLaunchMode) {
            case SOTM:
                dist = goal.minus(robot).getAsVector().getMagnitude();

                double accelMag = Math.floor(follower.getAcceleration().getMagnitude());
                double accelAngle = Math.toRadians(Math.floor(Math.toDegrees(follower.getAcceleration().getTheta())));
                Vector accel = new Vector(accelMag, accelAngle); // calculate acceleration rounded to nearest inch/s, nearest degree (in inch/s^2, rad)

                Vector velocity = follower.getVelocity().plus(new Vector(accel.getMagnitude() * globals.launcher.velTime, accel.getTheta())); // create a velocity vector by using v = u + at

                double distanceDiff = velocity.getMagnitude() * (0.0025 * dist + 0.3871);
                Vector robotVelocity = new Vector (distanceDiff, velocity.getTheta());
                Pose newGoal = new Pose(-robotVelocity.getXComponent() + goal.getX(), -robotVelocity.getYComponent() + goal.getY());

                double newGoalAngle = Math.atan2(newGoal.getY() - y, newGoal.getX()-x);
                turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - newGoalAngle));
                dist = newGoal.minus(robot).getAsVector().getMagnitude();
                break;
            case normal:

                Pose target = goal.minus(robot);
                Vector robotToGoal = target.getAsVector();
                double goalAngle = Math.atan2(goal.getY() - y, goal.getX()-x);

                turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
                dist = robotToGoal.getMagnitude();

                break;
        }

        if (robotZone.isInside(closeLaunchZone)) {
            targetRPM = 2414.2 * Math.exp(0.0036 * dist);
            if (dist < 35) {
                hoodAngle = 40;
            } else {
                hoodAngle = 147.8 * Math.log(dist) - 441.52;
            }
            robotLocation = "Close Zone";
        } else if (robotZone.isInside(farLaunchZone)) {
            targetRPM = 13.09 * dist + 2164.9;
            hoodAngle = 240;
            robotLocation = "Far Zone";
        } else {
            targetRPM = 3000;
            robotLocation = "No Zone";
        }

        if (Math.abs(turretAng) > 130) {
            turretAng = 0;
        }

        if (g2.getButton(GamepadKeys.Button.OPTIONS) && !prevOptions2) {
            autoAim = !autoAim;
        } prevOptions2 = g2.getButton(GamepadKeys.Button.OPTIONS);
        if (autoAim) {
            turret.set(setTurret(turretAng));
            turretPos = setTurret(turretAng);
        } else {
            turretPos -= 1.5 * (g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            turret.set(MathFunctions.clamp(turretPos, 50, 310));
        }
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

        if (g1.getButton(GamepadKeys.Button.DPAD_UP )) {
            //follower.setStartingPose(new Pose(136, 8, Math.toRadians(90)));
            follower.setPose(new Pose(14.731707317073187, 79.64878048780488, Math.PI/2));
        }

        if (g1.getButton(GamepadKeys.Button.CROSS) && !prevCross1) {
            slowDrive = !slowDrive;
        } prevCross1 = g1.getButton(GamepadKeys.Button.CROSS);

        if (slowDrive) {
            follower.setMaxPower(0.5);
        } else {
            follower.setMaxPower(0.9);
        }

        follower.setTeleOpDrive(leftY, -leftX, g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true);

    }

    public void RPM() {
        double currentTime = getRuntime();
        int currentPosition = l1.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        double deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.02) {
            previousRPM = RPM;
            double revs = deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }

    private double setTurret(double ang) {

        ang = 180 - ((ang * 3) / 2) - globals.turret.turretOffset;
        return ang;
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
    }
}
