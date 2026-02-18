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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.SquIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vars.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vars.states;

import java.sql.SQLData;
import java.util.Objects;

@TeleOp(name = "Blue")
public class Blue extends OpMode {
    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone robotZone = new PolygonZone(16, 16);
    
    private Motor l1, l2, intake, transfer;
    private ServoEx hood, gate, tiltl, tiltr;
    private CRServoEx t1, t2;
    private PIDController turretPIDF = new PIDController(globals.turret.pFar, globals.turret.i, globals.turret.d);
    private AnalogInput turretEncoder;
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
    
    
    private double lastTime, launchPower, RPM, previousRPM, dist, turretAng, targetRPM, hoodAngle, leftY, leftX, turretPower;
    private double turretPos = 0F;
    private int lastPosition;
    private boolean prevCross1, prevOptions2;
    private boolean autoAim = true, dip1 = false, dip2 = true;
    private boolean slowDrive = false;
    private boolean turretZeroed = false, turretInRange;
    private int ballsLaunched = 0;
    private double turretZeroOffset;

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook = new Style(
            "", "#3F51B5", 1
    );

    private Limelight3A limelight;

    private double xEst = 0, yEst = 0, hEst = 0;   // odometry estimate
    private double pX = globals.kalman.pX0, pY = globals.kalman.pY0, pH = globals.pHg; // odometry error

    private static final double M_TO_IN = 39.37007874015748;

    private Pose fusedPose = new Pose(0, 0, 0);
    private Pose odoPose = new Pose(0, 0, 0);
    private double zH=0.0;

    private boolean zapLeon = false, leftBumper = false, rightBumper = false;
    private ElapsedTime relocTimer = new ElapsedTime();
    private boolean relocReady = false;
    private double initialTurretOffset = 0F;
    @Override
    public void init() {
        relocTimer.startTime();
        timer.startTime();
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        t1 = new CRServoEx(hardwareMap, "t1");
        t2 = new CRServoEx(hardwareMap, "t2");
        t2.setInverted(true);
        t1.setInverted(true);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        turretPIDF.setTolerance(67);
        t1.set(0.001);
        t2.set(0.001);
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


        transfer = new Motor(hardwareMap, "transfer");
        intake.setRunMode(Motor.RunMode.RawPower);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer.setInverted(true);
        intake.setInverted(false);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        hood = new ServoEx(hardwareMap, "hood", 300);

        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        launchPIDF.setTolerance(50);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setStartingPose(states.autoEndPose); //TEMPORARY


        while (timer.seconds() < 1) {
            telemetry.addData("timer", timer.seconds());
            telemetry.update();
        }

        timer.reset();
        timer.startTime();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(0);

        // Initialize estimate to odometry so don't start at 0,0,0
        Pose p = follower.getPose();
        if (p != null) {
            xEst = p.getX();
            yEst = p.getY();
            hEst = p.getHeading();
            fusedPose = new Pose(xEst, yEst, hEst);
            odoPose = p;
        }


        turretZeroOffset = Math.abs(turretEncoder.getVoltage()) < 0.005 ? 0 : (degresToTicks(voltageToDegrees(turretEncoder.getVoltage() - 1.6)) * 2) + globals.turret.turretOffset;

    }

    @Override
    public void loop() {


        follower.update();
        RPM();
        launchCalc();
        drive();
        launch();
        drawRobot(follower.getPose(), robotLook);
        updateKalman();


        telemetry();
    }

    private void telemetry() {
        telemetry.addData("autoaim", autoAim);
        telemetry.addData("robotLocation", robotLocation);


        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", targetRPM);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);

        telemetry.addData("terrr", turretZeroed);
        telemetry.addData("pos", intake.getCurrentPosition());
        telemetry.addData("tare", turretEncoder.getVoltage());
        telemetry.addData("turretAng", turretAng);

        telemetry.update();
    }

    private void launch() {
        launchPIDF.setSetPoint(targetRPM);

        launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        if (g2.getButton(GamepadKeys.Button.CROSS)) {


            launchPower = launchPIDF.calculate(RPM);
            if (RPM < 400) {
                l1.set(0.65);
                l2.set(0.65);
            } else {
                l1.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                l2.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
            }
            hood.set(MathFunctions.clamp(hoodAngle, 40, 211.5));


        }
        if (g2.getButton(GamepadKeys.Button.TRIANGLE) || g1.getButton(GamepadKeys.Button.TRIANGLE)) {
            currentLaunchState = launchState.intaking;
        } else if (g2.getButton(GamepadKeys.Button.DPAD_UP) && launchPIDF.atSetPoint()) {
            currentLaunchState = launchState.launching;
        } else {
            currentLaunchState = launchState.idle;
        }

        switch (currentLaunchState) {

            case idle:
                l1.set(0);
                l2.set(0);
                intake.set(0);
                transfer.set(0);
                gate.set(globals.gate.close);
                break;

            case launching:
                if (launchPIDF.atSetPoint() && !robotLocation.equals("No Zone") && turretInRange) {
                    gate.set(globals.gate.open);
                    if (Objects.equals(robotLocation, "Far Zone")) {
                        intake.set(.7);
                        transfer.set(0.7);
                    } else {
                        intake.set(1);
                        transfer.set(1);
                    }
                } else if (!robotZone.isInside(closeLaunchZone) && !robotZone.isInside(farLaunchZone)) {
                    intake.set(0);
                    transfer.set(0);
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


            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            Pose robot = new Pose(x, y);
            robotZone.setPosition(x, y);
            robotZone.setRotation(follower.getPose().getHeading());
            Pose goal = new Pose(globals.turret.goalX, globals.turret.goalY);
            if (follower.getVelocity().getMagnitude() < 6 ) {
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

                    Vector velocity = follower.getVelocity().plus(
                                new Vector(accel.getMagnitude()
                                * globals.launcher.velTime, accel.getTheta())); // create a velocity vector by using v = u + at

                    double distanceDiff = velocity.getMagnitude() * (0.0025 * dist + 0.3871);
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

            if ((robotZone.isInside(closeLaunchZone) || robotZone.isInside(farLaunchZone)) && !zapLeon ) {
                gamepad1.rumble(0.6, 0.6, 400);

                zapLeon = true;
            }

            if (!robotZone.isInside(closeLaunchZone) && !robotZone.isInside(farLaunchZone) && zapLeon) {
                zapLeon = false;
            }

            telemetry.addData("zap", zapLeon);

            if (robotZone.isInside(closeLaunchZone)) {
                targetRPM = 2414.2 * Math.exp(0.0036 * dist);
                if (dist < 35) {
                    hoodAngle = 40;
                } else {
                    hoodAngle = 147.8 * Math.log(dist) - 441.52;
                }
                robotLocation = "Close Zone";
            } else if (robotZone.isInside(farLaunchZone)) {
                targetRPM = (13.09 * dist + 2164.9) * 1.01;
                hoodAngle = 211.5;
                robotLocation = "Far Zone";
            } else {
                targetRPM = (13.09 * dist + 2164.9) * 1.01;
                robotLocation = "No Zone";
            }

            if (Math.abs(turretAng) > 120) {
                turretAng = 0;
                turretInRange = false;
            } else  {
                turretInRange = true;
            }

            double turretTarget = degresToTicks((turretAng * 3)) + turretZeroOffset;
            turretPIDF.setSetPoint(turretTarget);


            if (g2.getButton(GamepadKeys.Button.OPTIONS) && !prevOptions2) {
                autoAim = !autoAim;
            }

            if (g1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && !rightBumper) {
                turretZeroOffset += 250;
            } else if (g1.getButton(GamepadKeys.Button.LEFT_BUMPER) && !leftBumper) {
                turretZeroOffset -= 250;
            }
            rightBumper = g1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
            leftBumper = g1.getButton(GamepadKeys.Button.LEFT_BUMPER);
            prevOptions2 = g2.getButton(GamepadKeys.Button.OPTIONS);


            if (autoAim) {
                if (Math.abs(turretPIDF.getPositionError()) > 1000) {
                    turretPIDF.setP(globals.turret.pFar);
                } else {
                    turretPIDF.setP(globals.turret.pClose);
                }
                turretPower = MathFunctions.clamp(turretPIDF.calculate(intake.getCurrentPosition()), -1, 1);
                if (!turretPIDF.atSetPoint()) {
                    t1.set(setTurret(turretPower));
                    t2.set(setTurret(turretPower));
                } else {
                    t1.set(0);
                    t2.set(0);
                }
                turretPos = intake.getCurrentPosition();
            } else {
                turretInRange = true;
                if (Math.abs(turretPIDF.getPositionError()) > 1000) {
                    turretPIDF.setP(globals.turret.pFar);
                } else {
                    turretPIDF.setP(globals.turret.pClose);
                }
                turretPos +=  400* (g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                turretPIDF.setSetPoint(MathFunctions.clamp(turretPos, -6000 + turretZeroOffset, 6000 + turretZeroOffset));
                if (Math.abs(turretPIDF.getPositionError()) > 1000) {
                    turretPIDF.setP(globals.turret.pFar);
                } else {
                    turretPIDF.setP(globals.turret.pClose);
                }
                turretPower = MathFunctions.clamp(turretPIDF.calculate(intake.getCurrentPosition()), -1, 1);
                t1.set(setTurret(turretPower));
                t2.set(setTurret(turretPower));


            }


        }

    private double setTurret(double power) {
        return Math.signum(power) * (Math.abs(power) + globals.turret.ks);
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

        if (g1.getButton(GamepadKeys.Button.SQUARE)) {
            //follower.setStartingPose(new Pose(136, 8, Math.toRadians(90)));
            follower.setHeading(Math.PI/2);
        }

        if (g1.getButton(GamepadKeys.Button.CIRCLE)) {
            follower.setPose(new Pose(16, 80, Math.toRadians(90)));
        }
        if (g1.getButton(GamepadKeys.Button.DPAD_UP)) {
            follower.setY(135);

        }

        if (g1.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            follower.setX(8.5);

        }
        if (g1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            follower.setY(9);

        }
        if (g1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            follower.setX(135);
        }


        if (g1.getButton(GamepadKeys.Button.CROSS) && !prevCross1) {
            slowDrive = !slowDrive;
        } prevCross1 = g1.getButton(GamepadKeys.Button.CROSS);

        if (slowDrive) {
            follower.setMaxPower(0.5);
        } else {
            follower.setMaxPower(0.9);
        }

        follower.setTeleOpDrive(leftY, -leftX, 0.75 * (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), false);

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

    public void updateKalman() {
        Pose odo = follower.getPose();
        if (odo == null) return;

        odoPose = odo;

        // Prediction
        double xPred = odo.getX(), yPred = odo.getY(), hPred = odo.getHeading();
        double pXPred = pX + globals.kalman.qX;
        double pYPred = pY + globals.kalman.qY;
        double pHPred = pH + globals.qH;



        // no measurement = keep position
        xEst = xPred; yEst = yPred; hEst = hPred;
        pX = pXPred; pY = pYPred; pH = pHPred;

        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            Pose3D bot = r.getBotpose();
            if (bot != null) {
                if (follower.getVelocity().getMagnitude() < 6) {
                    relocReady = true;
                } else {
                    relocReady = false;
                }
                double zX = 72 + (bot.getPosition().y * M_TO_IN);
                double zY = 72 - (bot.getPosition().x * M_TO_IN);
                zH = bot.getOrientation().getYaw(AngleUnit.RADIANS);
                zH-=Math.PI;
                zH += Math.PI / 2.0 ;
                zH = zH % (2.0 * Math.PI);
                if (zH < 0) zH += 2.0 * Math.PI;
                double headingError = zH - hPred;
                while (headingError > Math.PI) headingError -= 2.0 * Math.PI;
                while (headingError < -Math.PI) headingError += 2.0 * Math.PI;
                double kH = pHPred / (pHPred + globals.rH);
                hEst = hPred + kH * headingError;
                while (hEst > Math.PI) hEst -= 2.0 * Math.PI;
                while (hEst < -Math.PI) hEst += 2.0 * Math.PI;
                pH = (1.0 - kH) * pHPred;



                double kX = pXPred / (pXPred + globals.kalman.rX);
                double kY = pYPred / (pYPred + globals.kalman.rY);

                xEst = xPred + kX * (zX - xPred);
                yEst = yPred + kY * (zY - yPred);

                pX = (1.0 - kX) * pXPred;
                pY = (1.0 - kY) * pYPred;
            }

        } else {
            relocReady = false;
        }
        fusedPose = new Pose(xEst, yEst, hEst);
        if (relocReady && g2.getButton(GamepadKeys.Button.DPAD_DOWN))  {
            follower.setX(fusedPose.getX());
            follower.setY(fusedPose.getY());
            gamepad2.rumble(0.3, 0.3, 200);
        }

        telemetry.addData("timer", relocTimer);
        telemetry.addData("read", relocReady);



    }


}
