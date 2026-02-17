package org.firstinspires.ftc.teamcode.FROGTONOMOUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.tele.Blue;
import org.firstinspires.ftc.teamcode.vars.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous
public class FROGTONOMOUSCLOSEBLUE extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private ElapsedTime timer = new ElapsedTime();
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    private int shootnum = 0;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //PATHS/////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void buildpath(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.800, 124.900),

                                new Pose(47.063, 83.590)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(47.063, 83.590),

                                new Pose(42.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 60.000),

                                new Pose(23.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 60.000),

                                new Pose(52.000, 78.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.000, 78.000),

                                new Pose(12.448, 59.776)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(147))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.448, 59.776),

                                new Pose(54.000, 81.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(147), Math.toRadians(132))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.000, 81.000),

                                new Pose(12.448, 59.776)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(147))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.448, 59.776),

                                new Pose(54.000, 81.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(147), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(54.000, 81.000),

                                new Pose(42.500, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.500, 84.000),

                                new Pose(25.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.000, 84.000),

                                new Pose(45.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 84.000),

                                new Pose(42.500, 35.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.500, 35.500),

                                new Pose(23.000, 35.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 35.500),

                                new Pose(58.000, 104.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(147))

                .build();
    }

    //SUBSYSTEMS///////////////////////////////////////////////////////////////////////////////////////////////////

    public class everythingsubsys extends SubsystemBase {
        private ServoEx hood, gate;
        private CRServo t1, t2;
        private double dist, turretAng = 0, targetRPM, hoodAngle, RPM, lastTime, lastPosition, previousRPM, launchPower, turretPower, turretPos;
        private PIDController turretPIDF = new PIDController(globals.turret.pFar, globals.turret.i, globals.turret.d);
        private int ballsLaunched = 0;
        private MotorEx l1, l2, intake, transfer;
        private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
        private final PolygonZone robotZone = new PolygonZone(18, 18);
        private boolean dip1 = false, dip2 = false;
        private AnalogInput turretEncoder;
        private double turretZeroOffset;
        public everythingsubsys(HardwareMap hardwareMap){
            t1 = new CRServoEx(hardwareMap, "t1");
            t2 = new CRServoEx(hardwareMap, "t2");
            t2.setInverted(true);
            t1.setInverted(true);
            turretPIDF.setTolerance(67);
            t1.set(0.001);
            t2.set(0.001);


            intake = new MotorEx(hardwareMap, "intake");
            transfer = new MotorEx(hardwareMap, "transfer");
            intake.setRunMode(Motor.RunMode.RawPower);
            transfer.setRunMode(Motor.RunMode.RawPower);
            transfer.setInverted(true);
            intake.setInverted(false);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            intake.stopAndResetEncoder();


            l1 = new MotorEx(hardwareMap, "l1", 28, 6000);
            l2 = new MotorEx(hardwareMap, "l2", 28, 6000);
            l1.setRunMode(Motor.RunMode.RawPower);
            l2.setRunMode(Motor.RunMode.RawPower);
            l2.setInverted(true);
            l1.setInverted(false);
            l1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            l2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

            gate = new ServoEx(hardwareMap, "gate");
            gate.set(globals.gate.close);

            hood = new ServoEx(hardwareMap, "hood", 300, AngleUnit.DEGREES);

            launchPIDF.setTolerance(300);
            launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);

            turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

            turretZeroOffset =  degresToTicks(voltageToDegrees(turretEncoder.getVoltage() - 1.6)) * 2;

        }

        public void intaking(){
            intake.set(1);
            transfer.set(0.5);
            gate.set(globals.gate.close);
        }
        public void intakedone(){
            intake.set(0);
            transfer.set(0);
        }
        private double degresToTicks(double degree) {
            return (degree * 8192) / 360;
        }
        private double voltageToDegrees(double volts) {
            return ((volts) * 360) / 3.2 ;
        }
        public void launchcalc() {
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            Pose robot = new Pose(x, y);
            Pose goal = new Pose(globals.turret.goalX, globals.turret.goalY);
            robotZone.setPosition(x, y);
            robotZone.setRotation(follower.getPose().getHeading());

            if (shootnum == 4) {
                goal = new Pose(0, 134);
            }


                    Pose target = goal.minus(robot);
                    Vector robotToGoal = target.getAsVector();
                    double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

                    turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
                    dist = robotToGoal.getMagnitude();

            if (robotZone.isInside(closeLaunchZone)) {
                targetRPM = 2414.2 * Math.exp(0.0036 * dist);
                if (dist < 35) {
                    hoodAngle = 40;
                } else {
                    hoodAngle = 147.8 * Math.log(dist) - 441.52;
                }
            } else {
                targetRPM = 3000;
            }

            if (Math.abs(turretAng) > 120) {
                turretAng = 0;
            }

            double turretTarget = degresToTicks((turretAng * 3)) + turretZeroOffset;
            turretPIDF.setSetPoint(turretTarget);


                if (Math.abs(turretPIDF.getPositionError()) > 1000) {
                    turretPIDF.setP(globals.turret.pFar);
                } else {
                    turretPIDF.setP(globals.turret.pClose);
                }
                telemetry.addData("err", Math.abs(turretPIDF.getPositionError()));
                turretPower = MathFunctions.clamp(turretPIDF.calculate(intake.getCurrentPosition()), -1, 1);
                if (!turretPIDF.atSetPoint()) {
                    t1.set(setTurret(turretPower));
                    t2.set(setTurret(turretPower));
                } else {
                    t1.set(0);
                    t2.set(0);
                }

        }
        private double setTurret(double power) {
            return Math.signum(power) * (Math.abs(power) + globals.turret.ks);
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
        public void launch(){
            launchPIDF.setSetPoint(targetRPM);
            launchPower = launchPIDF.calculate(RPM);
            if (RPM < 300) {
                l1.set(0.35);
                l2.set(0.35);
            } else {
                l1.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                l2.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
            }

            if (launchPIDF.atSetPoint() && robotZone.isInside(closeLaunchZone) && !follower.isBusy()) {
                gate.set(globals.gate.open);
                intake.set(1);
                transfer.set(1);

                boolean RPMdip = previousRPM - RPM > 100;
                if (RPMdip && !dip1) {
                    ballsLaunched++;
                    dip1 = true;
                    dip2 = false;
                } else if (RPMdip && !dip2 && dip1) {
                    ballsLaunched++;
                    dip2 = true;
                }
            }
            hood.set(MathFunctions.clamp(hoodAngle, 40, 211.5));

        }
        public void launchend(){
            l1.set(0.2);
            l2.set(0.2);
            intake.set(0);
            transfer.set(0);
            dip1 = false;
            dip2 = false;
            ballsLaunched = 0;
            shootnum++;
        }

        @Override
        public void periodic(){
            RPM();
            launchcalc();
        }
    }

    public class visionsubsys extends SubsystemBase {
        private Limelight3A limelight;
        private double numspeed = 0.4;
        private Motor fl, bl, fr, br;
        public boolean hunted;
        private int alignedticks = 0;
        private double xEst = 0, yEst = 0, hEst = 0;   // odometry estimate
        private double pX = globals.kalman.pX0, pY = globals.kalman.pY0, pH = globals.pHg; // odometry error
        private static final double M_TO_IN = 39.37007874015748;
        private Pose fusedPose = new Pose(0, 0, 0);
        private Pose odoPose = new Pose(0, 0, 0);
        private double zH=0.0;
        public visionsubsys(HardwareMap hardwareMap){
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(50);
            limelight.start();

            FtcDashboard.getInstance().startCameraStream(limelight, 30);

            fl = new Motor(hardwareMap, "fl");
            fl.setInverted(true);
            fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            fl.setRunMode(Motor.RunMode.RawPower);

            bl = new Motor(hardwareMap, "bl");
            bl.setInverted(true);
            bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            bl.setRunMode(Motor.RunMode.RawPower);

            fr = new Motor(hardwareMap, "fr");
            fr.setInverted(false);
            fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            fr.setRunMode(Motor.RunMode.RawPower);

            br = new Motor(hardwareMap, "br");
            br.setInverted(false);
            br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            br.setRunMode(Motor.RunMode.RawPower);

        }

        public void pipeline(int num){
            limelight.pipelineSwitch(num);
        }
        public void relocalize() {
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

                    double midpointH = 0.15;   // radians (~8.5°)
                    double steepnessH = 15.0;
                    double kH = 1.0 / (1.0 + Math.exp(-steepnessH * (Math.abs(headingError) - midpointH)));

                    hEst = hPred + kH * headingError;
                    while (hEst > Math.PI) hEst -= 2.0 * Math.PI;
                    while (hEst < -Math.PI) hEst += 2.0 * Math.PI;
                    pH = (1.0 - kH) * pHPred;



                    double errorMag = Math.sqrt(Math.pow(zX - xPred, 2) + Math.pow(zY - yPred, 2));
                    double midpoint = 6.0;
                    double steepness = 0.8;
                    double kX = 1.0 / (1.0 + Math.exp(-steepness * (errorMag - midpoint)));
                    double kY = kX;

                    xEst = xPred + kX * (zX - xPred);
                    yEst = yPred + kY * (zY - yPred);

                    xEst = xPred + kX * (zX - xPred);
                    yEst = yPred + kY * (zY - yPred);

                    pX = (1.0 - kX) * pXPred;
                    pY = (1.0 - kY) * pYPred;
                }
            }
            fusedPose = new Pose(xEst, yEst, hEst);

            follower.setPose(fusedPose);
            follower.update();
        }

        public void balltracking() {
            pipeline(2);
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid() || result.getStaleness() >= 500) {
                strafe(0);
                return;
            }

            List<LLResultTypes.ColorResult> balls = result.getColorResults();
            if (balls == null || balls.isEmpty()) {
                strafe(0);
                return;
            }

            LLResultTypes.ColorResult ball = balls.get(0);
            double txPx = ball.getTargetXPixels();

            if (txPx < 130 && txPx > 110) {
                strafe(0);
                alignedticks++;
                if (alignedticks >= 5) {
                    hunted = true;
                }
            } else {
                alignedticks = 0;
                if (txPx > 130) strafe(+1);
                else if (txPx < 110) strafe(-1);
            }
        }

        public void strafe(int num) {
            fl.set(num*numspeed);
            fr.set(-num*numspeed);
            bl.set(-num*numspeed);
            br.set(num*numspeed);
        }

        public boolean done() {
            if (hunted){
                return  true;
            }
            return false;
        }
    }

    //COMMANDS/////////////////////////////////////////////////////////////////////////////////////////////////////

    public static class froggylaunch extends CommandBase {
        private final everythingsubsys everythingsubsystem;

        public froggylaunch(everythingsubsys everythingsubsystem){
            this.everythingsubsystem = everythingsubsystem;
            addRequirements(everythingsubsystem);
        }

        @Override
        public void execute(){
            everythingsubsystem.launch();
        }

        @Override
        public void end(boolean interrupted){
            everythingsubsystem.launchend();
        }
    }
    public static class froggyeat extends  CommandBase {
        private final everythingsubsys everythingsubsystem;

        public froggyeat(everythingsubsys everythingsubsystem){
            this.everythingsubsystem = everythingsubsystem;
            addRequirements(everythingsubsystem);
        }

        @Override
        public void initialize(){
            everythingsubsystem.intaking();
        }

        @Override
        public void end(boolean interrupted){
            everythingsubsystem.intakedone();
        }

    }
    public static class froggyhunting extends CommandBase {
        private final visionsubsys visionsubsystem;

        public froggyhunting(visionsubsys visionsubsystem){
            this.visionsubsystem = visionsubsystem;
            addRequirements(visionsubsystem);
        }

        @Override
        public void initialize(){
            visionsubsystem.pipeline(2);
        }

        @Override
        public void execute() {
            visionsubsystem.balltracking();
        }

        @Override
        public boolean isFinished(){
            return visionsubsystem.done();
        }
    }
    public static class froggyaim extends CommandBase {
        private final visionsubsys visionsubsystem;

        public froggyaim(visionsubsys visionsubsystem){
            this.visionsubsystem = visionsubsystem;
            addRequirements(visionsubsystem);
        }

        @Override
        public void initialize(){
            visionsubsystem.pipeline(0);
        }

        @Override
        public void execute() {
            visionsubsystem.relocalize();
        }
    }


    //INIT///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void initialize() {
        visionsubsys visionsubsystem = new visionsubsys(hardwareMap);
        everythingsubsys everythingsubsystem = new everythingsubsys(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.8, 124.9, Math.toRadians(144)));//todo

        buildpath();

        register(everythingsubsystem);


        froggyroute = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path1)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(500),
                        new froggyaim(visionsubsystem)
                ),
                new FollowPathCommand(follower, Path2),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path3),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path4)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path5),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(1000),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3000),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path6)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path7),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(1000),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3000),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path8)
                ),
                new FollowPathCommand(follower, Path9),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path10),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(2000),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path11)
                ),
                new FollowPathCommand(follower, Path12),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path13),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path14)
                )
        )
                ;
    }

    //RUN//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void run() {
        if (!scheduled) {
            schedule(froggyroute);
            scheduled = true;
        }
        super.run();
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();

    }
}

