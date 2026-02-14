package org.firstinspires.ftc.teamcode.FROGTONOMOUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vars.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous
public class FROGTONOMOUSFARBLUE extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private ElapsedTime timer = new ElapsedTime();
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //PATHS/////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void buildpath() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 9.000),

                                new Pose(11.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 9.000),

                                new Pose(48.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(48.000, 9.000),

                                new Pose(42.500, 35.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.500, 35.500),

                                new Pose(25.000, 35.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 35.500),

                                new Pose(50.000, 13.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(111))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 13.000),

                                new Pose(42.500, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(111), Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.500, 60.000),

                                new Pose(25.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 60.000),

                                new Pose(53.000, 14.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.000, 14.000),

                                new Pose(49.813, 21.831)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(112))

                .build();
    }

    //SUBSYSTEMS///////////////////////////////////////////////////////////////////////////////////////////////////

    public class everythingsubsys extends SubsystemBase {
        private ServoEx turret, hood, gate;
        private double dist, turretAng = 0, targetRPM, hoodAngle, RPM, lastTime, lastPosition, previousRPM, launchPower;
        private int ballsLaunched = 0;
        private MotorEx l1, l2, intake, transfer;
        private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);

        private boolean dip1 = false, dip2 = false;
        public everythingsubsys(HardwareMap hardwareMap){
            turret = new ServoEx(hardwareMap, "t2", 360, AngleUnit.DEGREES);
            turret.setInverted(true);
            turret.set(180);

            intake = new MotorEx(hardwareMap, "intake");
            transfer = new MotorEx(hardwareMap, "transfer");
            intake.setRunMode(Motor.RunMode.RawPower);
            transfer.setRunMode(Motor.RunMode.RawPower);
            transfer.setInverted(true);
            intake.setInverted(false);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

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

            launchPIDF.setTolerance(50);
            launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        }

        public void intaking(){
            intake.set(0.7);
            transfer.set(0.2);
            gate.set(globals.gate.close);
        }

        public void launchcalc(){
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            Pose robot = new Pose(x, y);
            Pose goal = new Pose(globals.turret.goalX, globals.turret.goalY);

            Pose target = goal.minus(robot);
            Vector robotToGoal = target.getAsVector();
            double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

            turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
            dist = robotToGoal.getMagnitude();

            targetRPM = 13.09 * dist + 2164.9;
            hoodAngle = 240;

            if (Math.abs(turretAng) > 115) {
                turretAng = 0;
            }

            turret.set(setTurret(turretAng));
        }
        private double setTurret(double hello) {
            hello = 180 - ((hello * 3) / 2) - globals.turret.turretOffset;
            return hello;
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
            if (RPM < 600) {
                l1.set(0.35);
                l2.set(0.35);
            } else {
                l1.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                l2.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
            }

            if (launchPIDF.atSetPoint()) {
                gate.set(globals.gate.open);
                intake.set(0.65);
                transfer.set(0.65);
            }

            boolean RPMdip = previousRPM - RPM > 300;
            if (RPMdip && !dip1) {
                ballsLaunched++;
                dip1 = true;
                dip2 = false;
            } else if (RPMdip && !dip2 && dip1) {
                ballsLaunched++;
                dip2 = true;
            }
        }

            public void launchend(){
                l1.set(0.2);
                l2.set(0.2);
                intake.set(0);
                dip1 = false;
                dip2 = false;
                ballsLaunched = 0;
            }
            public boolean launched(){
                if (ballsLaunched >= 2){
                    return true;
                }
                return false;
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

        public void balltracking() {
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

    //INIT///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void initialize() {
        //visionsubsys visionsubsystem = new visionsubsys(hardwareMap);
        everythingsubsys everythingsubsystem = new everythingsubsys(hardwareMap);
//        timer.startTime();
//        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.resetPosAndIMU();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(45, 9, Math.toRadians(180)));//todo

        buildpath();

        register(everythingsubsystem);

        SequentialCommandGroup froggyroute = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitCommand(5000),
                        new froggylaunch(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyeat(everythingsubsystem),
                        new FollowPathCommand(follower, Path1)
                ),
                new FollowPathCommand(follower, Path2),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggylaunch(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path3),
                new WaitCommand(2000),
                new FollowPathCommand(follower, Path4),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggylaunch(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path5),
                new WaitCommand(2000),
                new FollowPathCommand(follower, Path6),
                new WaitCommand(2000),
                new FollowPathCommand(follower, Path7),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggylaunch(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path8)
        );
        schedule(froggyroute);
    }

    //RUN//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void run() {
        super.run();
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();

    }
}
