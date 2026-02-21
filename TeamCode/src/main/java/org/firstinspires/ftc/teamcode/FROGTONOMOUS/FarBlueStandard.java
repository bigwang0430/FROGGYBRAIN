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
import org.firstinspires.ftc.teamcode.vars.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vars.states;

import java.util.List;

@Autonomous (name = "Far Blue STANDARD")
public class FarBlueStandard extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    private int shootnum = 0;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //PATHS/////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void buildpath() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.000, 9.00),

                                new Pose(10.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 9.00),

                                new Pose(44.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 9.00),

                                new Pose(10.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 9.00),

                                new Pose(44.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 9.00),

                                new Pose(10.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 9.00),

                                new Pose(44.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 9.00),

                                new Pose(10.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 9.00),

                                new Pose(44.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 9.00),

                                new Pose(10.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 9.00),

                                new Pose(44.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 9.00),

                                new Pose(10.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 9.00),

                                new Pose(44.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 9.00),

                                new Pose(10.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 9.00),

                                new Pose(44.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(44.000, 9.00),

                                new Pose(30.000, 9.00)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    //SUBSYSTEMS///////////////////////////////////////////////////////////////////////////////////////////////////

    public class everythingsubsys extends SubsystemBase {
        private ServoEx hood, gate;
        private CRServo t1, t2;
        private double dist, turretAng = 0, targetRPM, hoodAngle, RPM, lastTime, lastPosition, previousRPM, launchPower, turretPower, turretPos;
        private PIDController turretPIDF = new PIDController(globals.turret.pFarAuto, globals.turret.i, globals.turret.d);
        private MotorEx l1, l2, intake, transfer;
        private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        private final PolygonZone farLaunchZone = new PolygonZone(new Point(45, 0), new Point(72, 24), new Point(96, 0));
        private final PolygonZone robotZone = new PolygonZone(18, 18);
        private AnalogInput turretEncoder;
        private double turretZeroOffset;
        public everythingsubsys(HardwareMap hardwareMap){
            t1 = new CRServoEx(hardwareMap, "t1");
            t2 = new CRServoEx(hardwareMap, "t2");
            t2.setInverted(true);
            t1.setInverted(true);
            turretPIDF.setTolerance(100);
            t1.set(0.01);
            t2.set(0.01);


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

            hood = new ServoEx(hardwareMap, "hood", 300);

            launchPIDF.setTolerance(50);
            launchPIDF.setPID(globals.launcher.p, globals.launcher.i, globals.launcher.d);

            turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
            telemetryData.addData("voltage", turretEncoder.getVoltage());
            turretZeroOffset =  degresToTicks(voltageToDegrees(turretEncoder.getVoltage() - 1.6)) * 2;
            telemetryData.update();
        }

        public void intaking(){
            intake.set(1);
            transfer.set(0.5);
            gate.set(globals.gate.close);
        }
        public void launchcalc() {
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            Pose robot = new Pose(x, y);
            Pose goal = new Pose(globals.turret.goalX, globals.turret.goalY);//TODO
            robotZone.setPosition(x, y);
            robotZone.setRotation(follower.getPose().getHeading());

            Pose target = goal.minus(robot);
            Vector robotToGoal = target.getAsVector();
            double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

            turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
            dist = robotToGoal.getMagnitude();

            targetRPM = (13.09 * dist + 2164.9) * 1.005;
            hoodAngle = 211.5;

            if (Math.abs(turretAng) > 120) {
                turretAng = 0;
            }

            double turretTarget = degresToTicks((turretAng * 3)) + turretZeroOffset;
            turretPIDF.setSetPoint(turretTarget);
            if (Math.abs(turretPIDF.getPositionError()) > 1000) {
                turretPIDF.setP(globals.turret.pFarAuto);
            } else {
                turretPIDF.setP(globals.turret.pCloseAuto);
            }
            telemetry.addData("err", Math.abs(turretPIDF.getPositionError()));
            turretPower = MathFunctions.clamp(turretPIDF.calculate(intake.getCurrentPosition()), -1, 1);
                t1.set(setTurret(turretPower));
                t2.set(setTurret(turretPower));
        }
        public void intakedone(){
            if (shootnum ==1){
                intake.set(1);
            } else {
                intake.set(0);
                transfer.set(0);
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
            if (RPM < 400) {
                l1.set(0.5);
                l2.set(0.5);
            } else {
                l1.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
                l2.set(launchPower + globals.launcher.kv * targetRPM + globals.launcher.ks);
            }

            if (launchPIDF.atSetPoint() && robotZone.isInside(farLaunchZone) && !follower.isBusy() && turretPIDF.atSetPoint()) {//TODO MAYBE REMOVE TURRETPIDF
                gate.set(globals.gate.open);
                intake.set(0.7);//57
                transfer.set(0.7);//57
            }
            hood.set(MathFunctions.clamp(hoodAngle, 40, 211.5));

        }
        public void launchend(){
            l1.set(0.2);
            l2.set(0.2);
            intake.set(0);
            transfer.set(0);
        }

        @Override
        public void periodic(){
            RPM();
            launchcalc();
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

    //INIT///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void initialize() {
        everythingsubsys everythingsubsystem = new everythingsubsys(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(40, 9, Math.toRadians(180)));//todo

        buildpath();

        register(everythingsubsystem);


        froggyroute = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggylaunch(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path1),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3500),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path2)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path3),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3500),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path4)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path5),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3500),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path6)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path7),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3500),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path8)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path9),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3500),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path10)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path11),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3500),
                        new froggylaunch(everythingsubsystem),
                        new FollowPathCommand(follower, Path12)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path13),
                        new froggyeat(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path14)
        );
    }

    //RUN//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void run() {
        if (!scheduled) {
            schedule(froggyroute);
            scheduled = true;
        }
        states.autoEndPose = follower.getPose();
        super.run();
        follower.update();
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}