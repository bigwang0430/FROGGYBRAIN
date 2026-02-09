package org.firstinspires.ftc.teamcode.FROGTONOMOUS;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous
public class FROGTONOMOUS extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //PATHS/////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //SUBSYSTEMS///////////////////////////////////////////////////////////////////////////////////////////////////
    public class intakesubsys extends SubsystemBase {
        private MotorEx intake, transfer;
        public intakesubsys(HardwareMap hardwareMap){

        }
    }

    public class outtakesubsys extends SubsystemBase {
        private ServoEx tur1, hood;
        public outtakesubsys(HardwareMap hardwareMap){

        }

        @Override
        public void periodic(){

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

            if (txPx < 170 && txPx > 150) {
                strafe(0);
                alignedticks++;
                if (alignedticks >= 5) {
                    hunted = true;
                }
            } else {
                alignedticks = 0;
                if (txPx > 170) strafe(+1);
                else if (txPx < 150) strafe(-1);
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

    //INIT
    @Override
    public void initialize() {
//        launchsubsys launchsubsystem = new launchsubsys(hardwareMap);
//        visionsubsys visionsubsystem = new visionsubsys(hardwareMap);
//        intakesubsys intakesubsystem = new intakesubsys(hardwareMap);
//
//        timer.start();
//
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        sleep(1000);
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(24, 130, Math.toRadians(-115)));//todo
//        buildpath();
//
//        register(launchsubsystem);
//        register(intakesubsystem);


        SequentialCommandGroup froggyroute = new SequentialCommandGroup(

        );

        schedule(froggyroute);

    }

    //RUN
    @Override
    public void run() {
        super.run();
        if (follower.isBusy()) follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();

    }
}
