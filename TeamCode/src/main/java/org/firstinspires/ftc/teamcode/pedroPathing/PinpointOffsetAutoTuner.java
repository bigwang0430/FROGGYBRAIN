package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vars.globals;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * PinpointOffsetAutoTuner v2
 *
 * Finds and verifies optimal Pinpoint odometry pod offsets by minimising XY
 * positional drift during in-place spins. Uses coordinate-descent: tests
 * ±step around the current best offset for one axis, locks the winner, repeats
 * for the other axis, iterates until convergence, then shrinks the step size.
 *
 * ── CHANGES FROM v1 ─────────────────────────────────────────────────────────
 *  - Added VERIFY mode (DPAD_UP): runs 10 CW+CCW spin pairs with full
 *    statistics (mean, std dev, min, max, per-run breakdown) so you can
 *    confirm your offsets are dialled in.
 *  - Added multi-revolution spin option (3×360° = 1080°) for verify mode to
 *    amplify offset errors and catch sub-millimetre problems.
 *  - Post-spin settle increased to 400 ms (matching pre-spin) so the robot
 *    is truly stationary before the final position read.
 *  - Battery voltage logged at start of every test and printed in results;
 *    warns if voltage drops below 12.0V during a run.
 *  - Heading accuracy tracked: each spin reports how close to exactly 360°
 *    (or N×360°) the robot actually rotated, catching encoder/IMU drift.
 *  - Telemetry formatting improved for readability on Driver Station.
 *  - Added DPAD_DOWN quick-verify: 4-spin sanity check with pass/fail.
 *
 * ── HOW TO USE ──────────────────────────────────────────────────────────────
 *  1. Physically measure pod offsets with calipers. Set them in
 *     globals.offsets.xoff / yoff (mm). This tuner REFINES a good starting
 *     guess — it does not search from scratch.
 *
 *  2. Place robot on flat surface, perfectly still.
 *
 *  3. During init, the Pinpoint IMU calibrates. Wait for "Ready — press Play".
 *
 *  4. Press Play.
 *
 *  5. Controls:
 *       A          — reset Pinpoint pose (keep robot still!)
 *       B          — single CW spin (diagnostic)
 *       X          — single CCW spin (diagnostic)
 *       Y          — run full auto-tune (1–3 min)
 *       DPAD_UP    — VERIFY mode: 10 spin pairs with full statistics
 *       DPAD_DOWN  — quick verify: 4 spins, pass/fail verdict
 *
 *  6. After auto-tune or verify, telemetry shows results + copy-paste constants.
 *
 * ── WHAT "PERFECT" OFFSETS LOOK LIKE ────────────────────────────────────────
 *  After verify (DPAD_UP), you want:
 *    • Mean drift      < 0.15 inches    (ideally < 0.08)
 *    • Std deviation   < 0.05 inches    (consistent = good offsets, not luck)
 *    • Max drift       < 0.25 inches    (no bad outliers)
 *    • Heading error   < 3°             (IMU + encoder agreement)
 *    • CW and CCW means within 2× of each other (no directional bias)
 *
 *  If CW drift is low but CCW is high (or vice versa), the offset on one axis
 *  is slightly off — the error cancels in one direction but compounds in the
 *  other. Re-run auto-tune or manually nudge that axis by 0.5–1 mm.
 *
 * ── COORDINATE CONVENTIONS ──────────────────────────────────────────────────
 *  Pinpoint setOffsets(xMm, yMm):
 *    xMm = how far LEFT of robot centre the forward (X) pod is. Left +.
 *    yMm = how far FORWARD of robot centre the strafe (Y) pod is. Fwd +.
 *
 *  Pedro PinpointConstants (inches):
 *    forwardPodY = xMm / 25.4   (same physical distance, different name)
 *    strafePodX  = yMm / 25.4   (same physical distance, different name)
 */
@TeleOp(name = "Pinpoint Offset Auto Tuner v2", group = "Tuning")
public class PinpointOffsetAutoTuner extends LinearOpMode {

    // ── Initial guess (mm) — set to your physical measurement ────────────────
    private static final double X0_MM = globals.offsets.xoff;
    private static final double Y0_MM = globals.offsets.yoff;

    // ── Spin parameters ──────────────────────────────────────────────────────
    private static final double SPIN_POWER       = 0.28;   // lower = more repeatable
    private static final double TARGET_DEG       = 360.0;
    private static final double STOP_TOL_DEG     = 5.0;
    private static final double SPIN_TIMEOUT_S   = 25.0;

    // ── Verify mode: use multi-revolution spins to amplify tiny errors ───────
    private static final double VERIFY_TARGET_DEG = 1080.0; // 3 full revolutions
    private static final double VERIFY_TIMEOUT_S  = 75.0;
    private static final int    VERIFY_PAIRS      = 10;     // 10 CW + 10 CCW = 20 spins

    // ── Quick verify ─────────────────────────────────────────────────────────
    private static final int QUICK_VERIFY_PAIRS = 2; // 2 CW + 2 CCW = 4 spins

    // ── Scoring: spins per candidate during auto-tune ────────────────────────
    private static final int RUNS_PER_CANDIDATE = 2; // 2 CW+CCW pairs = 4 spins

    // ── Coordinate-descent step schedule (mm) ────────────────────────────────
    private static final double[] STEPS_MM = {12.0, 5.0, 2.0, 0.75};

    // ── Convergence ──────────────────────────────────────────────────────────
    private static final int MAX_PASSES_PER_STEP = 5;

    // ── Settle time (ms) — must exceed Pinpoint IMU cal time (~250 ms) ───────
    private static final int SETTLE_MS           = 400;
    private static final int POST_SPIN_SETTLE_MS = 400; // was 150 in v1 — too short

    // ── Pass/fail thresholds (inches) ────────────────────────────────────────
    private static final double DRIFT_EXCELLENT  = 0.08;
    private static final double DRIFT_GOOD       = 0.15;
    private static final double DRIFT_ACCEPTABLE = 0.25;
    // Above ACCEPTABLE = offsets need work

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor voltageSensor;

    @Override
    public void runOpMode() {
        // ── Hardware map ─────────────────────────────────────────────────────
        leftFront  = hardwareMap.get(DcMotorEx.class, "fl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        leftBack   = hardwareMap.get(DcMotorEx.class, "bl");
        rightBack  = hardwareMap.get(DcMotorEx.class, "br");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        setBrake(true);
        setDrivePower(0, 0, 0);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Grab any voltage sensor for battery monitoring
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Calibrate IMU — robot must be perfectly still
        pinpoint.resetPosAndIMU();
        sleep(SETTLE_MS);

        telemetry.addLine("═══ Pinpoint Offset Auto Tuner v2 ═══");
        telemetry.addLine("Robot must be STILL during init.");
        telemetry.addData("Initial offsets (mm)", "x=%.1f  y=%.1f", X0_MM, Y0_MM);
        telemetry.addData("Battery", "%.2fV", voltageSensor.getVoltage());
        telemetry.addLine("");
        telemetry.addLine("A = reset pose");
        telemetry.addLine("B = single CW spin");
        telemetry.addLine("X = single CCW spin");
        telemetry.addLine("Y = full auto-tune");
        telemetry.addLine("DPAD_UP = VERIFY (10 pairs, statistics)");
        telemetry.addLine("DPAD_DOWN = quick verify (4 spins)");
        telemetry.addLine("");
        telemetry.addLine("Ready — press Play");
        telemetry.update();

        waitForStart();

        double bestX = X0_MM;
        double bestY = Y0_MM;
        applyOffsets(bestX, bestY);

        boolean lastA = false, lastB = false, lastX = false, lastY = false;
        boolean lastDU = false, lastDD = false;

        while (opModeIsActive()) {
            pinpoint.update();

            boolean a  = gamepad1.a;
            boolean b  = gamepad1.b;
            boolean x  = gamepad1.x;
            boolean y  = gamepad1.y;
            boolean du = gamepad1.dpad_up;
            boolean dd = gamepad1.dpad_down;

            // ── A: reset pose ────────────────────────────────────────────────
            if (a && !lastA) {
                setDrivePower(0, 0, 0);
                sleep(SETTLE_MS);
                pinpoint.resetPosAndIMU();
                sleep(SETTLE_MS);
                telemetry.addLine(">> Pose reset.");
            }

            // ── B: single CW diagnostic ──────────────────────────────────────
            if (b && !lastB) {
                applyOffsets(bestX, bestY);
                resetPose();
                SpinResult r = doSpin(+1, TARGET_DEG, SPIN_TIMEOUT_S);
                showSpinResult("CW diagnostic (1×360°)", bestX, bestY, r);
            }

            // ── X: single CCW diagnostic ─────────────────────────────────────
            if (x && !lastX) {
                applyOffsets(bestX, bestY);
                resetPose();
                SpinResult r = doSpin(-1, TARGET_DEG, SPIN_TIMEOUT_S);
                showSpinResult("CCW diagnostic (1×360°)", bestX, bestY, r);
            }

            // ── Y: full auto-tune ────────────────────────────────────────────
            if (y && !lastY) {
                setDrivePower(0, 0, 0);
                sleep(SETTLE_MS);

                double startV = voltageSensor.getVoltage();
                telemetry.clearAll();
                telemetry.addLine("AUTO-TUNE STARTED — do not touch robot");
                telemetry.addData("Battery", "%.2fV", startV);
                telemetry.update();

                if (startV < 12.0) {
                    telemetry.addLine("!! WARNING: Battery below 12.0V — results may be unreliable");
                    telemetry.update();
                    sleep(2000);
                }

                double[] result = autoTune(bestX, bestY);
                bestX = result[0];
                bestY = result[1];
                applyOffsets(bestX, bestY);

                double endV = voltageSensor.getVoltage();
                showFinalResult(bestX, bestY, startV, endV);
            }

            // ── DPAD_UP: full verify (10 pairs, multi-revolution) ────────────
            if (du && !lastDU) {
                applyOffsets(bestX, bestY);
                runVerify(bestX, bestY, VERIFY_PAIRS, VERIFY_TARGET_DEG,
                        VERIFY_TIMEOUT_S, "FULL VERIFY (10×3rev)");
            }

            // ── DPAD_DOWN: quick verify (2 pairs, single revolution) ─────────
            if (dd && !lastDD) {
                applyOffsets(bestX, bestY);
                runVerify(bestX, bestY, QUICK_VERIFY_PAIRS, TARGET_DEG,
                        SPIN_TIMEOUT_S, "QUICK VERIFY (4×1rev)");
            }

            lastA = a; lastB = b; lastX = x; lastY = y;
            lastDU = du; lastDD = dd;

            // ── Idle telemetry ───────────────────────────────────────────────
            telemetry.addLine("═══════════════════════════════════");
            telemetry.addLine("B=CW  X=CCW  Y=auto-tune  A=reset");
            telemetry.addLine("DPAD_UP=verify  DPAD_DOWN=quick");
            telemetry.addData("Offsets (mm)", "x=%.2f  y=%.2f", bestX, bestY);
            telemetry.addData("Offsets (in)", "fwdPodY=%.4f  strPodX=%.4f",
                    bestX / 25.4, bestY / 25.4);
            telemetry.addData("Pos (in)", "X=%.3f  Y=%.3f",
                    pinpoint.getPosX(DistanceUnit.INCH),
                    pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (°)", "%.2f",
                    pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Battery", "%.2fV", voltageSensor.getVoltage());
            telemetry.update();
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  VERIFY MODE
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Runs N CW+CCW spin pairs and reports full statistics.
     * Multi-revolution spins amplify small offset errors:
     *   - 1×360° might show 0.1" drift even with slightly wrong offsets
     *   - 3×360° shows ~0.3" drift for the same error — much easier to detect
     */
    private void runVerify(double xMm, double yMm, int pairs,
                           double targetDeg, double timeoutS, String label) {
        setDrivePower(0, 0, 0);
        sleep(SETTLE_MS);

        double startVoltage = voltageSensor.getVoltage();

        telemetry.clearAll();
        telemetry.addLine(label + " — do not touch robot");
        telemetry.addData("Offsets (mm)", "x=%.2f  y=%.2f", xMm, yMm);
        telemetry.addData("Battery", "%.2fV", startVoltage);
        telemetry.addData("Spins", "%d CW + %d CCW = %d total",
                pairs, pairs, pairs * 2);
        telemetry.addData("Revolutions per spin", "%.0f", targetDeg / 360.0);
        telemetry.update();
        sleep(1000);

        List<Double> cwDrifts  = new ArrayList<>();
        List<Double> ccwDrifts = new ArrayList<>();
        List<Double> allDrifts = new ArrayList<>();
        List<Double> headingErrors = new ArrayList<>();

        boolean voltageWarning = false;

        for (int i = 0; i < pairs && opModeIsActive(); i++) {
            // ── CW spin ──────────────────────────────────────────────────
            telemetry.clearAll();
            telemetry.addData(label, "CW spin %d / %d", i + 1, pairs);
            telemetry.addData("Battery", "%.2fV", voltageSensor.getVoltage());
            telemetry.update();

            resetPose();
            SpinResult cw = doSpin(+1, targetDeg, timeoutS);
            cwDrifts.add(cw.driftMagIn);
            allDrifts.add(cw.driftMagIn);
            headingErrors.add(Math.abs(cw.rotDeg - targetDeg));

            if (voltageSensor.getVoltage() < 12.0) voltageWarning = true;

            if (!opModeIsActive()) break;

            // ── CCW spin ─────────────────────────────────────────────────
            telemetry.clearAll();
            telemetry.addData(label, "CCW spin %d / %d", i + 1, pairs);
            telemetry.addData("Battery", "%.2fV", voltageSensor.getVoltage());
            telemetry.update();

            resetPose();
            SpinResult ccw = doSpin(-1, targetDeg, timeoutS);
            ccwDrifts.add(ccw.driftMagIn);
            allDrifts.add(ccw.driftMagIn);
            headingErrors.add(Math.abs(ccw.rotDeg - targetDeg));

            if (voltageSensor.getVoltage() < 12.0) voltageWarning = true;
        }

        double endVoltage = voltageSensor.getVoltage();

        // ── Compute statistics ───────────────────────────────────────────
        Stats allStats     = computeStats(allDrifts);
        Stats cwStats      = computeStats(cwDrifts);
        Stats ccwStats     = computeStats(ccwDrifts);
        Stats headingStats = computeStats(headingErrors);

        // ── Determine verdict ────────────────────────────────────────────
        // For multi-revolution spins, scale thresholds by revolution count
        double revs = targetDeg / 360.0;
        double excellentThresh  = DRIFT_EXCELLENT  * revs;
        double goodThresh       = DRIFT_GOOD       * revs;
        double acceptableThresh = DRIFT_ACCEPTABLE * revs;

        String verdict;
        if (allStats.mean < excellentThresh && allStats.stdDev < 0.05 * revs) {
            verdict = "EXCELLENT — offsets are dialled in";
        } else if (allStats.mean < goodThresh && allStats.max < acceptableThresh) {
            verdict = "GOOD — offsets are solid for competition";
        } else if (allStats.mean < acceptableThresh) {
            verdict = "ACCEPTABLE — functional but could improve";
        } else {
            verdict = "NEEDS WORK — re-run auto-tune or check mechanicals";
        }

        // ── Check directional bias ───────────────────────────────────────
        String biasNote = "";
        if (cwStats.mean > 0.001 && ccwStats.mean > 0.001) {
            double ratio = Math.max(cwStats.mean, ccwStats.mean)
                    / Math.min(cwStats.mean, ccwStats.mean);
            if (ratio > 2.5) {
                String worse = cwStats.mean > ccwStats.mean ? "CW" : "CCW";
                biasNote = "DIRECTIONAL BIAS detected: " + worse
                        + " drift is " + String.format("%.1f", ratio)
                        + "× higher. One axis offset may be slightly off.";
            }
        }

        // ── Display results ──────────────────────────────────────────────
        telemetry.clearAll();
        telemetry.addLine("══ " + label + " RESULTS ══");
        telemetry.addData("Offsets (mm)", "x=%.2f  y=%.2f", xMm, yMm);
        telemetry.addData("Offsets (in)", "fwdPodY=%.4f  strPodX=%.4f",
                xMm / 25.4, yMm / 25.4);
        telemetry.addLine("");

        telemetry.addLine("── Drift (inches) ──");
        telemetry.addData("  Mean",    "%.4f", allStats.mean);
        telemetry.addData("  Std Dev", "%.4f", allStats.stdDev);
        telemetry.addData("  Min",     "%.4f", allStats.min);
        telemetry.addData("  Max",     "%.4f", allStats.max);
        telemetry.addLine("");

        telemetry.addLine("── CW vs CCW ──");
        telemetry.addData("  CW mean",  "%.4f in  (n=%d)", cwStats.mean, cwDrifts.size());
        telemetry.addData("  CCW mean", "%.4f in  (n=%d)", ccwStats.mean, ccwDrifts.size());
        telemetry.addLine("");

        telemetry.addLine("── Heading accuracy ──");
        telemetry.addData("  Mean error",  "%.2f°  (target %.0f°)",
                headingStats.mean, targetDeg);
        telemetry.addData("  Max error",   "%.2f°", headingStats.max);
        telemetry.addLine("");

        telemetry.addData("Battery", "%.2fV → %.2fV", startVoltage, endVoltage);
        if (voltageWarning) {
            telemetry.addLine("!! VOLTAGE DROPPED BELOW 12V DURING TEST");
            telemetry.addLine("!! Results may be unreliable — charge and re-run");
        }
        telemetry.addLine("");

        if (!biasNote.isEmpty()) {
            telemetry.addLine("!! " + biasNote);
            telemetry.addLine("");
        }

        telemetry.addLine("VERDICT: " + verdict);
        telemetry.addLine("");

        // Thresholds used (for user's reference)
        telemetry.addLine("── Thresholds (for " + String.format("%.0f", revs) + " rev) ──");
        telemetry.addData("  Excellent", "< %.3f in mean", excellentThresh);
        telemetry.addData("  Good",      "< %.3f in mean", goodThresh);
        telemetry.addData("  Acceptable","< %.3f in mean", acceptableThresh);

        telemetry.update();

        // Hold on screen until any button pressed
        while (opModeIsActive() && !gamepad1.a && !gamepad1.b
                && !gamepad1.x && !gamepad1.y
                && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            sleep(50);
        }
        sleep(300); // debounce
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  AUTO-TUNER (coordinate descent)
    // ═════════════════════════════════════════════════════════════════════════

    private double[] autoTune(double startX, double startY) {
        double curX = startX;
        double curY = startY;

        for (double step : STEPS_MM) {
            if (!opModeIsActive()) break;

            boolean improved = true;
            int pass = 0;

            while (improved && pass < MAX_PASSES_PER_STEP && opModeIsActive()) {
                improved = false;
                pass++;

                telemetry.clearAll();
                telemetry.addData("Step", "%.2f mm  |  Pass %d/%d", step, pass,
                        MAX_PASSES_PER_STEP);
                telemetry.addData("Current", "x=%.2f  y=%.2f mm", curX, curY);
                telemetry.addData("Battery", "%.2fV", voltageSensor.getVoltage());
                telemetry.update();

                double newX = tuneAxis(curX, curY, step, true);
                if (Math.abs(newX - curX) > 0.01) {
                    curX = newX;
                    improved = true;
                }
                if (!opModeIsActive()) break;

                double newY = tuneAxis(curX, curY, step, false);
                if (Math.abs(newY - curY) > 0.01) {
                    curY = newY;
                    improved = true;
                }
            }
        }
        return new double[]{curX, curY};
    }

    private double tuneAxis(double xMm, double yMm, double stepMm, boolean tuneX) {
        String axisName = tuneX ? "X (fwdPodY)" : "Y (strPodX)";
        double base = tuneX ? xMm : yMm;
        double[] candidates = {base, base + stepMm, base - stepMm};
        String[] labels = {"base", "+step", "-step"};
        double[] scores = new double[3];

        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            double cx = tuneX ? candidates[i] : xMm;
            double cy = tuneX ? yMm : candidates[i];

            telemetry.clearAll();
            telemetry.addData("Tuning", "%s  step=%.2fmm", axisName, stepMm);
            telemetry.addData("Testing", "%s = %.2f mm", labels[i], candidates[i]);
            telemetry.addData("Battery", "%.2fV", voltageSensor.getVoltage());
            telemetry.update();

            scores[i] = scoreOffsets(cx, cy);
        }

        int bestIdx = 0;
        for (int i = 1; i < 3; i++) {
            if (scores[i] < scores[bestIdx]) bestIdx = i;
        }

        telemetry.clearAll();
        telemetry.addData("Axis " + axisName, "");
        for (int i = 0; i < 3; i++) {
            telemetry.addData("  " + labels[i] + " (" +
                            String.format("%.2f", candidates[i]) + " mm)",
                    "%.4f in  %s", scores[i], i == bestIdx ? " << BEST" : "");
        }
        telemetry.update();
        sleep(500);

        return candidates[bestIdx];
    }

    private double scoreOffsets(double xMm, double yMm) {
        applyOffsets(xMm, yMm);
        double totalDrift = 0.0;
        int count = 0;

        for (int run = 0; run < RUNS_PER_CANDIDATE && opModeIsActive(); run++) {
            resetPose();
            SpinResult cw = doSpin(+1, TARGET_DEG, SPIN_TIMEOUT_S);
            totalDrift += cw.driftMagIn;
            count++;
            if (!opModeIsActive()) break;

            resetPose();
            SpinResult ccw = doSpin(-1, TARGET_DEG, SPIN_TIMEOUT_S);
            totalDrift += ccw.driftMagIn;
            count++;
        }
        return count > 0 ? totalDrift / count : Double.MAX_VALUE;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  SPIN PRIMITIVE
    // ═════════════════════════════════════════════════════════════════════════

    /**
     * Spins in place by approximately targetDeg degrees.
     * dir = +1 for CW, -1 for CCW.
     *
     * Uses delta-based heading accumulation (robust to slow loops).
     * Post-spin settle is now SETTLE_MS to ensure the robot is truly stopped
     * before reading final position — this eliminates drift from residual
     * momentum corrupting the measurement.
     */
    private SpinResult doSpin(int dir, double targetDeg, double timeoutS) {
        pinpoint.update();
        double startX = pinpoint.getPosX(DistanceUnit.INCH);
        double startY = pinpoint.getPosY(DistanceUnit.INCH);

        double lastHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
        double accumulatedDeg = 0.0;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeoutS) {
            pinpoint.update();

            double h = pinpoint.getHeading(AngleUnit.DEGREES);
            double dh = h - lastHeadingDeg;

            // Normalise delta to (-180, +180]
            while (dh >  180.0) dh -= 360.0;
            while (dh < -180.0) dh += 360.0;

            accumulatedDeg += dh;
            lastHeadingDeg = h;

            if (Math.abs(accumulatedDeg) >= (targetDeg - STOP_TOL_DEG)) break;

            setDrivePower(0, 0, SPIN_POWER * dir);
        }

        // Stop motors and wait for robot to physically settle
        setDrivePower(0, 0, 0);
        sleep(POST_SPIN_SETTLE_MS);

        // Final position read — robot must be completely stationary
        pinpoint.update();
        double endX = pinpoint.getPosX(DistanceUnit.INCH);
        double endY = pinpoint.getPosY(DistanceUnit.INCH);

        double dx  = endX - startX;
        double dy  = endY - startY;
        double mag = Math.hypot(dx, dy);

        return new SpinResult(dx, dy, mag, Math.abs(accumulatedDeg));
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  HELPERS
    // ═════════════════════════════════════════════════════════════════════════

    private void resetPose() {
        setDrivePower(0, 0, 0);
        sleep(SETTLE_MS);
        pinpoint.resetPosAndIMU();
        sleep(SETTLE_MS);
    }

    private void applyOffsets(double xMm, double yMm) {
        pinpoint.setOffsets(xMm, yMm, DistanceUnit.MM);
    }

    private void showSpinResult(String label, double xMm, double yMm, SpinResult r) {
        telemetry.clearAll();
        telemetry.addData("Test", label);
        telemetry.addData("Offsets (mm)", "x=%.2f  y=%.2f", xMm, yMm);
        telemetry.addData("Rotation (°)", "%.1f  (target %.0f)", r.rotDeg, TARGET_DEG);
        telemetry.addData("Heading err", "%.2f°", Math.abs(r.rotDeg - TARGET_DEG));
        telemetry.addData("Drift X (in)", "%.4f", r.driftXIn);
        telemetry.addData("Drift Y (in)", "%.4f", r.driftYIn);
        telemetry.addData("Drift |d| (in)", "%.4f", r.driftMagIn);
        telemetry.addData("Battery", "%.2fV", voltageSensor.getVoltage());
        telemetry.update();
        sleep(3000);
    }

    private void showFinalResult(double xMm, double yMm,
                                 double startV, double endV) {
        double xIn = xMm / 25.4;
        double yIn = yMm / 25.4;

        telemetry.clearAll();
        telemetry.addLine("══ AUTO-TUNE COMPLETE ══════════════════");
        telemetry.addData("Best X offset", "%.2f mm  /  %.4f in", xMm, xIn);
        telemetry.addData("Best Y offset", "%.2f mm  /  %.4f in", yMm, yIn);
        telemetry.addData("Battery", "%.2fV → %.2fV", startV, endV);
        telemetry.addLine("");
        telemetry.addLine("── Pedro PinpointConstants (inches): ──");
        telemetry.addData("  .forwardPodY", "(%.4f)", xIn);
        telemetry.addData("  .strafePodX",  "(%.4f)", yIn);
        telemetry.addLine("");
        telemetry.addLine("── globals.offsets (mm): ──");
        telemetry.addData("  .xoff", "= %.2f", xMm);
        telemetry.addData("  .yoff", "= %.2f", yMm);
        telemetry.addLine("");
        telemetry.addLine(">> Press DPAD_UP to VERIFY these offsets");
        telemetry.addLine("════════════════════════════════════════");
        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  STATISTICS
    // ═════════════════════════════════════════════════════════════════════════

    private static class Stats {
        final double mean, stdDev, min, max;
        final int n;

        Stats(double mean, double stdDev, double min, double max, int n) {
            this.mean   = mean;
            this.stdDev = stdDev;
            this.min    = min;
            this.max    = max;
            this.n      = n;
        }
    }

    private Stats computeStats(List<Double> values) {
        if (values.isEmpty()) return new Stats(0, 0, 0, 0, 0);

        double sum = 0;
        for (double v : values) sum += v;
        double mean = sum / values.size();

        double sumSq = 0;
        for (double v : values) sumSq += (v - mean) * (v - mean);
        double stdDev = Math.sqrt(sumSq / values.size());

        return new Stats(mean, stdDev,
                Collections.min(values), Collections.max(values),
                values.size());
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  DATA CLASS
    // ═════════════════════════════════════════════════════════════════════════

    private static class SpinResult {
        final double driftXIn, driftYIn, driftMagIn, rotDeg;

        SpinResult(double driftXIn, double driftYIn, double driftMagIn, double rotDeg) {
            this.driftXIn   = driftXIn;
            this.driftYIn   = driftYIn;
            this.driftMagIn = driftMagIn;
            this.rotDeg     = rotDeg;
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  DRIVE HELPERS
    // ═════════════════════════════════════════════════════════════════════════

    private void setDrivePower(double x, double y, double turn) {
        double lf = y + x + turn;
        double rf = y - x - turn;
        double lb = y - x + turn;
        double rb = y + x - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }

    private void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior zpb = brake
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFront.setZeroPowerBehavior(zpb);
        rightFront.setZeroPowerBehavior(zpb);
        leftBack.setZeroPowerBehavior(zpb);
        rightBack.setZeroPowerBehavior(zpb);
    }
}