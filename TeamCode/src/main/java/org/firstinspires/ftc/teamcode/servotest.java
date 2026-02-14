package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class servotest extends OpMode {

    private ServoEx t1, t2;
    private boolean spin;

    @Override
    public void init(){
        t1 = new ServoEx(hardwareMap, "t1", 300, AngleUnit.DEGREES);
        t2 = new ServoEx(hardwareMap, "t2", 300, AngleUnit.DEGREES);

        t1.set(0);
        t2.set(0);

        spin = false;
    }

    @Override
    public void loop(){
        if (!spin){
            t1.set(300);
            t2.set(300);
            spin = true;
        }
    }
}
