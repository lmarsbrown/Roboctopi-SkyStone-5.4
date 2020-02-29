package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.MotorController;
import org.firstinspires.ftc.teamcode.Robot.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Positions;
import org.firstinspires.ftc.teamcode.Robot.Robot_Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot_Localizer;
import org.firstinspires.ftc.teamcode.Robot.StonePipeline;
import org.firstinspires.ftc.teamcode.Utils.Interval;
import org.firstinspires.ftc.teamcode.Utils.Lambda;
import org.firstinspires.ftc.teamcode.Utils.Transform;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
//@Disabled
public class FullBlueAuto extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Robot_Localizer rowboat;
    private Robot_Controller control;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor horizontal_extender;
    private DcMotor vertical_extender;

    private Servo collector_arm;
    private Servo foundation_mover;
    private Servo right_stone_collector;
    private Servo right_stone_collector_arm;
    private Servo left_foundation_mover;
    private Servo right_foundation_mover;
    private Servo left_stone_collector;
    private Servo left_stone_collector_arm;

    private CRServo outer_collector;
    private CRServo inner_collector;

    private double gp1_percent_pwr;
    private double gp2_percent_pwr;

    private Transform saved_robot_pos;
    private Transform robot_vector;

    private boolean going_to_pt;

    private DigitalChannel limit_switch_front;
    private DigitalChannel limit_switch_back;

    StonePipeline pip;

    OpenCvCamera webcam;

    private MotorController vertCont;


    @Override
    public void init() {
        leftFront           = hardwareMap.get(DcMotor.class, "left_front");
        rightFront          = hardwareMap.get(DcMotor.class, "right_front");
        leftBack            = hardwareMap.get(DcMotor.class, "left_back");
        rightBack           = hardwareMap.get(DcMotor.class, "right_back");
        horizontal_extender = hardwareMap.get(DcMotor.class, "horizontal_ext");
        vertical_extender   = hardwareMap.get(DcMotor.class, "vertical_ext");

        collector_arm       = hardwareMap.get(Servo.class, "collector_arm");
        foundation_mover    = hardwareMap.get(Servo.class, "Foundation_mover");
        left_foundation_mover     = hardwareMap.get(Servo.class, "front_foundation_left");
        right_foundation_mover     = hardwareMap.get(Servo.class, "front_foundation_right");
        right_stone_collector = hardwareMap.get(Servo.class, "right_stone_collector");
        right_stone_collector_arm = hardwareMap.get(Servo.class, "right_stone_collector_arm");
        left_stone_collector = hardwareMap.get(Servo.class, "left_stone_collector");
        left_stone_collector_arm = hardwareMap.get(Servo.class, "left_stone_collector_arm");

        outer_collector     = hardwareMap.get(CRServo.class, "outer_collector");
        inner_collector     = hardwareMap.get(CRServo.class, "inner_collector");

        limit_switch_back   = hardwareMap.get(DigitalChannel.class, "limit_switch1");
        limit_switch_front  = hardwareMap.get(DigitalChannel.class, "limit_switch2");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        vertical_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertical_extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical_extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rowboat = new Robot_Localizer(leftBack,rightFront,rightBack,0.958);
        control = new Robot_Controller(rightFront,leftFront,rightBack,leftBack,rowboat,false);

        going_to_pt = false;


        //Move collector_arm up
        //collector_arm.setPosition(0.77);







        //foundation_mover.setPosition(0);

        //WebcamName camName = hardwareMap.get(WebcamName.class,"Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */

        pip = new StonePipeline(800,0,320,240,60);
        webcam.setPipeline(pip);

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at 720p will limit you to up to 10FPS. However, streaming at frame rates other
         * than 30FPS is not currently supported, although this will likely be addressed in a future
         * release. TLDR: You can't stream in greater than 480p from a webcam at the moment.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        vertCont = new MotorController(vertical_extender,new PIDController(0.002,0.0001,0.002,0));
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        int stonePos = pip.stonePos;
        getStone(stonePos+3,720,200,(Object stone)->{
            control.gotoPoint(new Transform(2254,750,Math.PI*0.5),true,0.35,0.85,100,(Object alphabet)->{
                right_stone_collector_arm.setPosition(Positions.RIGHT_ARM_ISH);
                right_stone_collector.setPosition(Positions.RIGHT_PINCHER_BRIDGE);
                try {
                    Thread.sleep(284);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                control.gotoPoint(new Transform(2254,550,Math.PI*0.5),true
                        ,0.25,0.6,80,(Object abbcdea)->{
                            right_stone_collector_arm.setPosition(Positions.RIGHT_ARM_RETRACT);
                            getStone(stonePos,750,225,(Object stone1)->{
                                control.gotoPoint(new Transform(2021,777,Math.PI*0.5),true,0.35,1,100,(Object alphabetcdefg)->{
                                    right_stone_collector.setPosition(Positions.RIGHT_PINCHER_BRIDGE);
                                    right_stone_collector_arm.setPosition(Positions.RIGHT_ARM_ISH);
                                    try {
                                        Thread.sleep(273);
                                        Thread.sleep(500);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                    right_stone_collector_arm.setPosition(Positions.RIGHT_ARM_RETRACT);
                                    right_stone_collector.setPosition(Positions.RIGHT_PINCHER_RETRACT);


                                    if(runtime.seconds()<24)
                                    {
                                        vertCont.setTarget(700,false);
                                        left_foundation_mover.setPosition(0.27);
                                        right_foundation_mover.setPosition(0.73);
                                        control.gotoPoint(new Transform(2021,500,Math.PI),true,0.35,0.85,30,Math.PI*0.7,0.1,(Object abcdefhlep)->{
                                            control.gotoPoint(new Transform(2021,725,Math.PI),false,0.25,0.85,50,Math.PI,0.1,(Object a1)->{
                                                left_foundation_mover.setPosition(0.14);
                                                right_foundation_mover.setPosition(0.86);
                                                try {
                                                    Thread.sleep(560);
                                                } catch (InterruptedException e) {
                                                    e.printStackTrace();
                                                }
                                                //TODO: LOWEER THHHAT Y COOOORDINSSTTES
                                                control.gotoPoint(new Transform(1200,500,Math.PI*0.65),true,0.75,0.9,400,Math.PI*0.5,0.25,(Object a2)->{
                                                    left_foundation_mover.setPosition(0.14);
                                                    right_foundation_mover.setPosition(0.86);
                                                    control.gotoPoint(new Transform(1928,500,Math.PI*0.5),false,0.85,0.9,130,Math.PI*0.5,0.07,(Object a3)->{
                                                        left_foundation_mover.setPosition(0.73);
                                                        right_foundation_mover.setPosition(0.25);
                                                        control.gotoPoint(new Transform(1000,800,Math.PI*0.5), true,0.5,0.85,150,(Object b3)->0);

                                                        return 0;
                                                    });
                                                    return 0;
                                                });
                                                return 0;
                                            });
                                            return 0;
                                        });
                                    }
                                    else
                                    {
                                        control.gotoPoint(new Transform(2021,500,Math.PI*0.5),true,0.25,0.85,50,(Object afjrj)->{
                                            control.gotoPoint(new Transform(1300,550,Math.PI*0.5), true,0.5,0.85,150,(Object abcdefhlep)->0);
                                            return 0;
                                        });

                                    }
                                    return 0;
                                });
                                return 0;
                            });
                            return 0;
                        });
                return 0;
            });
            return 0;
        });
        Interval delay = new Interval((Object obj42)->{
            right_stone_collector.setPosition(Positions.RIGHT_PINCHER_ISH);

            try {
                Thread.sleep(250);
                collector_arm.setPosition(0.63);
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            vertCont.setTarget(1302,true);
            webcam.stopStreaming();
            return 1;
        },0);
        delay.start();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        rowboat.relocalize();
        vertCont.updateController();
        telemetry.addData("color",pip.stonePos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void getStone(int stoneNum,double y,double bDist, Lambda callback)
    {
        //TODO: Fix 2nd pos
        control.gotoPoint(new Transform(Math.max(350-(200*(stoneNum)),-558),y-150,Math.PI*0.5),true,0.35,0.9,60,Math.PI*0.5,0.045,(Object obj)->{
            right_stone_collector.setPosition(Positions.RIGHT_PINCHER_ISH);
            right_stone_collector_arm.setPosition(Positions.RIGHT_ARM_ISH);
            control.gotoPoint(new Transform(350-(200*(stoneNum)),y,Math.PI*0.5),true,0.35,0.6,35,(Object obj1)->{
                try {
                    right_stone_collector_arm.setPosition(Positions.RIGHT_ARM_DOWN);
                    Thread.sleep(200);
                    right_stone_collector.setPosition(Positions.RIGHT_PINCHER_DOWN);
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                right_stone_collector_arm.setPosition(Positions.RIGHT_ARM_RETRACT+0.06);
                control.gotoPoint(new Transform(350-(200*(stoneNum)),y-bDist,Math.PI*0.5),true,0.35,0.6,80,(Object abbcdea)->{
                    callback.call(stoneNum);
                    return 0;
                });
                return 0;
            });
            return 0;
        });
    }

}