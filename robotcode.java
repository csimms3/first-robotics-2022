package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot2driveV2 (Blocks to Java)")
public class Robot2driveV2 extends LinearOpMode {

  private ElapsedTime     runtime = new ElapsedTime();

  //initializing all motors and servos (making code recognize motors and assigning names)

  private DcMotor rightRearMotor;
  private DcMotor rightFrontMotor;
  private DcMotor leftRearMotor;
  private DcMotor leftFrontMotor;
  private DcMotor strafeMotor;
  private DcMotor chainMotor;

  private CRServo rightIntake;
  private CRServo leftIntake;
  private CRServo rightCarousel;
  private CRServo leftCarousel;

  private CRServo topIntake;

  //power to left/right side of drivetrain
  double leftDrivePower;
  double rightDrivePower;
  double strafePower;

  //drive constants, to control power
  double masterSpeed = 0.4;
  double turnSpeed = 0.3;
  double maxStrafePower = 0.5;

  //chain
  double chainSpeed = 0.3;
  double topIntPow = 0.2;
  double timer = 0;

  int chainPosition = 1;
  double liftTimeout = 0;
  double dumpTimer = 0;


  @Override
  public void runOpMode() {
    //hardware configuration, naming all motors/servos and configuring direction/behaviour

    //motor config
    rightRearMotor = hardwareMap.get(DcMotor.class, "right_rear_motor");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
    leftRearMotor = hardwareMap.get(DcMotor.class, "left_rear_motor");
    leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
    strafeMotor = hardwareMap.get(DcMotor.class, "strafe_motor");
    chainMotor = hardwareMap.get(DcMotor.class, "chain_motor");

    //servo config
    rightIntake = hardwareMap.get(CRServo.class, "right_intake");
    leftIntake = hardwareMap.get(CRServo.class, "left_intake");
    topIntake = hardwareMap.get(CRServo.class, "top_intake");

    rightCarousel = hardwareMap.get(CRServo.class, "right_carousel");
    leftCarousel = hardwareMap.get(CRServo.class, "left_carousel");

    //direction fixing (so all motors drive in the same direction)
    leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    topIntake.setDirection(DcMotorSimple.Direction.REVERSE);

    //prevent robot from rolling when power is cut
    rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //opMode loop, this is what actually runs when you press start.
    waitForStart();
    if (opModeIsActive()) {
      //stuff here will be done once when "INIT" is pressed (driver hub)
      while (opModeIsActive()) {
        //main drive loop, methods here are called repeatedly while active
        drive("singlejoystick");
        Intake();
        Carousel();
        liftFreight();
        Telemetry();
      }
    }
  }



  
  //drive loop, handles joystick control for wheels and strafe wheel, as well as speed boost
  private void drive(String mode) {
    if (gamepad2.right_trigger > 0.5) {
      masterSpeed = 0.8;
    } else {
      masterSpeed = 0.4;
    }

    leftDrivePower = 0;
    rightDrivePower = 0;
    strafePower = 0;

    if (mode.equals("A")) {
      //left stick used for driving straight and strafing
      if (-0.2 > gamepad2.left_stick_y | gamepad2.left_stick_y > 0.2) {
        leftDrivePower = masterSpeed * gamepad2.left_stick_y;
        rightDrivePower = masterSpeed * gamepad2.left_stick_y;
      }
      //right stick used for turning
      if (-0.2 > gamepad2.right_stick_x | gamepad2.right_stick_x > 0.2) {
        leftDrivePower = turnSpeed * -gamepad2.right_stick_x;
        rightDrivePower = turnSpeed * gamepad2.right_stick_x;
      }
      //strafing
      if (-0.2 > gamepad2.left_stick_x | gamepad2.left_stick_x > 0.2) {
        strafePower = maxStrafePower * gamepad2.left_stick_x;
      }
    }
    else if (mode.equals("singlejoystick")) {
      //right stick used for driving straight and turning
      if (-0.5 < gamepad2.left_stick_x && gamepad2.left_stick_x < 0.5) {
        leftDrivePower = masterSpeed * gamepad2.left_stick_y;
        rightDrivePower = masterSpeed * gamepad2.left_stick_y;
      }

      if (-0.5 < gamepad2.left_stick_y && gamepad2.left_stick_y < 0.5) {
        leftDrivePower = turnSpeed * gamepad2.left_stick_x;
        rightDrivePower = turnSpeed * -gamepad2.left_stick_x;
      }
      Strafe();
    }

    rightRearMotor.setPower(rightDrivePower);
    rightFrontMotor.setPower(rightDrivePower);
    leftRearMotor.setPower(leftDrivePower);
    leftFrontMotor.setPower(leftDrivePower);
    strafeMotor.setPower(strafePower);
  }




  //strafe code, used in drive method (currently not used due to new drive style)
  private void Strafe() {
    if (gamepad2.right_bumper) {
      strafePower = maxStrafePower * -1;
    } else if (gamepad2.left_bumper) {
      strafePower = maxStrafePower * 1;
    }
    else {
      strafePower = 0;
    }
  }




  //intake code, to take in cargo.
  private void Intake() {
    if (gamepad1.left_trigger > 0.1) {
      intakePower(gamepad1.left_trigger);
    } else if(gamepad1.right_trigger > 0.1) {
      intakePower(gamepad1.right_trigger * -1);
    } else {
      intakePower(0);
    }
  }




  //set intake power (only used to shorten above code)
  private void intakePower(float d) {
    rightIntake.setPower(d);
    leftIntake.setPower(d);
  }




  //handles both duck carousels
  private void Carousel() {
    if (gamepad1.a) {
      leftCarousel.setPower(0.7);
      rightCarousel.setPower(-0.7);
    } else {
      leftCarousel.setPower(0);
      rightCarousel.setPower(0);
    }
  }




  //moves chain up and down
  private void moveChain() {
    if (gamepad2.right_bumper) {
      chainMotor.setPower(1*chainSpeed);
    } else if(gamepad2.left_bumper) {
      chainMotor.setPower(-1*chainSpeed);
    }
  }

  private boolean posDiff(DcMotor a, int tolerance) {
    return Math.abs(a.getCurrentPosition() - a.getTargetPosition()) > tolerance;
  }






  private void liftFreight() {
    //chainPosition, 0 = load; 1 = hold; 2 = dump
    // 2 should return to 1 after dump

    if (liftTimeout < runtime.seconds()) {
      if (gamepad1.dpad_up && chainPosition < 2) {
        chainPosition++;
        liftTimeout = runtime.seconds() + 0.5;
      }
      if (gamepad1.dpad_down && chainPosition > 0) {
        chainPosition--;
        liftTimeout = runtime.seconds() + 0.5;
      }
    }

    int bottomChainPosition = -1860;
    int holdChainPosition = 0;


    if ((chainMotor.getCurrentPosition() < (holdChainPosition - 900)) && 
        (chainMotor.getCurrentPosition() > (holdChainPosition - 1840)) && chainPosition == 1) {
      topIntake.setPower(topIntPow);
    }
    else if ((chainMotor.getCurrentPosition() < (holdChainPosition - 900)) && 
             (chainMotor.getCurrentPosition() > (holdChainPosition -1840)) && chainPosition == 0) {
      topIntake.setPower(-1 * topIntPow);
    }
    else {
      topIntake.setPower(0);
    }

    if (chainPosition == 0) {
      chainMotor.setTargetPosition(holdChainPosition - 1865);
    }
    //+1850
    else if (chainPosition == 1) {
      chainMotor.setTargetPosition(holdChainPosition);
    }
    //+500
    else if (chainPosition == 2) {
      chainMotor.setTargetPosition(holdChainPosition + 310);
      //if (!posDiff(chainMotor, 5)) {
      //chainPosition--;
      //}
    }
    if (posDiff(chainMotor, 1)) {
      chainMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      chainMotor.setPower(chainSpeed);
    } else {
      chainMotor.setPower(0);
    }
  }





  //telemetry updates, to see info live while robot is active
  private void Telemetry() {
    telemetry.addLine("Bartholomew V2");
    telemetry.addData("Chain pow", chainMotor.getPower());
    telemetry.addData("Chain encoder", chainMotor.getCurrentPosition());
    telemetry.addData("Chain pos", chainPosition);


    telemetry.update();
  }
}