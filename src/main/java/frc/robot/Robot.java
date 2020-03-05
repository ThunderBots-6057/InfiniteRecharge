/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
//3/5/20 push
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

//all global vars, double for deci, boolean for binary, string for word 
  double rightTog;
  double leftTog;
  double rightTrolley;
  double leftTrolley;
  double unicornStrt;
  double test2;

  boolean intakeSysR;
  boolean outsysR;
  boolean climbUp;
  boolean climbdown;
  boolean shooter;
  boolean test1;
  boolean conveytorSysIn;
  boolean conveytorsysOut;
  String colorString;

 // all declarations
 Joystick controller = new Joystick(0);
 Joystick controller2 = new Joystick(1);

 //color sensor parts
private final I2C.Port i2cPort = I2C.Port.kOnboard;//this is a port on the roboRIO
private final ColorSensorV3 unicornsensor = new ColorSensorV3(i2cPort); //this is the sensor itself 
private final ColorMatch m_colorMatcher = new ColorMatch(); // this the part on the sensor that actually senses the color 

//these are all of the colors that we need, however we may need to change them to our current colors though 
private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


  // all of these motors are to be assigned to the first controller, includes:
  // driving,climbing motors and the "trolley" part, aka the part that allows us
  // to move
  // back and forth on pendulum
  //will need to change these due to wiring issues please check!!!!! 2/23/2020 
  VictorSPX frontLeft = new VictorSPX(5);
  VictorSPX rearLeft = new VictorSPX(1);
  VictorSPX frontRight = new VictorSPX(6);
  VictorSPX rearRight = new VictorSPX(7);
  VictorSPX trolleyPrt = new VictorSPX(3);
  VictorSPX climbPrt1 = new VictorSPX(11);
  VictorSPX climbPrt2 = new VictorSPX(2);

  // all of these are assigned to the second controller, includ es: intake system,
  // Shooter, and unicorn horn
  VictorSPX convey1 = new VictorSPX(4);
  VictorSPX convey2 = new VictorSPX(9);
  VictorSPX intakeRoller = new VictorSPX(10);
  VictorSPX shtrRight = new VictorSPX(8);
  VictorSPX shtrLeft = new VictorSPX(0);
  VictorSPX unicorn = new VictorSPX(12);

  // these speed controller groups make it so that both motor controllers listed
  // are given the same input in any function used, if one is set at one speed,
  // the other
  // one listed will also be set to that speed, even if a speed is not specified
  // and is set to a parameter instead
  //SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRight, rearRight);
  //SpeedControllerGroup climbMtrs = new SpeedControllerGroup(climbPrt1,climbPrt2);
  //SpeedControllerGroup intakeSystm = new SpeedControllerGroup(intakeLft,intakeRght,intakeDwnCntr);


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(640, 480);

      UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture();
      camera2.setResolution(640, 480);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }
    }).start();



//all colors must be initialized or else smartdashboard will not know what to do 
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    


  }
  

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */



   //this is the drive section
  void drive() {
		leftTog = controller.getRawAxis(1) * -0.75;//  for the controller and sets drive speed at 75%; the port numbers set the controls to the left and right sticks
		rightTog = controller.getRawAxis(5) * 0.75;

		// DriverStation.reportWarning("$L" + leftTog, false);
		// DriverStation.reportWarning("$R" + rightTog, false);

		rearLeft.set(ControlMode.PercentOutput, leftTog);
		frontLeft.set(ControlMode.PercentOutput,leftTog);

		rearRight.set(ControlMode.PercentOutput,rightTog);
		frontRight.set(ControlMode.PercentOutput,rightTog);
	}

	public void backUp() {

		rearLeft.set(ControlMode.PercentOutput,-.75);
		frontLeft.set(ControlMode.PercentOutput,-.75);

		rearRight.set(ControlMode.PercentOutput,.75);
		frontRight.set(ControlMode.PercentOutput,.75);

	}

	public void turnRight() { // turns right going forward

		rearLeft.set(ControlMode.PercentOutput,.75);
		frontLeft.set(ControlMode.PercentOutput,.75);

		rearRight.set(ControlMode.PercentOutput,.6);
		frontRight.set(ControlMode.PercentOutput,.6);
	}

	public void turnLeft() { // Okay, this looks suspect to me, because of the positive/negative difference
								// in the speed. Test, create a new branch to fix it, test again, repeat

		rearLeft.set(ControlMode.PercentOutput,-.75);
		frontLeft.set(ControlMode.PercentOutput,-.75);

		rearRight.set(ControlMode.PercentOutput,0.6);
		frontRight.set(ControlMode.PercentOutput,0.6);
	}

	public void driveForward() {

		rearLeft.set(ControlMode.PercentOutput,.75);
		frontLeft.set(ControlMode.PercentOutput,.75);

		rearRight.set(ControlMode.PercentOutput,-.75);
		frontRight.set(ControlMode.PercentOutput,-.75);

	}

	public void driveStop()

	{
		rearLeft.set(ControlMode.PercentOutput,0);
		frontLeft.set(ControlMode.PercentOutput,0);

		rearRight.set(ControlMode.PercentOutput,0);
		frontRight.set(ControlMode.PercentOutput,0);

  }
  

  

void climbsys(){

climbUp = controller.getRawButton(3);
climbdown = controller.getRawButton(1);

if(climbUp == true){

climbPrt1.set(ControlMode.PercentOutput,0.5);
climbPrt2.set(ControlMode.PercentOutput,0.5);



}

else if(climbdown==true){

  climbPrt1.set(ControlMode.PercentOutput,-0.5);
  climbPrt2.set(ControlMode.PercentOutput,-0.5);
  

} 

else{

  climbPrt1.set(ControlMode.PercentOutput,0);
  climbPrt2.set(ControlMode.PercentOutput,0);
  


}



}




void trolley(){
rightTrolley = controller.getRawAxis(3);
leftTrolley = controller.getRawAxis(2);


if(rightTrolley >= 0.1 ){

  trolleyPrt.set(ControlMode.PercentOutput,0.75);
  
  


}
else if(leftTrolley >= 0.1 ){

  trolleyPrt.set(ControlMode.PercentOutput,-0.75);

}

else{

  trolleyPrt.set(ControlMode.PercentOutput,0);
  

}


}

void unicorn(){


unicornStrt = controller2.getRawAxis(3) * 0.5; 

if(unicornStrt >= 0.5){

unicorn.set(ControlMode.PercentOutput,unicornStrt);

}
//might not need this else if, keep for right now 
else if(unicornStrt <= 0.5 ){

  unicorn.set(ControlMode.PercentOutput,unicornStrt* 0.25);




}

else{

unicorn.set(ControlMode.PercentOutput,0);


}





}
void shooter(){

  shooter = controller2.getRawButton(1);//button A is used to activate
 

  if(shooter==true){

    shtrLeft.set(ControlMode.PercentOutput,1);
    shtrRight.set(ControlMode.PercentOutput,1);

  }
else{

  shtrLeft.set(ControlMode.PercentOutput,0);
  shtrRight.set(ControlMode.PercentOutput,0);



}

}




void roller(){

intakeSysR = controller2.getRawButton(1);
outsysR = controller2.getRawButton(4);
// button B is used; i set it to 1 variable becasue once the button is clicked, all 3 motors will be activated at the same time right
//angela: yes it should be only one variable for all 3 :) 

if(intakeSysR == true){

  intakeRoller.set(ControlMode.PercentOutput,-0.5);
  //intakeLft.set(ControlMode.PercentOutput,0.5);
  //intakeRght.set(ControlMode.PercentOutput,0.5);

}
else if(outsysR == false){

  intakeRoller.set(ControlMode.PercentOutput,0);
  //intakeLft.set(ControlMode.PercentOutput,0);
  //intakeRght.set(ControlMode.PercentOutput,0);

}
else{

  intakeRoller.set(ControlMode.PercentOutput,0);
  //intakeLft.set(ControlMode.PercentOutput,0);
  //intakeRght.set(ControlMode.PercentOutput,0);
     
}

}


void conveytor(){

  conveytorSysIn = controller2.getRawButton(2);
  conveytorsysOut = controller2.getRawButton(3);
  // button B is used; i set it to 1 variable becasue once the button is clicked, all 3 motors will be activated at the same time right
  //angela: yes it should be only one variable for all 3 :) 
  
  if(conveytorSysIn == true){
  
    
    convey1.set(ControlMode.PercentOutput,0.5);
    convey2.set(ControlMode.PercentOutput,0.5);
  
  }
  else if(conveytorsysOut == false){
  
   
      convey1.set(ControlMode.PercentOutput,0);
      convey2.set(ControlMode.PercentOutput,0);
    convey1.set(ControlMode.PercentOutput,0);
    convey2.set(ControlMode.PercentOutput,0);
       
  }






}







  @Override
  public void robotPeriodic() {
    //stores detected color as variable
    Color detectedColor = unicornsensor.getColor();

//stored as the acutal matcher, so color can be slightly off

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
//puts color through if-else, to determine what to give to smartdash

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

//gives val to smartdash, makes it print it so driver sees 
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }




  

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
     m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);


    m_autoSelected=m_chooser.getSelected();
		
		System.out.println("Auto Selected: "+m_autoSelected);
		
	
    

    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here




        break;
      case kDefaultAuto:
      default:
        // Put default auto code here




        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    drive();
    climbsys();
    trolley();
    unicorn();
    shooter();
    roller();
    conveytor();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    drive();
    climbsys();
    trolley();
    unicorn();
    shooter();
    roller();
    conveytor();
  }
}
