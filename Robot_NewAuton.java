//
// ** This code is for Team 2866's 2013 robot. ***/
/*
 * Change History 
 * 
 *  1-28-2013   Created
 *  6 Feb 2013  corrected network Table calls
 *              reading from networktable and writing to Smartdashboard
 * 
 *  7 Feb 2013  added auto aim in autonomous
 *              changed button 8 to toggle gun on/off
 * 
 *  9 Feb 2013  changed angle and intake motors to Jaguar controllers
 *              added dashboard reporting to disabled
 *              moved auto aim to button 6 vs 8
 *              added elevator control for auto aim
 *              added scoop and hang solenoids and controls
 * 
 * 10 Feb 2013  added checks for encoder and gyro functionality.
 *              added PID azimuth control on auto aim
 * 
 * 17 Feb 2013  addded exception handlers for network tables
 *              consolidated print staments and only print every 1/2 sec
 *              added limit switches for elevator
 * 
 * 18 Feb 2013  Added Auton
 * 
 *  1 Mar 2013 Converted Auton to switch statements
 *              Added reverse for intake roller on device button 9
 * 
 * 16 Mar 2013  Timer reads in seconds - removed divide by 1e6 for print timer
 *              added auton selction and 4 disk, 5 disk, 2 disk modes
 *
 */
/*
 **************************************************************************
 *
 * The details of the behavior are summarized below:
 *
 * Disabled Mode: - reset shaft encoders and gyro
 *
 * Autonomous Mode: Shoot from initial position then move forward to pick up
 * frisbee and shoot again
 *
 * Teleop Mode: 
 *  - tank drive drive operation using Gamepad on Joystick1 (left & right sticks) 
 *  - Gamepad on Joystick 3 operates the devices
 *
 * This code assumes the following connections: 
 * - Driver Station: - USB 1 - Dual Action Gamepad. Used for tank drive 
 * - USB 3 - Dual Action Gamepad. Used as the device control
 *
 * Robot_NewAuton Ports: 
 * - Analog Module in CRio Slot 1
 *      - analog input 1 - Gyro angle 
 *      - analog input 2 - Gyro temperature compensation
 *
 * - Solenoid Module in CRio Slot 3 
 *      - Solenoid 1 - hopper raise/lower 
 *      - Solenoid 2 - shoot: feed frisbee 
 *      - Solenoid 3 - scoop raise/lower 
 *      - Solenoid 4 - hang raise/lower 
 *
 * - Digital Sidecar 1: in CRio Slot 2 
 *      - PWM 1 - Connected to "left" drive motor(s) 
 *      - PWM 2 - Connected to "right" drive motor(s) 
 *      - PWM 3 - gun angle
 *      - PWM 4 - gun 1 
 *      - PWM 5 - gun 2 
 *      - PWM 6 - gun 3 
 *      - PWM 7 - intake rollers 
 *
 *      - digital I/O 1 - Air compressor pressure switch 
 *      - digital I/O 2 
 *      - digital I/O 3 - Right Drive Encoder, Phase A 
 *      - digital I/O 4 - Right Drive Encoder, Phase B (single wire) 
 *      - digital I/O 5 
 *      - digital I/O 6 - Left Drive Encoder, Phase A 
 *      - digital I/O 7 - Left Drive Encoder, Phase B (single wire) 
 *      - digital I/O 8 
 *      - digital I/O 9 - elevator upper limit
 *      - digital I/O 10 - elevator lower limit
 * 
 * - relay 1 - Air compressor power
 *
 * Joystick control definitions 
 *  - DriveStick (Gamepad on joystick port 1) 
 *      - Left stick (axis 4) - left drive 
 *      - Right stick (axis 2) - right drive 
 *      - Button 1 (left) - 
 *      - Button 2 (bottom)- 
 *      - Button 3 (right) - 
 *      - Button 4 (top) - 
 *      - Button 5 (Left Upper Trigger) - intake on 
 *      - Button 7 (Left lower Trigger) - intake off 
 *      - Button 6 (Right Upper Trigger) - Auto Aim 
 *      - Button 8 (Right Lower Trigger) - Drive 
 *      - Button 9 - 
 *      - Button 10 -
 *
 * - DeviceStick (Gamepad on joystick port 3) 
 *      - Left stick (axis 4) 
 *      - Right stick (axis 2) - gun angle 
 *      - D pad (axis 6) raise /lower hanging
 *      - Button 1 (left)  -  hopper down 
 *      - Button 4 (top)   -  hopper up 
 *      - Button 2 (bottom)- 
 *      - Button 3 (right) - 
 *      - Button 5 (Left Upper Trigger) - Arm gun 
 *      - Button 7 (Left lower Trigger) - Disarm & pickup 
 *      - Button 6 (Right Upper Trigger) - Shoot 
 *      - Button 8 (Right Lower Trigger) - Gun On/Off
 *      - Button 9 - 
 *      - Button 10 -
 * **************************************************************************
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.TableKeyNotDefinedException;

public class Robot_NewAuton extends IterativeRobot {

    //  Objects for the robot drive system and other motors
    RobotDrive m_robotDrive;	// robot will use PWM 1-2 for drive motors
    Victor m_intake;
    Victor m_angle;
    Victor m_gun1;
    Victor m_gun2;
    Victor m_gun3;
    //
    // Objects for navigation sensors, shaft encoders, gyro 
    Encoder m_leftCount;        // on di I/O 3 & 4
    Encoder m_rightCount;       // on di I/O 6 & 7
    Gyro m_gyro;
    //
    //Object for air compressor
    Compressor m_airCompressor;
    //
    // Objects for the two joysticks being used
    Joystick m_driveStick;	// joystick 1 (drive stick)
    Joystick m_deviceStick;	// joystick 3 (device controller)
    //
    // Objects for the solenoid outputs
    Solenoid m_hopperSolenoid;
    Solenoid m_shooterSolenoid;
    Solenoid m_scoopSolenoid;
    Solenoid m_hangSolenoid;
    //I/O objects
    DigitalInput m_elevUpperLimit;
    DigitalInput m_elevLowerLimit;
    //Objects for Relays
    //
    // Timer Object
    Timer timer; //time shpuld be in microseconds (10^-6)
    // 
    //Variables
    double m_elevOffset = 8.0; //  larger is higher aim
    boolean m_autoAim = false;
    double azimuth = 0;
    double m_PIDAz;
    double m_PIDOut;
    double m_targetElevation = 0;
    boolean m_found = false;
    public static NetworkTable targetTable;
    SendableChooser autoMode;
    Integer AutonMode;
    int autonStep;
    boolean button8;        //Gun on/Off control
    boolean button8Previous = false;
    boolean gunOn = false;
    double m_Kp;
    double m_Ki;
    double m_Kd;
    //double printSec;
    double previousTime;
    double currentTime;

    public Robot_NewAuton() {
        /*
         *  Constructor for this "Robot_NewAuton" Class.
         *
         * The constructor creates all of the objects used for the different
         * inputs and outputs of the robot. Essentially, the constructor defines
         * the input/output mapping for the robot, providing named objects for
         * each of the robot interfaces.
         */
        m_robotDrive = new RobotDrive(1, 2);  // using PWM ports 1 & 2
        m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
        m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
        m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
        m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);

        m_gun1 = new Victor(4);
        m_gun2 = new Victor(5);
        m_gun3 = new Victor(6);
        m_intake = new Victor(7);
        m_angle = new Victor(8);

        m_rightCount = new Encoder(3, 4, false);    // on dig I/O 3 & 4
        m_rightCount.setDistancePerPulse(.004 * 6);  // 112" per 5900 counts

        m_leftCount = new Encoder(6, 7, true);   // on dig I/O 6 & 7
        m_leftCount.setDistancePerPulse(.004 * 6);  // 112" per 5900 counts
        m_leftCount.setReverseDirection(true);

        m_gyro = new Gyro(1);
        m_gyro.setSensitivity(.007);

        m_airCompressor = new Compressor(1, 1); //Dig I/O 1, Relay 1

        m_elevUpperLimit = new DigitalInput(9);
        m_elevLowerLimit = new DigitalInput(10);

        // set the MotorSafety expiration timer
        m_robotDrive.setExpiration(7200);

        // Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
        m_driveStick = new Joystick(1);
        m_deviceStick = new Joystick(3);

        // define solenoids
        m_hopperSolenoid = new Solenoid(1);
        m_shooterSolenoid = new Solenoid(2);
        m_scoopSolenoid = new Solenoid(3);
        m_hangSolenoid = new Solenoid(4);

        // define relays
        //relay 1 used for compressor switch

        timer = new Timer();

        System.out.println(" Constructor Completed\n");
    }
    //********************************* System Clock Timer Setup *************************************
    static double printSec = ((Timer.getFPGATimestamp()) + 1.0); //FPGA Time should be in seconds
    static final double startSec = (Timer.getFPGATimestamp());

    //*********************** POWER ON ROUTINE ************************* 
    //**     Run on robot power up - place initialization code here.  ***/
    public void robotInit() {
        SmartDashboard.putString("System State", "Initializing...");
        targetTable = NetworkTable.getTable("targetTable");

        autoMode = new SendableChooser();

        autoMode.addDefault("Center Pos - 5 disk", Integer.valueOf(1));
        autoMode.addObject("Center Pos - 4 disk", Integer.valueOf(2));
        autoMode.addObject("Side Pos - 2 disk", Integer.valueOf(3));
        SmartDashboard.putData("Autonomous mode chooser", autoMode);

        //start air compressor
        m_airCompressor.start();

        // reset encoders, gyro, & timer
        m_leftCount.reset();
        m_rightCount.reset();
        m_gyro.reset();
        timer.reset();

        //turn off all motors
        m_angle.set(0);
        m_gun1.set(0);
        m_gun2.set(0);
        m_gun3.set(0);
        m_robotDrive.tankDrive(0, 0);

        // clear solenoids to a known SAFE state
        m_hopperSolenoid.set(false);  //retracted: down position
        m_shooterSolenoid.set(false); //retracted
        m_scoopSolenoid.set(false);   //retracted
        m_hangSolenoid.set(false);    //retracted
        gunOn = false;
        m_found = false;

//        SmartDashboard.putString("System State", "Initialize Complete..."
        System.out.println("RobotInit() completed.\n");
        System.out.println("printSec =" + printSec);
        System.out.println("startSec =" + startSec);
    }

    //****************************************************************
//*****************   DISABLED STATES   ********************
//****************************************************************
    //**   This function is called once on entering Disabled mode   ***/
    public void disabledInit() {
        boolean Auton = false;
        System.out.println("Disabled Init\n");
        SmartDashboard.putString("System State", "Disabled");
        SmartDashboard.putData("Autonomous mode chooser", autoMode);

        m_airCompressor.start();
        m_leftCount.reset();  // reset encoder counts
        m_rightCount.reset();
        m_gyro.reset();

        //Safe the solendoids
        m_hopperSolenoid.set(false);  //retracted: down position
        m_shooterSolenoid.set(false); //retracted
        m_scoopSolenoid.set(false); //retracted
        m_hangSolenoid.set(false); //retracted

        m_robotDrive.tankDrive(0, 0);
        m_gun1.set(0);
        m_gun2.set(0);
        m_gun3.set(0);
        m_angle.set(0);

        m_autoAim = false;
        SmartDashboard.putBoolean("Auto_Aim", m_autoAim);
        gunOn = false;
        SmartDashboard.putBoolean("Gun", gunOn);
    }
    //**   This function is called periodically during disabled state   ***/

    public void disabledPeriodic() {
        //get auton mode
        AutonMode = (Integer) autoMode.getSelected();
        SmartDashboard.putInt("AUTON MODE: ", AutonMode.intValue());

        //get target info from driverstation
        m_robotDrive.tankDrive(0, 0);

        try {
            azimuth = targetTable.getNumber("Az");
            m_targetElevation = targetTable.getNumber("El");
            m_found = targetTable.getBoolean("found");
        } catch (TableKeyNotDefinedException e) {
            System.out.println("undefined table key exception in Disabled Periodic ");
        }
        // update status info
        if (m_found) {
            SmartDashboard.putString("Target", "Acquired");
        } else {
            SmartDashboard.putString("Target", "Looking");
        }
        SmartDashboard.putNumber("Az", azimuth);
        SmartDashboard.putNumber("El", m_targetElevation);

        // output info for debug
        if ((Timer.getFPGATimestamp()) > printSec) {
            System.out.println("Target Found = " + m_found);
            System.out.println("Az = " + azimuth);
            System.out.println("El = " + m_targetElevation);
            System.out.println("printSec =" + printSec);
            printSec++;
        }
    }

//****************************************************************
//*****************   AUTONOMOUS Operations   ********************
//****************************************************************
//
    //*** This function is called once on entering Autonomous Operations   **/
    public void autonomousInit() {
        // set Autonomous mode based on SmartDashboard selection
        AutonMode = (Integer) autoMode.getSelected();
        autonStep = 1;
        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);
        System.out.println("AutoInit started\n");
        SmartDashboard.putString("System State", "Auto");

        m_gyro.reset();
        m_hopperSolenoid.set(false); //retracted
        m_shooterSolenoid.set(false); //retracted
        m_scoopSolenoid.set(false); //retracted
        m_hangSolenoid.set(false); //retracted
        //all motors off
        m_angle.set(0);
        m_gun1.set(0);
        m_gun2.set(0);
        m_gun3.set(0);
        SmartDashboard.putBoolean("Gun", false);

        Timer.delay(.01);   // give some time for the encoders and gyro to reset before starting them

        m_leftCount.start();
        m_rightCount.start();
        m_robotDrive.tankDrive(0, 0);

        // set Autonomous mode based on SmartDashboard selection
        //     AutonMode = (Integer) autoMode.getSelected();

    }

    //**   This function is called periodically during autonomous   ***/
    public void autonomousPeriodic() {
        double R = 0;
        double L = 0;

        switch (AutonMode.intValue()) {
            case 1: //Center Pos 5 disks 
                //shoot 3, drive FWD & pick up 2, backup and shoot 
                switch (autonStep) {
                    case 1: //read Dasshboard
                        try {
                            azimuth = targetTable.getNumber("Az");
                            m_targetElevation = targetTable.getNumber("El");
                            m_found = targetTable.getBoolean("found");
                        } catch (TableKeyNotDefinedException e) {
                            System.out.println("undefined table key exceptionin Autonomous Periodic");
                            azimuth = 0;
                            m_targetElevation = 0;
                            m_found = false;
                        }
                        //send data to SmartDashboard for debug
                        if (m_found) {
                            SmartDashboard.putString("Target", "Acquired");
                        } else {
                            SmartDashboard.putString("Target", "Looking");
                        }

                        //Aim
                        if (m_found && (m_targetElevation + m_elevOffset) < -.25
                                && m_elevLowerLimit.get()) {       // target is higher higher elevation is aim lower
                            m_angle.set(.75);
                        } else if (m_found && (m_targetElevation + m_elevOffset) > .25
                                && m_elevUpperLimit.get()) {  // target is lower
                            m_angle.set(-.75);
                        } else {
                            m_angle.set(0);
                            autonStep = 2;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 2://step 2 shoot
                        m_hopperSolenoid.set(true); //extended: armed position
                        m_gun1.set(-1);
                        m_gun2.set(-1);
                        m_gun3.set(-1);
                        gunOn = true;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        Timer.delay(2.0); //wait till gun up to speed
                        m_shooterSolenoid.set(true); // shoot 4 times
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(1);
                        m_shooterSolenoid.set(true);//shoot 2nd
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(1);
                        m_shooterSolenoid.set(true);//shoot 3nd
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(1);
                        m_shooterSolenoid.set(true);//shoot 4th
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(.2);
                        autonStep = 3;
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);
                        break;

                    case 3: // gun off, hopper down and prepare to move
                        m_hopperSolenoid.set(false); // lower hopper
                        m_gun1.set(0);
                        m_gun2.set(0);
                        m_gun3.set(0);
                        gunOn = false;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        m_intake.set(-1);  // ON
                        m_scoopSolenoid.set(true);
                        autonStep = 4;
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);
                        break;

                    case 4: //drive fwd 
                        if (m_rightCount.getDistance() < 138) { // drive for 138 in
                            m_robotDrive.tankDrive(-.5, -.5);
                        } else {
                            m_robotDrive.tankDrive(0, 0);
                            Timer.delay(2);
                            autonStep = 5;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 5: //drive back
                        if (m_rightCount.getDistance() > 0) { // drive back to starting position
                            m_robotDrive.tankDrive(.5, .5);
                        } else {
                            m_robotDrive.tankDrive(0, 0);
                            autonStep = 7;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 7: //adjust azimuth & elevation
                        try {
                            azimuth = targetTable.getNumber("Az");
                            m_targetElevation = targetTable.getNumber("El");
                            m_found = targetTable.getBoolean("found");
                        } catch (TableKeyNotDefinedException e) {
                            System.out.println("undefined table key exception");
                            azimuth = 0;
                            m_targetElevation = 0;
                            m_found = false;
                        }
                        if ((m_targetElevation + m_elevOffset) > .25 || (m_targetElevation + m_elevOffset) < -.25
                                || (azimuth + 1) < -2 || (azimuth + 1) > 2) { // vision tack until shooter is aimed

                            if (m_found) {
                                SmartDashboard.putString("Target", "Acquired");
                            } else {
                                SmartDashboard.putString("Target", "Looking");
                            }

                            if (m_found && (m_targetElevation + m_elevOffset) < .25 && (m_targetElevation + m_elevOffset) > -.25
                                    || !m_elevLowerLimit.get() || !m_elevUpperLimit.get()) {
                                m_angle.set(0);
                            } else if (m_found && (m_targetElevation + m_elevOffset) < -.25
                                    && m_elevLowerLimit.get()) {       // target is higher
                                m_angle.set(.75);
                            } else if (m_found && (m_targetElevation + m_elevOffset) > .25
                                    && m_elevUpperLimit.get()) {  // target is lower
                                m_angle.set(-.75);
                            }
                            if (m_found && (azimuth + 1) < -2) {       // target to the left
                                m_robotDrive.tankDrive(.55, -.55);
                            } else if (m_found && (azimuth + 1) > 2) {       // target to the right
                                m_robotDrive.tankDrive(-.55, .55);
                            } else if (m_found && ((azimuth + 1) > -2) && ((azimuth + 1) < 2)) {
                                m_robotDrive.tankDrive(0, 0);       // on target 
                            }
                        } else {
                            autonStep = 8;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 8:  //shoot
                        m_intake.set(0);  // OFF
                        m_scoopSolenoid.set(false);
                        m_robotDrive.tankDrive(0, 0);
                        m_angle.set(0);
                        m_hopperSolenoid.set(true); //extended: armed position
                        m_gun1.set(-1);
                        m_gun2.set(-1);
                        m_gun3.set(-1);
                        gunOn = true;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        Timer.delay(1);
                        int i = 3;
                        if (i > 0) {
                            m_shooterSolenoid.set(true); // shoot 3 times
                            Timer.delay(.1);
                            m_shooterSolenoid.set(false);
                            Timer.delay(1);
                            i--;
                        } else {
                            m_gun1.set(0);
                            m_gun2.set(0);
                            m_gun3.set(0);
                            gunOn = false;
                                    SmartDashboard.putBoolean("Gun", gunOn);

                        }
                        break;
                    default:
                        m_gun1.set(0);
                        m_gun2.set(0);
                        m_gun3.set(0);
                        gunOn = false;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        m_shooterSolenoid.set(false);
                        m_intake.set(0);  // OFF
                        m_scoopSolenoid.set(false); //up
                        m_robotDrive.tankDrive(0, 0);
                        m_angle.set(0);
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + ".Default");
                        break;
                }

            case 2: //Center Pos 4 disks 
                //shoot 3, drive FWD & pick up 1, backup and shoot 
                switch (autonStep) {
                    //shoot 3, drive FWD & pick up 2, backup and shoot 
                    case 1: //read Dasshboard
                        try {
                            azimuth = targetTable.getNumber("Az");
                            m_targetElevation = targetTable.getNumber("El");
                            m_found = targetTable.getBoolean("found");
                        } catch (TableKeyNotDefinedException e) {
                            System.out.println("undefined table key exceptionin Autonomous Periodic");
                            azimuth = 0;
                            m_targetElevation = 0;
                            m_found = false;
                        }
                        //send data to SmartDashboard for debug
                        if (m_found) {
                            SmartDashboard.putString("Target", "Acquired");
                        } else {
                            SmartDashboard.putString("Target", "Looking");
                        }

                        //Aim
                        if (m_found && (m_targetElevation + m_elevOffset) < -.25
                                && m_elevLowerLimit.get()) {       // target is higher higher elevation is aim lower
                            m_angle.set(.75);
                        } else if (m_found && (m_targetElevation + m_elevOffset) > .25
                                && m_elevUpperLimit.get()) {  // target is lower
                            m_angle.set(-.75);
                        } else {
                            m_angle.set(0);
                            autonStep = 2;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 2://step 2 shoot
                        m_hopperSolenoid.set(true); //extended: armed position
                        m_gun1.set(-1);
                        m_gun2.set(-1);
                        m_gun3.set(-1);
                        gunOn = true;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        Timer.delay(2.0); //wait till gun up to speed
                        m_shooterSolenoid.set(true); // shoot 4 times
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(1);
                        m_shooterSolenoid.set(true);//shoot 2nd
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(1);
                        m_shooterSolenoid.set(true);//shoot 3nd
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(1);
                        m_shooterSolenoid.set(true);//shoot 4th
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(.2);
                        autonStep = 3;
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);
                        break;

                    case 3: // gun off, hopper down and prepare to move
                        m_hopperSolenoid.set(false); // lower hopper
                        m_gun1.set(0);
                        m_gun2.set(0);
                        m_gun3.set(0);
                        gunOn = false;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        m_intake.set(-1);  // ON
                        m_scoopSolenoid.set(true);
                        autonStep = 4;
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);
                        break;

                    case 4: //drive fwd 
                        if (m_rightCount.getDistance() < 54) { // drive for 42 in
                            m_robotDrive.tankDrive(-.5, -.5);
                        } else {
                            m_robotDrive.tankDrive(0, 0);
                            Timer.delay(2);
                            autonStep = 5;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 5: //drive back
                        if (m_rightCount.getDistance() > 0) { // drive back to starting position
                            m_robotDrive.tankDrive(.5, .5);
                        } else {
                            m_robotDrive.tankDrive(0, 0);
                            autonStep = 7;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 7: //adjust azimuth & elevation
                        try {
                            azimuth = targetTable.getNumber("Az");
                            m_targetElevation = targetTable.getNumber("El");
                            m_found = targetTable.getBoolean("found");
                        } catch (TableKeyNotDefinedException e) {
                            System.out.println("undefined table key exception");
                            azimuth = 0;
                            m_targetElevation = 0;
                            m_found = false;
                        }
                        if ((m_targetElevation + m_elevOffset) > .25 || (m_targetElevation + m_elevOffset) < -.25
                                || (azimuth + 1) < -2 || (azimuth + 1) > 2) { // vision tack until shooter is aimed

                            if (m_found) {
                                SmartDashboard.putString("Target", "Acquired");
                            } else {
                                SmartDashboard.putString("Target", "Looking");
                            }

                            if (m_found && (m_targetElevation + m_elevOffset) < .25 && (m_targetElevation + m_elevOffset) > -.25
                                    || !m_elevLowerLimit.get() || !m_elevUpperLimit.get()) {
                                m_angle.set(0);
                            } else if (m_found && (m_targetElevation + m_elevOffset) < -.25
                                    && m_elevLowerLimit.get()) {       // target is higher
                                m_angle.set(.75);
                            } else if (m_found && (m_targetElevation + m_elevOffset) > .25
                                    && m_elevUpperLimit.get()) {  // target is lower
                                m_angle.set(-.75);
                            }
                            if (m_found && (azimuth + 1) < -2) {       // target to the left
                                m_robotDrive.tankDrive(.55, -.55);
                            } else if (m_found && (azimuth + 1) > 2) {       // target to the right
                                m_robotDrive.tankDrive(-.55, .55);
                            } else if (m_found && ((azimuth + 1) > -2) && ((azimuth + 1) < 2)) {
                                m_robotDrive.tankDrive(0, 0);       // on target 
                            }
                        } else {
                            autonStep = 8;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 8:  //shoot
                        m_intake.set(0);  // OFF
                        m_scoopSolenoid.set(false);
                        m_robotDrive.tankDrive(0, 0);
                        m_angle.set(0);
                        m_hopperSolenoid.set(true); //extended: armed position
                        m_gun1.set(-1);
                        m_gun2.set(-1);
                        m_gun3.set(-1);
                        gunOn = true;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        Timer.delay(1);
                        int i = 3;
                        if (i > 0) {
                            m_shooterSolenoid.set(true); // shoot 3 times
                            Timer.delay(.1);
                            m_shooterSolenoid.set(false);
                            Timer.delay(1);
                            i--;
                        } else {
                            m_gun1.set(0);
                            m_gun2.set(0);
                            m_gun3.set(0);
                            gunOn = false;
                                    SmartDashboard.putBoolean("Gun", gunOn);

                        }
                        break;
                    default:
                        m_gun1.set(0);
                        m_gun2.set(0);
                        m_gun3.set(0);
                        gunOn = false;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        m_shooterSolenoid.set(false);
                        m_intake.set(0);  // OFF
                        m_scoopSolenoid.set(false); //up
                        m_robotDrive.tankDrive(0, 0);
                        m_angle.set(0);
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + ".Default");
                        break;
                }
            case 3: //Side Pos 2 disks 
                //shoot 2 and don;t move 
                switch (autonStep) {
                    case 1: //read Dasshboard
                        try {
                            azimuth = targetTable.getNumber("Az");
                            m_targetElevation = targetTable.getNumber("El");
                            m_found = targetTable.getBoolean("found");
                        } catch (TableKeyNotDefinedException e) {
                            System.out.println("undefined table key exceptionin Autonomous Periodic");
                            azimuth = 0;
                            m_targetElevation = 0;
                            m_found = false;
                        }
                        //send data to SmartDashboard for debug
                        if (m_found) {
                            SmartDashboard.putString("Target", "Acquired");
                        } else {
                            SmartDashboard.putString("Target", "Looking");
                        }

                        //Aim
                        if (m_found && (m_targetElevation + m_elevOffset) < -.25
                                && m_elevLowerLimit.get()) {       // target is higher higher elevation is aim lower
                            m_angle.set(.75);
                        } else if (m_found && (m_targetElevation + m_elevOffset) > .25
                                && m_elevUpperLimit.get()) {  // target is lower
                            m_angle.set(-.75);
                        } else {
                            m_angle.set(0);
                            autonStep = 2;
                            System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);

                        }
                        break;

                    case 2://step 2 shoot
                        m_hopperSolenoid.set(true); //extended: armed position
                        m_gun1.set(-1);
                        m_gun2.set(-1);
                        m_gun3.set(-1);
                        gunOn = true;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        Timer.delay(2.0); //wait till gun up to speed
                        m_shooterSolenoid.set(true); // shoot 4 times
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(1);
                        m_shooterSolenoid.set(true);//shoot 2nd
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(1);
                        m_shooterSolenoid.set(true);//shoot 3nd
                        Timer.delay(.1);
                        m_shooterSolenoid.set(false);
                        Timer.delay(.2);
                        autonStep = 3;
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);
                        break;

                    case 3: // gun off, hopper down and prepare to move
                        m_hopperSolenoid.set(false); // lower hopper
                        m_gun1.set(0);
                        m_gun2.set(0);
                        m_gun3.set(0);
                        gunOn = false;
                                SmartDashboard.putBoolean("Gun", gunOn);
                        m_intake.set(-1);  // ON
                        m_scoopSolenoid.set(true);
                        //                        autonStep = 3;
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + "." + autonStep);
                        break;

                    default:
                        m_gun1.set(0);
                        m_gun2.set(0);
                        m_gun3.set(0);
                        gunOn = false;
                        m_shooterSolenoid.set(false);
                        m_intake.set(0);  // OFF
                        m_scoopSolenoid.set(false); //up
                        m_robotDrive.tankDrive(0, 0);
                        m_angle.set(0);
                        System.out.println("Auton Mode.Step = " + AutonMode.intValue() + ".Default");
                        break;
                }

        }
        SmartDashboard.putNumber("Az", azimuth);
        SmartDashboard.putNumber("El", m_targetElevation);

        //output debug info
        SmartDashboard.putNumber("Left Encoder count", m_leftCount.getRaw());
        SmartDashboard.putNumber("Left Encoder distance", m_leftCount.getDistance());
        SmartDashboard.putNumber("Right Encoder count", m_rightCount.getRaw());
        SmartDashboard.putNumber("Right Encoder distance", m_rightCount.getDistance());
        SmartDashboard.putNumber("gyro", m_gyro.getAngle());
        SmartDashboard.putBoolean("Intake", m_scoopSolenoid.get());
        SmartDashboard.putBoolean("Gun", gunOn);
    }

//****************************************************************
//*****************   TELEOP Operations   ************************
//****************************************************************
//
    //*** This function is called once on entering Teleop Operations   **/
    public void teleopInit() {

        SmartDashboard.putString("System State", "Telop");
        System.out.println("telop init");
        m_leftCount.start();  // needed for debug
        m_rightCount.start();
        timer.start();
        // all motors off
        m_angle.set(0);
        m_gun1.set(0);
        m_gun2.set(0);
        m_gun3.set(0);
        m_robotDrive.tankDrive(0, 0);

        //Safe the solendoids
        m_hopperSolenoid.set(false);  //retracted: down position
        m_shooterSolenoid.set(false); //retracted
        m_scoopSolenoid.set(false);   //retracted
        m_hangSolenoid.set(false);    //retracted
        m_autoAim = false;
        SmartDashboard.putBoolean("Intake", false);

        button8Previous = false;
        gunOn = false;
    }

    /**
     * This function is called periodically during operator control **
     */
    public void teleopPeriodic() {
        //get target info from DriverStation Vision Code
        try {
            azimuth = targetTable.getNumber("Az");
            m_targetElevation = targetTable.getNumber("El");
            m_found = targetTable.getBoolean("found");
        } catch (TableKeyNotDefinedException e) {
            System.out.println("undefined table key exceptionin Teleop Periodic");
        }
        if (m_found) {
            SmartDashboard.putString("Target", "Acquired");
        } else {
            SmartDashboard.putString("Target", "Looking");
        }

        // ************************ Drive Stick ************************
        // Intake Control
        if (m_driveStick.getRawButton(5)) {
            m_intake.set(-1);  // ON
            m_scoopSolenoid.set(true);
            SmartDashboard.putBoolean("Intake", true);
        } else if (m_driveStick.getRawButton(7)) {
            m_intake.set(0);  // OFF
            m_scoopSolenoid.set(false);
            SmartDashboard.putBoolean("Intake", false);
        }
        if (m_driveStick.getRawButton(9)) {
            m_intake.set(1);// REVERSE
            SmartDashboard.putBoolean("Intake", true);
        }

        // AutoAim Selection
        if (m_driveStick.getRawButton(6)) {
            m_autoAim = true;
        } else if (m_driveStick.getRawButton(8)) {
            m_autoAim = false;

        }
        SmartDashboard.putBoolean("Auto_Aim", m_autoAim);

        if (!m_autoAim) {   //manual control
            // drive control with deadband      
            double LeftSpeed = m_driveStick.getRawAxis(2);
            double RightSpeed = m_driveStick.getRawAxis(4);
            if ((LeftSpeed < .05) && (LeftSpeed > -0.05)) { //5% is approx 6 counts
                LeftSpeed = 0;
            }
            if ((RightSpeed < .05) && (RightSpeed > -0.05)) {
                RightSpeed = 0;
            }
            m_robotDrive.tankDrive(LeftSpeed, RightSpeed);	// drive with tank style

        } //auto_aim for target
        else {
            if (m_found && (azimuth + 1) < -2) {       // target to the left
                m_robotDrive.tankDrive(.53, -.53);
            } else if (m_found && (azimuth + 1) > 2) {       // target to the right
                m_robotDrive.tankDrive(-.53, .53);
            } else if (m_found && ((azimuth + 1) > -2) && ((azimuth + 1) < 2)) {
                m_robotDrive.arcadeDrive(0, 0);       // on target 
            }
        }
        //************************ Device Stick ***********************//
        // angle control with deadband   
        if (!m_autoAim) {   //manual control
            double angleSpeed = m_deviceStick.getRawAxis(4);
//            System.out.println("anglespeed = " + angleSpeed);
            //           System.out.println("upperlimit = " + m_elevUpperLimit.get());

            if ((angleSpeed < 0 && !m_elevUpperLimit.get()) // joystick values opposite of direction
                    || (angleSpeed > 0 && !m_elevLowerLimit.get())) {
//                System.out.println("limit");
                angleSpeed = 0;
            }
            if (angleSpeed < .2 && angleSpeed > -0.2) { //deadband
                angleSpeed = 0;
            }
//                System.out.println("upper ="+ m_elevUpperLimit.get());                
//                System.out.println("lower ="+ m_elevLowerLimit.get());
            m_angle.set(angleSpeed);

//            System.out.println("anglespeed_set = " + angleSpeed);
        } //auto_aim for target
        else {
            if (m_found && (m_targetElevation + 5.7) < -.25
                    && m_elevLowerLimit.get()) {       // target is higher
                m_angle.set(.75);
            } else if (m_found && (m_targetElevation + 5.7) > .25
                    && m_elevUpperLimit.get()) {  // target is lower
                m_angle.set(-.75);
            } else {
                m_angle.set(0);       // on target 
            }
        }

        // Arm the shooter: intake roller off, hopper up, gun on
        if (m_deviceStick.getRawButton(5)) {
            //           m_intake.set(0);  // OFF
            m_hopperSolenoid.set(true); // Up
            gunOn = true;
        } else if (m_deviceStick.getRawButton(7)) {
            //       m_intake.set(-1);  // On
            m_hopperSolenoid.set(false); // down
            gunOn = false;
        }

        //hopper control
        if (m_deviceStick.getRawButton(4)) {
            m_hopperSolenoid.set(true);
        }
        if (m_deviceStick.getRawButton(1)) {
            m_hopperSolenoid.set(false);
        }

        // Shoot
        if (m_deviceStick.getRawButton(6)) {
            m_shooterSolenoid.set(true); // fire
        } else {
            m_shooterSolenoid.set(false); // retract
        }

        // Turn gun on/off
        button8 = m_deviceStick.getRawButton(8);
        if (button8 && !button8Previous) { //button pushed
            gunOn = !gunOn;
        }
        button8Previous = button8;
        SmartDashboard.putBoolean("Gun", gunOn);
        //Gun motor control
        if (gunOn) {
            m_gun1.set(-1);
            m_gun2.set(-1);
            m_gun3.set(-1);
        } else {
            m_gun1.set(0);
            m_gun2.set(0);
            m_gun3.set(0);
        }

        //Hang control
        if (m_deviceStick.getRawAxis(6) == -1) {
            m_hangSolenoid.set(true); //extend
        }

        if (m_deviceStick.getRawAxis(6) == 1) {
            m_hangSolenoid.set(false); //retract
        }

        //update status
        SmartDashboard.putNumber("Left Encoder count", m_leftCount.getRaw());
        SmartDashboard.putNumber("Left Encoder distance", m_leftCount.getDistance());
        SmartDashboard.putNumber("Right Encoder count", m_rightCount.getRaw());
        SmartDashboard.putNumber("Right Encoder distance", m_rightCount.getDistance());
        SmartDashboard.putNumber("gyro", m_gyro.getAngle());
        //send data to SmartDashboard for debug
        SmartDashboard.putNumber("Az", azimuth);
        SmartDashboard.putNumber("El", m_targetElevation);

        if ((Timer.getFPGATimestamp()) > printSec) {
            System.out.println("Target Found = " + m_found);
            System.out.println("Az = " + azimuth);
            System.out.println("El = " + m_targetElevation);
            System.out.println("printSec =" + printSec);
            printSec++;
        }
    }
//****************************************************************
//*****************   TEST MODE Operations   ************************
//****************************************************************
//
//**   This function is called periodically during test mode   ***/

    public void testPeriodic() {
        boolean Auton = false;
        SmartDashboard.putString("System State", "Test");
    }
//
}
