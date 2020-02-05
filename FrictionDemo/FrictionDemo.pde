/**
 **********************************************************************************************************************
 * @file       Haptic_Physics_Template.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       27-September-2018
 * @brief      Base project template for use with pantograph 2-DOF device and 2-D physics engine
 *             creates a blank world ready for creation
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/



/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haply_avatar;

/* ground*/
FBox obs1;
FBox obs2;
FCircle ball;
PImage bg;

/* background shapes */
FBox square_light;
FBox square_dense;

/* end elements definition *********************************************************************************************/

/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1000, 400);
  bg = loadImage("img/forestfloor.jpg");

  /* device setup */

  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[1], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CW, 1);
  widgetOne.add_actuator(2, CW, 2);

  widgetOne.add_encoder(1, CW, 180, 13824, 1);
  widgetOne.add_encoder(2, CW, 0, 13824, 2);

  widgetOne.device_set_parameters();


  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();



// Ball

  ball = new FCircle(1);
  ball.setPosition(edgeTopLeftX+worldWidth/2, edgeTopLeftY+2);
  ball.setFill(150,75,0);
  ball.setDamping(600);
  world.add(ball);
  
  /* Setup obstacles */
  obs1 = new FBox(3, 2);
  obs2 = new FBox(2, 1);
  obs1.setPosition(worldWidth*1/3, worldHeight*1/3);
  obs2.setPosition(worldWidth-5, 2);
  obs1.setRotation(3);
  obs1.setFill(0, 0, 0);
  obs2.setFill(0 ,0, 0);
  obs1.setStatic(true);
  obs2.setStatic(true);
  
  world.add(obs1);
  world.add(obs2);


  /* Setup the Virtual Coupling Contact Rendering Technique */
  haply_avatar = loadImage("img/dungbeetle_cut.png"); 
  haply_avatar.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
  s                   = new HVirtualCoupling((0.5)); 
  s.h_avatar.attachImage(haply_avatar); 
  s.h_avatar.setSize(0.5);
  s.h_avatar.setDensity(400); 
  s.h_avatar.setFill(255, 255, 255);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  square_light = new FBox(12.5,9);
  square_light.setPosition(3*worldWidth/4, worldHeight/2);
  square_light.setFill(0, 0, 0, 0);
  square_light.setDensity(100);
  square_light.setSensor(true);
  square_light.setNoStroke();
  square_light.setStatic(true);
  world.add(square_light);
  
  square_dense = new FBox(12.5,9);
  square_dense.setPosition(1*worldWidth/4, worldHeight/2);
  square_dense.setFill(0, 0, 0, 0);
  square_dense.setDensity(100);
  square_dense.setSensor(true);
  square_dense.setNoStroke();
  square_dense.setStatic(true);
  world.add(square_dense);

  world.draw();


  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}



/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  background(bg);
  world.draw();
  
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();

      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));
    }
    //println(pos_ee);

    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-7); 
    s.updateCouplingForce();
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.div(20000); //
    
    if (s.h_avatar.isTouchingBody(square_light)){
      s.h_avatar.setDamping(10);
      ball.setDamping(10);
      s.h_avatar.setHapticStiffness(25000);
      //println("inside square light");
    } else{
      s.h_avatar.setDamping(350);
      ball.setDamping(350);
      s.h_avatar.setHapticStiffness(25000);
      //println("inside square dense");
    }
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);

    rendering_force = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
