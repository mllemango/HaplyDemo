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
float             check1                              =0;
float             check2                              =0;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haply_avatar;
PImage eagle;
PImage home;
/* ground*/
FBox obs1;
FBox obs2;
FBox obs3;
FBox obs4;
FBox obs5;
FBox obs_1;
FBox obs_2;
FBox obs_3;
FBox obs_4;
FBox obs_5;
FBox beet;

/* background shapes */
PImage img;

/* end elements definition *********************************************************************************************/

/* setup section *******************************************************************************************************/
void setup() {

  // Parameters go inside the parentheses when the object is constructed.
  
  /* put setup code here, run once: */

  /* screen size definition */
  size(1000, 400);
  img = loadImage("img/dirt.jpg");
  background(img);
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
   print(Serial.list());

  haplyBoard          = new Board(this, Serial.list()[0], 0);
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



  /* Setup obstacles */
  obs1 = new FBox(1, 7);
 // obs1.attachImage(loadImage("img/maze.png"));
  obs1.setPosition(worldWidth-2, 2);
  obs1.setFill(0, 0, 0);
  obs1.setStatic(true);
  world.add(obs1);
  
  
  eagle = loadImage("img/eagle.png");
  
   obs2 = new FBox(10,10);
   //obs2.vertex(1, 2);
  obs2.attachImage(eagle);
  obs2.setHeight(0.5);
  obs2.setWidth(0.5);
  obs2.setPosition(worldWidth-5, -2);
  obs2.setVelocity(0,0.00001);
  obs2.setDamping(200);
  obs2.setFill(0, 0, 0);
  obs2.setFriction(1000);
  obs2.setDensity(1000);
  obs2.setForce(0,3000);
  world.add(obs2);
  
  home = loadImage("img/fembeet.png");
  beet = new FBox(3,3);
  beet.attachImage(home);
  beet.setPosition(worldWidth-2,8);
  beet.setStatic(true);
  world.add(beet);
  
   obs3 = new FBox(1, 5);
 // obs1.attachImage(loadImage("img/maze.png"));
  obs3.setPosition(worldWidth-5, 9);
  obs3.setFill(0, 0, 0);
  
  obs3.setVelocity(0,0.007);
  obs3.setStatic(true);
  //world.add(obs3);
  
   obs1 = new FBox(1, 8);
 // obs1.attachImage(loadImage("img/maze.png"));
  obs1.setPosition(worldWidth-8, 8);
  obs1.setFill(0, 0, 0);
  obs1.setStatic(true);
  world.add(obs1);
  
  obs1 = new FBox(1, 10);
 // obs1.attachImage(loadImage("img/maze.png"));
  obs1.setPosition(worldWidth-11, 2);
  obs1.setFill(0, 0, 0);
  obs1.setStatic(true);
  world.add(obs1);
  
  
  ////////////
  
   obs1 = new FBox(1, 8);
 // obs1.attachImage(loadImage("img/maze.png"));
  obs1.setPosition(worldWidth-22, 2);
  obs1.setFill(0, 0, 0);
  obs1.setStatic(true);
  world.add(obs1);
  
  obs1 = new FBox(1, 4);
  //obs1.attachImage(loadImage("img/maze.png"));
  obs1.setPosition(worldWidth-20, 8);
  obs1.setFill(0, 0, 0);
  obs1.setStatic(true);
  world.add(obs1);
  obs1.isHaptic();
  
  obs1 = new FBox(1, 3);
 // obs1.attachImage(loadImage("img/maze.png"));
  obs1.setPosition(worldWidth-20, 2);
  obs1.setFill(0, 0, 0);
  obs1.setStatic(true);
  world.add(obs1);
  
  obs1 = new FBox(1, 9);
 // obs1.attachImage(loadImage("img/maze.png"));
  obs1.setPosition(worldWidth-17, 8);
  obs1.setFill(0, 0, 0);
  obs1.setStatic(true);
  world.add(obs1);

 obs_2 = new FBox(1,7);
 // obs1.attachImage(loadImage("img/maze.png"));
  obs_2.setPosition(worldWidth-16, 2);
  obs_2.setFill(160, 82, 45);
  obs_2.addTorque(1);
  obs_2.setRestitution(0.5);
  obs_2.setFriction(0.5);
  obs_2.setDensity(2);
 // obs1.setVelocity(0,0);
  world.add(obs_2);
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  haply_avatar = loadImage("img/dungbeetle_cut.png"); 
  haply_avatar.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
  s                   = new HVirtualCoupling((0.5)); 
  s.h_avatar.attachImage(haply_avatar); 
  s.h_avatar.setSize(0.5);
  s.h_avatar.setDensity(100); 
  s.h_avatar.setFill(255, 255, 255);
  s.init(world, edgeTopLeftX +0.9,edgeTopLeftY+5); 

  /* World conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.5);
  world.setEdgesFriction(0.5);
  
/*  square_light = new FBox(12.5,9);
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
*/
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
  background(img);
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
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
    
    
    /* Set up up and down obstacle*/
    
  
    if (obs2.getY() > 8) {
      obs2.setPosition(worldWidth-5, 0);
      obs2.setVelocity(0,-0.00001);
    }
    
    if(s.h_avatar.isTouchingBody(obs2)){
      print("touching");
     // s.h_avatar.setDamping(100);
     // s.h_avatar.setHapticStiffness(25000);
      obs_2.setPosition(worldWidth-16, 2);
      s.setAvatarPosition(edgeTopLeftX +0.9,edgeTopLeftY+5);
      
    }

    world.step(1.0f/1000.0f);

    rendering_force = false;

  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
