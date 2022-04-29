package creaturerevisited;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.TextureKey;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.MultiBodyAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.texture.Texture;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import jme3utilities.minie.FilterAll;

/**
 * This is the Main Class of your Game. You should only do initialization here.
 * Move your Logic into AppStates or Controls
 * @author normenhansen
 */
public class Main extends SimpleApplication implements ActionListener{
    
    private RigidBodyControl    floor_phy;
    private RigidBodyControl    wall01_phy;
    private RigidBodyControl    wall02_phy;
    private RigidBodyControl    wall03_phy;
    private RigidBodyControl    wall04_phy;
    private Box    floor;
    private Box    wall01;
    private Box    wall02;
    private Box    wall03;
    private Box    wall04;
    Material wall_mat;
    Material stone_mat;
    Material floor_mat;
    Material redmat;
    Material greenmat;
    Material bluemat;
    Material yellowmat;
    Material orangemat;
    Material brownmat;
    
    boolean canlog = false;
    public static Logger log;
    ArrayList<MJCreature> creatures;
    MJCreature playcreature;
    ArrayList<Material> allmaterials;
    private float time;
    private int generation = 0;
    private boolean logdata;
    
    public static Geometry target;
    private int indexrandpositions = 0;
    private ArrayList<Vector3f> randpositions = new ArrayList<Vector3f>();
    private int randpositionsize = 5;
    private int changetargettime = 30;
    private int checkfitnessmaxtime = Math.round((changetargettime * randpositionsize) / 6);
    private float checkfitnesstime = 0;
    
    
    private int state = 0;
    private boolean applyForceUp = false;
    private boolean applyForceDown = false;
    private boolean applyForceLeft = false;
    private boolean applyForceRight = false;
    
    
    
    public static void main(String[] args) {
        log = Logger.getLogger("BulletLog");  
        FileHandler fh;
        try {
            fh = new FileHandler("bulletlog.log");  
            log.addHandler(fh);
            CustomLogFormatter formatter = new CustomLogFormatter();  
            fh.setFormatter(formatter);  

        } catch (SecurityException e) {  
            e.printStackTrace();  
        } catch (IOException e) {  
            e.printStackTrace();  
        }  
        
        Main app = new Main();
        app.setPauseOnLostFocus(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {
        this.flyCam.setMoveSpeed(50f);
        this.getCamera().setLocation(new Vector3f(0,0, 50f));
        this.time = 0f;
        this.logdata = false;
        
        initMaterials();
         
        initEnvironment();
        
        registerInput();
        
        initRandPositions(false);
        
        LoadTarget();
        
        loadCreatures();
    }
    
    private void initRandPositions(boolean clear)
    {
        if (clear)
        {
            randpositions.clear();
        }
        
        for (int i = 0; i < randpositionsize; i++)
        {
            Vector3f randpos = new Vector3f(MJFastMath.nextRandomFloat()*300 - 150f,MJFastMath.nextRandomFloat()*10 -20.0f, MJFastMath.nextRandomFloat()*300 - 150f);
            randpositions.add(randpos);
        }
    }
    
    private void LoadTarget()
    {
        Box box = new Box(0.48f, 0.24f, 0.12f);
        box.scaleTextureCoordinates(new Vector2f(1f, .5f));
        
        target = new Geometry("Target", box);
        MJNode.target = Main.target;
        
        Material redmat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        redmat.setColor("Color", ColorRGBA.Blue);
        target.setMaterial(redmat);
        
        target.setLocalTranslation(randpositions.get(indexrandpositions));
        target.setLocalScale(5);
        
        rootNode.attachChild(target);
    }
    
    
    public void loadCreatures()
    {
        MJCreature newcreature01 = new MJCreature(rootNode, redmat);
        this.initFloor(newcreature01.GetPhysicState());

        MJCreature newcreature02 = new MJCreature(rootNode, greenmat);
        this.initFloor(newcreature02.GetPhysicState());
        
        MJCreature newcreature03 = new MJCreature(rootNode, bluemat);
        this.initFloor(newcreature03.GetPhysicState());
        
        MJCreature newcreature04 = new MJCreature(rootNode, orangemat);
        this.initFloor(newcreature04.GetPhysicState());
        
        MJCreature newcreature05 = new MJCreature(rootNode, yellowmat);
        this.initFloor(newcreature05.GetPhysicState());

        
        creatures.add(newcreature01);
        creatures.add(newcreature02);
        creatures.add(newcreature03);
        creatures.add(newcreature04);
        creatures.add(newcreature05);
        
        playcreature = new MJCreature(rootNode, brownmat);
        this.initFloor(newcreature05.GetPhysicState());
    }
    
    public void initMaterials() {
      allmaterials = new ArrayList<Material>();  
        
      wall_mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      TextureKey key = new TextureKey("Textures/Terrain/splat/dirt.jpg");
      key.setGenerateMips(true);
      Texture tex = assetManager.loadTexture(key);
      wall_mat.setTexture("ColorMap", tex);

      stone_mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      TextureKey key2 = new TextureKey("Textures/Terrain/splat/dirt.jpg");
      key2.setGenerateMips(true);
      Texture tex2 = assetManager.loadTexture(key2);
      stone_mat.setTexture("ColorMap", tex2);

      floor_mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      TextureKey key3 = new TextureKey("Textures/Terrain/splat/dirt.jpg");
      key3.setGenerateMips(true);
      Texture tex3 = assetManager.loadTexture(key3);
      tex3.setWrap(Texture.WrapMode.Repeat);
      floor_mat.setTexture("ColorMap", tex3);
      
      redmat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      redmat.setColor("Color", ColorRGBA.Red);
      
      greenmat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      greenmat.setColor("Color", ColorRGBA.Green);
      
      bluemat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      bluemat.setColor("Color", ColorRGBA.Blue);
      
      orangemat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      orangemat.setColor("Color", ColorRGBA.Orange);
      
      yellowmat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      yellowmat.setColor("Color", ColorRGBA.Yellow);
      
      brownmat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
      brownmat.setColor("Color", ColorRGBA.Brown);
      
      allmaterials.add(redmat);
      allmaterials.add(greenmat);
      allmaterials.add(bluemat);
      allmaterials.add(orangemat);
      allmaterials.add(yellowmat);
    }
    
    public void initEnvironment() {
        creatures = new ArrayList<MJCreature>();
        
        
        floor = new Box(240f, 0.1f, 240f);
        floor.scaleTextureCoordinates(new Vector2f(3, 6));
        
        wall01 = new Box(240f, 120.0f, 20.0f);
        wall01.scaleTextureCoordinates(new Vector2f(3, 6));
        
        wall02 = new Box(240f, 120.0f, 20.0f);
        wall02.scaleTextureCoordinates(new Vector2f(3, 6));
        
        wall03 = new Box(20.0f, 120.0f, 240f);
        wall03.scaleTextureCoordinates(new Vector2f(3, 6));
        
        wall04= new Box(20.0f, 120.0f, 240f);
        wall04.scaleTextureCoordinates(new Vector2f(3, 6));
        
        
        Geometry floor_geo = new Geometry("Floor", floor);
        floor_geo.setMaterial(floor_mat);
        floor_geo.setLocalTranslation(0, -20.0f, 0);
        this.rootNode.attachChild(floor_geo);

        floor_phy = new RigidBodyControl(0.0f);

        floor_geo.addControl(floor_phy);
        floor_geo.setShadowMode(RenderQueue.ShadowMode.Receive);

        Geometry wall01_geo = new Geometry("wall01", wall01);
        wall01_geo.setMaterial(floor_mat);
        wall01_geo.setLocalTranslation(0f, -20.0f, -240f);
        this.rootNode.attachChild(wall01_geo);

        wall01_phy = new RigidBodyControl(0.0f);

        wall01_geo.addControl(wall01_phy);
        wall01_geo.setShadowMode(RenderQueue.ShadowMode.Receive);

        Geometry wall02_geo = new Geometry("wall02", wall02);
        wall02_geo.setMaterial(floor_mat);
        wall02_geo.setLocalTranslation(0f, -20.0f, 240f);
        this.rootNode.attachChild(wall02_geo);

        wall02_phy = new RigidBodyControl(0.0f);

        wall02_geo.addControl(wall02_phy);
        wall02_geo.setShadowMode(RenderQueue.ShadowMode.Receive);

        Geometry wall03_geo = new Geometry("wall03", wall03);
        wall03_geo.setMaterial(floor_mat);
        wall03_geo.setLocalTranslation(-240f, -20.0f, 0);
        this.rootNode.attachChild(wall03_geo);

        wall03_phy = new RigidBodyControl(0.0f);

        wall03_geo.addControl(wall03_phy);
        wall03_geo.setShadowMode(RenderQueue.ShadowMode.Receive);

        Geometry wall04_geo = new Geometry("wall04", wall04);
        wall04_geo.setMaterial(floor_mat);
        wall04_geo.setLocalTranslation(240f, -20.0f, 0);
        this.rootNode.attachChild(wall04_geo);

        wall04_phy = new RigidBodyControl(0.0f);

        wall04_geo.addControl(wall04_phy);
        wall04_geo.setShadowMode(RenderQueue.ShadowMode.Receive);
      }
    
    
    public void initFloor(PhysicsSpace state) {
        Geometry floor_geo = new Geometry("Floor", floor);
        floor_geo.setMaterial(floor_mat);
        floor_geo.setLocalTranslation(0, -20.0f, 0);
        floor_phy = new RigidBodyControl(0.0f);

        floor_geo.addControl(floor_phy);
        state.getPhysicsSpace().add(floor_phy);


        Geometry wall01_geo = new Geometry("wall01", wall01);
        wall01_geo.setMaterial(floor_mat);
        wall01_geo.setLocalTranslation(0, -20.0f, -240f);
        wall01_phy = new RigidBodyControl(0.0f);

        wall01_geo.addControl(wall01_phy);
        state.getPhysicsSpace().add(wall01_phy);


        Geometry wall02_geo = new Geometry("wall02", wall02);
        wall02_geo.setMaterial(floor_mat);
        wall02_geo.setLocalTranslation(0, -20.0f, 240f);
        wall02_phy = new RigidBodyControl(0.0f);

        wall02_geo.addControl(wall02_phy);
        state.getPhysicsSpace().add(wall02_phy);


        Geometry wall03_geo = new Geometry("wall03", wall03);
        wall03_geo.setMaterial(floor_mat);
        wall03_geo.setLocalTranslation(-240, -20.0f, 0);
        wall03_phy = new RigidBodyControl(0.0f);

        wall03_geo.addControl(wall03_phy);
        state.getPhysicsSpace().add(wall03_phy);


        Geometry wall04_geo = new Geometry("wall04", wall04);
        wall04_geo.setMaterial(floor_mat);
        wall04_geo.setLocalTranslation(240f, -20.0f, 0f);
        wall04_phy = new RigidBodyControl(0.0f);

        wall04_geo.addControl(wall04_phy);
        state.getPhysicsSpace().add(wall04_phy);  
    }
    
    public void registerInput() {
        inputManager.addMapping("load creature",new KeyTrigger(keyInput.KEY_1));
        inputManager.addMapping("save creature",new KeyTrigger(keyInput.KEY_2));
        
        inputManager.addMapping("up",new KeyTrigger(keyInput.KEY_U));
        inputManager.addMapping("down",new KeyTrigger(keyInput.KEY_J));
        inputManager.addMapping("left",new KeyTrigger(keyInput.KEY_H));
        inputManager.addMapping("right",new KeyTrigger(keyInput.KEY_K));
        
        inputManager.addListener(this, "load creature");
        inputManager.addListener(this, "save creature");
        
        inputManager.addListener(this, "up");
        inputManager.addListener(this, "down");
        inputManager.addListener(this, "left");
        inputManager.addListener(this, "right");
    }
    

    @Override
    public void simpleUpdate(float tpf) {
        
        
        if (applyForceUp) {
            playcreature.GetRootWorm().GetBodyControl().applyForce(new Vector3f(0, 0, -10) , Vector3f.ZERO.clone()); 
        }
        if (applyForceDown) {
            playcreature.GetRootWorm().GetBodyControl().applyForce(new Vector3f(0, 0,10), Vector3f.ZERO.clone());  
        }
        if (applyForceLeft) {
            playcreature.GetRootWorm().GetBodyControl().applyForce(new Vector3f(-10, 0, 0), Vector3f.ZERO.clone());
        }
        if (applyForceRight) {
            playcreature.GetRootWorm().GetBodyControl().applyForce(new Vector3f(10, 0, 0), Vector3f.ZERO.clone());  
        }
        
        
        switch(state)
        {
            case 0: //init time
            {
                time += 0.01;
                
                for (int i = 0; i < creatures.size(); i++)
                {
                    creatures.get(i).tick(0.01f, false); //update physics with a fixed time
                }
                
                //ticking also the play creature
                playcreature.tick(0.01f, false);
                
                if (time > 10)
                {
                    System.out.println("Starting now...");
                    state = 1; //start simulation time
                    time = 0f;//restart simulation time
                    checkfitnesstime = 0; //start fitness time check
                }
                
                break;
            }
            case 1:
            {
                time += 0.01;
                checkfitnesstime +=  0.01;
                
                for (int i = 0; i < creatures.size() ; i++)
                {
                    creatures.get(i).tick(0.01f, true); //update physics with a fixed time
                }
                
                //ticking also the play creature
                playcreature.tick(0.01f, true);
            
                if ((Math.round(time) % changetargettime) == (changetargettime - 1))
                {
                    if (indexrandpositions < (randpositions.size() - 1))
                    {
                        indexrandpositions++;
                        target.setLocalTranslation(randpositions.get(indexrandpositions));
                        time = 0f;
                    }
                    else
                    {
                        System.out.println("End...");
                        //this.LogAll();
                        this.Evolve();
                        this.RestartSimulation();
                        state = 0;
                    }
                }

                if (Math.round(checkfitnesstime) > checkfitnessmaxtime)
                {
                    checkfitnesstime = 0f;

                    for (int i = 0; i < creatures.size(); i++)
                    {
                        creatures.get(i).EvaluateApproachVel(target.getWorldTranslation());
                    }
                    
                    playcreature.EvaluateApproachVel(target.getWorldTranslation());
                }
                break;
            }
        }     
        
        /*
        if (!logdata && setuptime > 50.0f)
        {
            System.out.println("Log data...");
            logdata = true;
            
            for (int i = 0 ; i < creatures.size(); i++)
            {
                creatures.get(i).logWorm();
            }
        }
        */
    }
    
    private void Evolve()
    {
        for (int i = 0; i < creatures.size(); i++)
        {
            creatures.get(i).SetFitness(creatures.get(i).GetAverageVel());
            //System.out.println("creature " + i + " fitness: " + creatures.get(i).GetFitness());
        }
        
        playcreature.SetFitness(playcreature.GetAverageVel());
        
        Collections.sort(creatures);
        
        for (int i = 0; i < creatures.size(); i++)
        {
            //reatures.get(i).SetFitness(creatures.get(i).GetAverageVel());
            System.out.println("creature " + i + " fitness: " + creatures.get(i).GetFitness());
        }
        
        System.out.println("play creature fitness: " + playcreature.GetFitness());
        
        MJCreature newcreature01 = new MJCreature(rootNode,redmat,creatures.get(0), false);
        this.initFloor(newcreature01.GetPhysicState());
        MJCreature newcreature02 = new MJCreature(rootNode,greenmat,creatures.get(0), true);
        this.initFloor(newcreature02.GetPhysicState());
        MJCreature newcreature03 = new MJCreature(rootNode,bluemat,creatures.get(0), true);
        this.initFloor(newcreature03.GetPhysicState());
        MJCreature newcreature04 = new MJCreature(rootNode,orangemat,creatures.get(0), true);
        this.initFloor(newcreature04.GetPhysicState());
        MJCreature newcreature05 = new MJCreature(rootNode,yellowmat,creatures.get(0), true);
        this.initFloor(newcreature05.GetPhysicState());
        
        for (int i = 0; i < creatures.size(); i++)
        {
            creatures.get(i).KillAll();
        }
        
        creatures.clear();
        
        creatures.add(newcreature01);
        creatures.add(newcreature02);
        creatures.add(newcreature03);
        creatures.add(newcreature04);
        creatures.add(newcreature05);
        
        generation += 1;
        System.out.println("Generation: " + generation);
        SaveState();
        
        playcreature.KillAll();
        
        playcreature = new MJCreature(rootNode, brownmat);
        this.initFloor(newcreature05.GetPhysicState());
    }
    
    private void RestartSimulation()
    {
        time = 0.0f;
        checkfitnesstime = 0;
        state = 0;
        indexrandpositions = 0;
        target.setLocalTranslation(randpositions.get(indexrandpositions));
    }
    
    private void LogAll()
    {
        creatures.get(0).logWorm();
    }

    @Override
    public void simpleRender(RenderManager rm) {
    }
    
    private void SaveState()
    {
        String userHome = System.getProperty("user.home");
        BinaryExporter exporter = BinaryExporter.getInstance();
        File file = new File(userHome + "/Env/"+"State.j3o");
        try {
            
            if (file.exists())
            {
                String timestamp =  new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new java.util.Date());
                File dest = new File( userHome + "/Env/"+"State-" + timestamp + ".j3o");
                Files.copy(file.toPath(),dest.toPath());
            }
               
            if (creatures.size() > 0){
                MJSavableState savestate = new MJSavableState();
                savestate.setCreatures(creatures);
                savestate.setGeneration(generation);
                savestate.setRandpositions(randpositions);
                exporter.save(savestate, file);
            }
            
        } catch (Exception ex) {
          //Logger.getLogger(Main.class.getName()).log(Level.SEVERE, "Error: Failed to save game!", ex);
            System.out.println("Error while saving: " + ex.getMessage());
        }
    }
    
    private void LoadState()
    {
        String userHome = System.getProperty("user.home");
        assetManager.registerLocator(userHome, FileLocator.class);
        MJSavableState loadedstate = (MJSavableState)assetManager.loadAsset( "/Env/"+"State.j3o");
        
        for (int i = 0; i < creatures.size(); i++)
        {
            creatures.get(i).KillAll();
        }
        
        creatures.clear();
        
        creatures = loadedstate.getCreatures();
        
        randpositions.clear();
        
        randpositions = loadedstate.getRandpositions();
        
        generation = loadedstate.getGeneration();
        
        for (int i = 0; i < creatures.size(); i++)
        {
            creatures.get(i).InitExternalCreature(rootNode, allmaterials.get(i));
            this.initFloor(creatures.get(i).GetPhysicState());
        }
        
        RestartSimulation();
    }
    
    

    @Override
    public void onAction(String string, boolean isPressed, float tpf) {
        if (string.equals("load creature") && isPressed)
        {
            LoadState();
        }
        
        if (string.equals("save creature") && isPressed)
        {
            SaveState();
        }
        
        if ("up".equals(string)) {
            if (isPressed) {
                applyForceUp = true;
            } else {
                applyForceUp = false;
            }
        }
        
        if ("down".equals(string)) {
            if (isPressed) {
                applyForceDown = true;
            } else {
                applyForceDown = false;
            }
        }
        
        if ("left".equals(string)) {
            if (isPressed) {
                applyForceLeft = true;
            } else {
                applyForceLeft = false;
            }
        }
        
        if ("right".equals(string)) {
            if (isPressed) {
                applyForceRight = true;
            } else {
                applyForceRight = false;
            }
        }
    }
}