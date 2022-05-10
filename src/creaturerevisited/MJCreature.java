/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package creaturerevisited;

import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.joints.motors.MotorParam;
import com.jme3.bullet.joints.motors.RotationMotor;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.material.Material;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import static creaturerevisited.Main.log;
import java.io.IOException;
import java.util.ArrayList;

/**
 *
 * @author Cristian.Villalba
 */
public class MJCreature implements Comparable, Savable{
    private ArrayList<MJNeuron[]> neuronmain;

    private ArrayList<Float> averagedistances = new ArrayList<Float>();
    private ArrayList<MJNode> worm;
    private PhysicsSpace bulletstate;
    private float fitness;
    private int wormsize = -1;
    
    
    public MJCreature()
    {
        
    }
    
    public MJCreature( Node rootNode, Material creatmat)
    {
        worm = new ArrayList<MJNode>();
        //wormbody = new ArrayList<RigidBodyControl>();
        
        bulletstate = LoadPhysicsBasic();
        
        loadCreature(rootNode, creatmat, -1);
        
        GenerateBrain();
    }
    
    public MJCreature( Node rootNode, Material creatmat, MJCreature tocopy, boolean mutate)
    {
        worm = new ArrayList<MJNode>();
        //wormbody = new ArrayList<RigidBodyControl>();
        
        bulletstate = LoadPhysicsBasic();
        
        loadCreature(rootNode, creatmat, tocopy.GetWormLength());
        
        CloneBrain(tocopy.GetBrain(), mutate);
    }
    
    public PhysicsSpace GetPhysicState()
    {
        return bulletstate;
    }
    
    private PhysicsSpace LoadPhysicsBasic()
    {
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.SIMPLE);        
        //stateManager.attach(space);
        //bulletAppState.getPhysicsSpace().setAccuracy(1/120f);
         
        return space;
    }
    
    public void KillAll()
    {
        for (int i = 0; i < worm.size(); i++){
            bulletstate.getPhysicsSpace().removeAll(worm.get(i));
            worm.get(i).removeFromParent();
        }
    }
    
    
    private void loadCreature(Node rootNode, Material redmat, int size)
    {
        float ypos = 25;  
        int finalsize = 0;
        ArrayList<New6Dof> wormjoint = new ArrayList<New6Dof>();
        
        if (size ==  -1)
        {
            finalsize = MJFastMath.nextRandomInt(4, 7);
        }
        else
        {
            finalsize = size;
        }
        
        wormsize = finalsize;
        
        for (int i = 0 ; i < finalsize ; i++, ypos -= 5.0f)
        {
            Box dimensionbox = new Box(1.0f, 1.0f, 1.0f);
            Geometry body = new Geometry("part", dimensionbox);
            body.setMaterial(redmat);

            RigidBodyControl body_phy = new RigidBodyControl(0.5f);

            GhostControl ghost_control = new GhostControl(new BoxCollisionShape(new Vector3f(0.5f,0.5f,0.5f)));

            MJNode childnode = new MJNode();
            childnode.setLocalTranslation(0, ypos, 0);
            childnode.attachChild(body);
            body.addControl(body_phy);
            body.addControl(ghost_control);

            rootNode.attachChild(childnode);
            
                                  
            worm.add(childnode);
            childnode.setBody(body_phy);
            childnode.setGhost(ghost_control);
            //wormbody.add(body_phy);
            //wormghost.add(ghost_control);
        }
        
        for (int i = 0 ; i < (finalsize - 1); i++)
        {
            Vector3f jointa = new Vector3f();
            Vector3f jointb = new Vector3f();

            jointa.set(worm.get(i).getWorldTranslation());
            jointb.set(worm.get(i+1).getWorldTranslation().subtract(worm.get(i).getWorldTranslation()).mult(0.5f));
            jointb.addLocal(jointa);
            
            Vector3f pivotA = worm.get(i).worldToLocal(jointb, new Vector3f());
            Vector3f pivotB = worm.get(i+1).worldToLocal(jointb, new Vector3f());
            
            //New6Dof newcone = new New6Dof(wormbody.get(i), wormbody.get(i+1), pivotA, pivotB, new Matrix3f(), new Matrix3f(), RotationOrder.XYZ);
            New6Dof newcone = new New6Dof(worm.get(i).GetBodyControl(), worm.get(i+1).GetBodyControl(), pivotA, pivotB, new Matrix3f(), new Matrix3f(), RotationOrder.XYZ);
            RotationMotor xMotor = newcone.getRotationMotor(PhysicsSpace.AXIS_X);

            xMotor.set(MotorParam.MaxMotorForce, 40000f);
            xMotor.setMotorEnabled(true);
            RotationMotor yMotor = newcone.getRotationMotor(PhysicsSpace.AXIS_Y);

            yMotor.set(MotorParam.MaxMotorForce, 40000f);
            yMotor.setMotorEnabled(true);

            //RotationMotor motorz = newcone.getRotationMotor(PhysicsSpace.AXIS_Z);
            //motorz.set(MotorParam.MaxMotorForce, 40000f);
            //motorz.setMotorEnabled(true);
            
            wormjoint.add(newcone);
        }
        
        for (int i = 0; i < finalsize ; i++)
        {
            //bulletstate.getPhysicsSpace().add(wormbody.get(i));
            //bulletstate.getPhysicsSpace().add(wormghost.get(i));
            bulletstate.getPhysicsSpace().add(worm.get(i).GetBodyControl());
            bulletstate.getPhysicsSpace().add(worm.get(i).GetGhostControl());
        }
        
        for (int i = 0; i < (finalsize -1) ; i++)
        {
            bulletstate.getPhysicsSpace().add(wormjoint.get(i));
        }
    }
    
    public void tick(float step, boolean issimulating)
    {
        bulletstate.update(step); //fixed time
        
        if (issimulating)
        {
            this.tickBrain(step);
        }
    }
    
    public void tickBrain(float step)
    {
        int maxtick = 5;
        
        if (neuronmain != null && neuronmain.size() > 0){
            //for (int i =0 ; i < neuronmain.get(3).length - 1; i++)
            for (int i =0 ; i < neuronmain.get(2).length - 1; i++)
            {
                //neuronmain.get(3)[i].fire(tpf, maxtick);
                neuronmain.get(2)[i].fire(step, maxtick);
            }
        }
        
    }
    
    private void GenerateBrain()
    {
        MJNeuron[] sensors = null;
        
        neuronmain = new ArrayList<MJNeuron[]>();
     
        int randsensor = 6; // 2 eyes and more other type sensors;
        int randnoteyessensor = MJFastMath.nextRandomInt(2, 7);
        sensors = new MJNeuron[randsensor + randnoteyessensor];
        
        int randneuronsizev0 = MJFastMath.nextRandomInt(6, 14);
        //int randneuronsizev0 = 10;
        MJNeuron[] neuronsv0 = new MJNeuron[randneuronsizev0];
        
        
        int randeffector = MJFastMath.nextRandomInt(6, 12);
        MJNeuron[] effectors = new MJNeuron[randeffector];
        
        neuronmain.add(sensors);
        neuronmain.add(neuronsv0);
        //neuronmain.add(neurons);
        neuronmain.add(effectors);
               
        MJFastMath.nextRandomInt(0, worm.size() - 1);
        
        int randpartindex = MJFastMath.nextRandomInt(0, worm.size() - 1);
        MJNode randpart = worm.get(randpartindex);
        for(int i = 0; i <  3; i++)//first eye
        {
            MJNeuron sens2;
            
            sens2 = new MJNeuron( 0, i);       
            sens2.SetPartAndIndex(randpart, randpartindex);
            sensors[i] = sens2;
        }
        
        randpartindex = MJFastMath.nextRandomInt(0, worm.size() - 1);
        randpart = worm.get(randpartindex);
        for(int i = 0; i <  3; i++)//second eye
        { 
            MJNeuron sens2;
            
            sens2 = new MJNeuron( 0, i);       
            sens2.SetPartAndIndex(randpart, randpartindex);
            sensors[i + 3] = sens2;
        }
        
        for(int i = 0; i <  randnoteyessensor; i++)//other type of sensors
        {
            MJNeuron sens2;
            int subtype = MJFastMath.nextRandomInt(-2, -1);
            
            sens2 = new MJNeuron( 0, subtype);   
            int rpartindex = MJFastMath.nextRandomInt(0, worm.size() - 1);
            sens2.SetPartAndIndex(worm.get(rpartindex), rpartindex);
            sensors[i + 6] = sens2;
        }
        
        for(int i = 0; i < neuronsv0.length; i++)
        {
            MJNeuron neur = new MJNeuron( 1, 0);
            neuronsv0[i] = neur;
            
            for (int k = 0 ; k < sensors.length; k++)
            {
               neur.connect(sensors[k]);
            }
        }
        
        
        for(int i = 0; i < effectors.length; i++)
        {
            randpartindex = MJFastMath.nextRandomInt(0, worm.size() - 1);
            MJNode part =  worm.get(randpartindex);
            int randtorqueaxis = MJFastMath.nextRandomInt(0,5);
            
            MJNeuron eff = new MJNeuron(2, randtorqueaxis);
            eff.SetPartAndIndex(part, randpartindex);
            effectors[i] = eff;
            
            
            for (int k = 0 ; k < neuronsv0.length; k++)
            {
                eff.connect(neuronsv0[k]);
            }
        }
        
        for (int i = 0; i < neuronsv0.length; i++)
        {
            neuronsv0[i].GenerateWeights();
        }
        
        for (int i = 0; i < effectors.length; i++)
        {
            effectors[i].GenerateWeights();
        }
    }
    
    public void PrintBrain(ArrayList<MJNeuron[]> b)
    {
        System.out.println("Sens:");
        for(int i = 0; i < b.get(0).length; i++)
        {
            System.out.println(b.get(0)[i].GetInfo());
        }
        
        System.out.println("Neur2:");
        for(int i = 0; i < b.get(1).length; i++)
        {
            System.out.println(b.get(1)[i].GetInfo());
        }
        System.out.println("Eff:");
        for(int i = 0; i < b.get(2).length; i++)
        {
            System.out.println(b.get(2)[i].GetInfo());
        }
    }
    
    private void CloneBrain(ArrayList<MJNeuron[]> brain, boolean mutate)
    {        
        neuronmain = new ArrayList<MJNeuron[]>();
     
        MJNeuron[] sensors = new MJNeuron[brain.get(0).length];
        MJNeuron[] neuronsv0 = new MJNeuron[brain.get(1).length];
        MJNeuron[] effectors = new MJNeuron[brain.get(2).length];
        
        neuronmain.add(sensors);
        neuronmain.add(neuronsv0);
        neuronmain.add(effectors);
        
        //clone neurons
        for (int i = 0; i < brain.get(0).length; i++)
        {
            MJNeuron newsensor = brain.get(0)[i].Mirror(mutate);
            
            int partindex = brain.get(0)[i].GetPartIndex();
            newsensor.SetPartAndIndex(worm.get(partindex), partindex);
            sensors[i] = newsensor;
        }
        
        for (int i = 0; i < brain.get(1).length; i++)
        {
            MJNeuron newneuron = brain.get(1)[i].Mirror(mutate);
            neuronsv0[i] = newneuron;
        }
        
        for (int i = 0; i < brain.get(2).length; i++)
        {
            MJNeuron neweffector = brain.get(2)[i].Mirror(mutate);
            
            int partindex = brain.get(2)[i].GetPartIndex();
            neweffector.SetPartAndIndex(worm.get(partindex), partindex);
            effectors[i] = neweffector;
        }
        
        
        //clone connections
        for(int i = 0; i < neuronsv0.length; i++)
        {
            for (int k = 0 ; k < sensors.length; k++)
            {
               neuronsv0[i].connect(sensors[k]);
            }
        }
        
        for(int i = 0; i < effectors.length; i++)
        {
            for (int k = 0 ; k < neuronsv0.length; k++)
            {
                effectors[i].connect(neuronsv0[k]);
            }
        }
    }
    
    private void ConnectBrain()
    {
        for (int i = 0; i < neuronmain.get(0).length; i++)
        {
            MJNeuron sensor = neuronmain.get(0)[i];
            
            int partindex = neuronmain.get(0)[i].GetPartIndex();
            sensor.SetPartAndIndex(worm.get(partindex), partindex);
        }
        
        for (int i = 0; i < neuronmain.get(2).length; i++)
        {
            MJNeuron effector = neuronmain.get(2)[i];
            
            int partindex = neuronmain.get(2)[i].GetPartIndex();
            effector.SetPartAndIndex(worm.get(partindex), partindex);
        }
    }
    
    public ArrayList<MJNeuron[]> GetBrain()
    {
        return neuronmain;
    }
    
    public void logWorm()
    {
        for (int i = 0 ; i < worm.size(); i++)
        {
            log.info("body n" + i + " : " + worm.get(i).getChild(0).getWorldTranslation());
        }
    }
    
    public void SetFitness(float g)
    {
        fitness = g;
    }
    
    
    public float GetFitness()
    {
        return fitness;
    }
    
    public float GetAverageVel()
    {
        float totals = 0;
        
        //System.out.println("distances: " + averagedistances);
        
        for (int i = 0 ; i < averagedistances.size(); i++)
        {
            totals += averagedistances.get(i);
        }
        
        //System.out.println("total: " + (totals / ((float)averagedistances.size())));
        //return (totals / ((float)averagedistances.size()));
        return totals;
    }
    
    private void SetDistance(float v)
    {
        averagedistances.add(v);
    }
    
    public MJNode GetRootWorm()
    {
        return worm.get(0);
    }
    
    public int GetWormLength(){
        
        if (worm == null){
            return wormsize;
        }
        else{
            return worm.size();
        }
    }
    
    public void EvaluateApproachVel(Vector3f targetpos)
    {
        //System.out.println("targetpos: " + targetpos + "\n");
        //Spatial nearest = rootnode.GetNearest(targetpos, null);
        Spatial nearest = worm.get(0).getChild(0);
        Vector3f velocity = worm.get(0).GetAverageVel();
        //System.out.println(velocity);
        Vector3f totargetactual = targetpos.subtract(nearest.getWorldTranslation());
        //System.out.println("totarget: " + totargetactual + "\n");
        float rate = 0;
     
         
        if (totargetactual.length() <= 3.0f)
        {
           this.SetDistance(0);
        }
        else
        {

            Vector3f speedtowards = velocity.project(totargetactual);
            //rate = 1/speedtowards.length() * totargetactual.length(); //distance + speed
            
            rate = totargetactual.length();//only distance
            //System.out.println("rate:" + rate + "\n");
            this.SetDistance(rate);
        }
        
        //originalpos.set(nearest.getWorldTranslation()); //guarda la nueva posicion inicial
        this.SavePreviousPos();
    }
    
    public void SavePreviousPos()
    {
        for (int i = 0 ; i< worm.size(); i++)
        {
            worm.get(i).SavePreviousPos();
        }
        
    }

    @Override
    public int compareTo(Object o) {
        float outfitness = ((MJCreature)o).GetFitness();
        
        if ( new Float(outfitness).isInfinite() &&  new Float(this.fitness).isInfinite()){
            return 0;
        }
        
        if ( new Float(this.fitness).isInfinite()){
            return 1;
        }
        
        if ( new Float(outfitness).isInfinite()){
            return -1;
        }
        
        return      this.fitness > outfitness ? 1
                :   this.fitness < outfitness ? -1
                :0;
    }

    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule capsule = ex.getCapsule(this);
        
        if (neuronmain != null && neuronmain.size() == 3){
            ArrayList<MJNeuron> sensors = new ArrayList<MJNeuron>();
            ArrayList<MJNeuron> neuronsv0 = new ArrayList<MJNeuron>();
            //ArrayList<MJNeuron> neurons = new ArrayList<MJNeuron>();
            ArrayList<MJNeuron> effectors = new ArrayList<MJNeuron>();

            for (int i = 0; i < neuronmain.get(0).length;i++)
            {
                sensors.add(neuronmain.get(0)[i]);
            }
            for (int i = 0; i < neuronmain.get(1).length;i++)
            {
                neuronsv0.add(neuronmain.get(1)[i]);
            }
            //for (int i = 0; i < neuronmain.get(2).length;i++)
            //{
            //    neurons.add(neuronmain.get(2)[i]);
            //}
            for (int i = 0; i < neuronmain.get(2).length;i++)
            {
                effectors.add(neuronmain.get(2)[i]);
            }

            capsule.writeSavableArrayList(sensors, "sensors", null);
            capsule.writeSavableArrayList(neuronsv0, "neuronsv0", null);
            //capsule.writeSavableArrayList(neurons, "neurons", null);
            capsule.writeSavableArrayList(effectors, "effectors", null);
            capsule.write(wormsize, "wormsize", 0);
        }
    }

    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule capsule = im.getCapsule(this);
 
        ArrayList<MJNeuron> sensors = capsule.readSavableArrayList("sensors", null);
        ArrayList<MJNeuron> neuronsv0 = capsule.readSavableArrayList("neuronsv0", null);
        //ArrayList<MJNeuron> neurons = capsule.readSavableArrayList("neurons", null);
        ArrayList<MJNeuron> effectors = capsule.readSavableArrayList("effectors", null);
        wormsize = capsule.readInt("wormsize", 0);
        
        neuronmain = new ArrayList<MJNeuron[]>();
        
        
        if (sensors != null && neuronsv0 != null && effectors != null){
            MJNeuron[] sens = new MJNeuron[sensors.size()];
            MJNeuron[] neuv0 = new MJNeuron[neuronsv0.size()];
            //MJNeuron[] neu = new MJNeuron[neurons.size()];
            MJNeuron[] eff = new MJNeuron[effectors.size()];

            for (int i = 0; i < sensors.size();i++)
            {
                sens[i] = sensors.get(i);
            }
            for (int i = 0; i < neuronsv0.size();i++)
            {
                neuv0[i] = neuronsv0.get(i);
            }
            //for (int i = 0; i < neurons.size();i++)
            //{
            //    neu[i] = neurons.get(i);
            //}
            for (int i = 0; i < effectors.size();i++)
            {
                eff[i] = effectors.get(i);
            }

            neuronmain.add(sens);
            neuronmain.add(neuv0);
            //neuronmain.add(neu);
            neuronmain.add(eff);
        }
    }
}
