/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package creaturerevisited;

import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.control.GhostControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import java.io.IOException;

/**
 *
 * @author Cristian.Villalba
 */
public class MJNode extends Node implements Savable{
    public static Geometry target;
    private RigidBodyControl body_phy;
    private GhostControl ghost_control;
    
    
    public RigidBodyControl GetBodyControl()
    {
        return body_phy;
    }
    
    public GhostControl GetGhostControl()
    {
        return ghost_control;
    }
    
    public void setBody(RigidBodyControl rc)
    {
        body_phy = rc;
    }
    
    public void setGhost(GhostControl gc)
    {
        ghost_control = gc;
    }
    
    public void SavePreviousPos()
    {
        this.setUserData("previouspos", this.getWorldTranslation().clone());
    }
    
    public Vector3f GetAverageVel()
    {
        Vector3f previousvel = (Vector3f)this.getUserData("previouspos");
        
        if (previousvel == null)
        {
            previousvel = new Vector3f();
        }
        
        Vector3f vel = this.getWorldTranslation().subtract(previousvel);
        
        return vel;
    }
    
    public void write(JmeExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);

        capsule.write(body_phy,   "body_phy",   null);
        capsule.write(ghost_control,   "ghost_control",   null);
       
        capsule.write(target,   "target",   null);
      
    }

    public void read(JmeImporter im) throws IOException {
        InputCapsule capsule = im.getCapsule(this);

        
        body_phy = new RigidBodyControl(0.5f);
        ghost_control = new GhostControl(new BoxCollisionShape(new Vector3f(0.5f,0.5f,0.5f)));
        this.addControl(body_phy);
        this.addControl(ghost_control);
        //}
        
        
        
        target = (Geometry)capsule.readSavable("target",   null);
        
        
    }
}
