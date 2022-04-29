/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package creaturerevisited;

import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Vector3f;
import java.io.IOException;
import java.util.ArrayList;

/**
 *
 * @author Cristian.Villalba
 */
public class MJSavableState implements Savable{
    private ArrayList<MJCreature> creatures;
    private int generation;
    private ArrayList<Vector3f> randpositions = new ArrayList<Vector3f>();

    
    public ArrayList<MJCreature> getCreatures() {
        return creatures;
    }

    public void setCreatures(ArrayList<MJCreature> creat) {
        this.creatures = creat;
    }

    public int getGeneration() {
        return generation;
    }

    public void setGeneration(int generation) {
        this.generation = generation;
    }
    
    public ArrayList<Vector3f> getRandpositions() {
        return randpositions;
    }

    public void setRandpositions(ArrayList<Vector3f> randpositions) {
        this.randpositions = randpositions;
    }
    
    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.writeSavableArrayList(creatures, "creatures", null);
        capsule.write(generation, "generation", 0);
        capsule.writeSavableArrayList(randpositions, "randpositions", null);
    }

    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule capsule = im.getCapsule(this);
        
        creatures = capsule.readSavableArrayList("creatures", null);
        generation = capsule.readInt("generation", 0);
        randpositions = capsule.readSavableArrayList("randpositions", null);
    }
}
