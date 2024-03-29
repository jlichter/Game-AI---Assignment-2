﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// MapStateManager is the place to keep a succession of events or "states" when building 
/// a multi-step AI demo. Note that this is a way to manage 
/// 
/// State changes could happen for one of two reasons:
///     when the user has pressed a number key 0..9, desiring a new phase
///     when something happening in the game forces a transition to the next phase
/// 
/// One use will be for AI demos that are switched up based on keyboard input. For that, 
/// the number keys 0..9 will be used to dial in whichever phase the user wants to see.
/// </summary>

public class MapStateManager : MonoBehaviour {
    // Set prefabs
    public GameObject PlayerPrefab;     // You, the player
    public GameObject HunterPrefab;     // Agent doing chasing
    public GameObject WolfPrefab;       // Agent getting chased
    public GameObject RedPrefab;        // Red Riding Hood, or just "team red"
    public GameObject BluePrefab;       // "team blue"
    public GameObject TreePrefab;       // New for Assignment #2

    public NPCController house;         // for future use

    // Set up to use spawn points. Can add more here, and also add them to the 
    // Unity project. This won't be a good idea later on when you want to spawn
    // a lot of agents dynamically, as with Flocking and Formation movement.

    public GameObject spawner1;
    public Text SpawnText1;
    public GameObject spawner2;
    public Text SpawnText2;
    public GameObject spawner3;
    public Text SpawnText3;

    public int TreeCount;
 
    private List<GameObject> spawnedNPCs;   // When you need to iterate over a number of agents.
    private List<GameObject> trees;

    private int currentPhase = 0;           // This stores where in the "phases" the game is.
    private int previousPhase = 0;          // The "phases" we were just in

    //public int Phase => currentPhase;

    LineRenderer line;                 
    public GameObject[] Path;
    public Text narrator;

    [Header("our variables for MapStateManager")]
    GameObject wolfNPC;
    public PlayerController thePlayer;
    public Light dirLight;
    float duration = 1.0f;
    public bool theEnd = false;
    Color color0 = Color.red;
    Color color1 = Color.blue;
    //GameObject

    // Use this for initialization. Create any initial NPCs here and store them in the 
    // spawnedNPCs list. You can always add/remove NPCs later on.

    void Start() {
        
        /*
        narrator.text = "Welcome to our demo of ai movement and collision avoidance. The following number" +
            "keys exhibit these algorithms: \n" +
            "(1) Pursue and Evade with Wall Avoidance\n" +
            "(2) More intelligent Wander\n"+
            "(3) Chasing player with Collision Prediction\n"+
            "(4) "; //todo edit
        */
        narrator.text = "Welcome to the tale of Little Red Riding Hood, \n" +
            "the Hunter, and the Wolf";
        //narrator.resizeTextForBestFit = true;
        narrator.alignment = TextAnchor.UpperLeft;

        TreeCount = 100;    // TreeCount isn't showing up in Inspector
        house.GetComponent<NPCController>().isHouse = true;
        //trees = new List<GameObject>();
        // SpawnTrees(TreeCount);

        spawnedNPCs = new List<GameObject>();
        EnterMapStateOne();
        //StartCoroutine(playStory());
        /*
        GameObject temp_hunter = SpawnItem(spawner2, HunterPrefab, null, SpawnText1, 1);
        GameObject wolf_evade = SpawnItem(spawner1, WolfPrefab, temp_hunter.GetComponent<NPCController>(), SpawnText1, 2);
        temp_hunter.GetComponent<SteeringBehavior>().target = wolf_evade.GetComponent<NPCController>();
        spawnedNPCs.Add(temp_hunter);
        spawnedNPCs.Add(wolf_evade);
        */
        // GameObject temp_hunter = SpawnItem(spawner2, HunterPrefab, null, SpawnText1, 4);
        // GameObject temp_hunter = SpawnItem(spawner2, HunterPrefab, null, SpawnText1, 4);
        //   spawnedNPCs.Add(temp_hunter);
        //  StartCoroutine(playStory());
        //   EnterMapStateOne();
        //  PlayerPrefab = GameObject.FindGameObjectWithTag("Player");
        // thePlayer = PlayerPrefab.GetComponent<PlayerController>();
        // add to list, with call to SpawnItem (returns game object) --> args ( game object location, the prefab, the target, the text, 


        //Invoke("SpawnWolf", 12);
        //Invoke("Meeting1", 30);
    }

    /// <summary>
    /// This is where you put the code that places the level in a particular phase.
    /// Unhide or spawn NPCs (agents) as needed, and give them things (like movements)
    /// to do. For each case you may well have more than one thing to do.
    /// </summary>
    private void Update()
    {
        if (theEnd) {
             float t = Mathf.PingPong(Time.time, duration) / duration;
             dirLight.color = Color.Lerp(color0, color1, t);
        }

    }

    IEnumerator playStory() {
        print(Time.time);
        
        narrator.text = "The Hunter appears...";
        GameObject hunter = SpawnItem(spawner1, HunterPrefab, null, SpawnText1, 0);
        spawnedNPCs.Add(hunter);
        // waits for 2 seconds, then begins to wander 
        yield return new WaitForSeconds(2);
        narrator.text = "...and begins to wander";
        SpawnText1.text = "";
        //spawnedNPCs.Add(SpawnItem(spawner1, HunterPrefab, null, SpawnText1, 0));
        spawnedNPCs[0].GetComponent<NPCController>().phase = 4;
        print(Time.time);
        yield return new WaitForSeconds(5);        
        // no 2, pat's code
        narrator.text = "Ah! A wolf, seemingly docile...";
        // spawn wolf wandering
        GameObject wolf = SpawnItem(spawner1, WolfPrefab, null, SpawnText2, 0);
        spawnedNPCs.Add(wolf);
        // wait 5 seconds, then begin to wander 
        yield return new WaitForSeconds(3);
        SpawnText2.text = "";
        spawnedNPCs[1].GetComponent<NPCController>().phase = 4;
        yield return new WaitForSeconds(5);
        StartCoroutine("checkDistance");
      
        narrator.text = "The hunter spots the wolf, and begins his pursuit...";
        SpawnText1.text = "";
        SpawnText2.text = "";
        spawnedNPCs[0].GetComponent<NPCController>().phase = 0;
        spawnedNPCs[1].GetComponent<NPCController>().phase = 0;
        spawnedNPCs[0].GetComponent<SteeringBehavior>().target = spawnedNPCs[1].GetComponent<NPCController>();
        spawnedNPCs[0].GetComponent<NPCController>().phase = 1;
        spawnedNPCs[1].GetComponent<SteeringBehavior>().target = spawnedNPCs[0].GetComponent<NPCController>();
        spawnedNPCs[0].GetComponent<NPCController>().phase = 2;
        
            
        // make the wolf the hunter's target and change hunter's phase to pursue state
        
      //  spawnedNPCs[0].GetComponent<NPCController>().phase = 1;
        yield return new WaitForSeconds(5);
     
    }
        

    private void EnterMapStateOne() { // note => ** PURSUE AND EVADE, WITH WALL AVOIDANCE **

        narrator.text = "First, the hunter appears, and then begins to wander around";
        GameObject hunter = SpawnItem(spawner1, HunterPrefab, null, SpawnText1, 0);
        spawnedNPCs.Add(hunter);
        Invoke("startHunterWander", 2f);
        Invoke("EnterMapStateTwo", 5f);
    }

    private void startHunterWander() {
        spawnedNPCs[0].GetComponent<NPCController>().phase = 4;

    }

    private void EnterMapStateTwo()
    {
        narrator.text = "Soon after, the wolf appears, and begins to wander";
        GameObject wolf = SpawnItem(spawner1, WolfPrefab, null, SpawnText2, 0);
        spawnedNPCs.Add(wolf);
        Invoke("startWolfWander", 3f);
        StartCoroutine("checkDistance");
    }

    private void startWolfWander() {
        spawnedNPCs[1].GetComponent<NPCController>().phase = 4;
    }

    private IEnumerator checkDistance() {
        while (true) {
            if(Vector3.Distance(spawnedNPCs[0].transform.position, spawnedNPCs[1].transform.position) < 10f) {
                break;
            }
            yield return null;
        }
        EnterMapStateThree();
    }

    private void EnterMapStateThree()
    {
        narrator.text = "The hunter spots the wolf! He begins to chase the fleeing wolf!";
        
        spawnedNPCs[1].GetComponent<SteeringBehavior>().target = spawnedNPCs[0].GetComponent<NPCController>();
        spawnedNPCs[1].GetComponent<NPCController>().phase = 2;
        spawnedNPCs[0].GetComponent<SteeringBehavior>().target = spawnedNPCs[1].GetComponent<NPCController>();
        spawnedNPCs[0].GetComponent<NPCController>().phase = 1;
        StartCoroutine("checkCollision");
        //spawnedNPCs.Add(SpawnItem(spawner2, WolfPrefab, null, SpawnText2, 4));
    }

    private IEnumerator checkCollision() {
        while (true) {
            if (Vector3.Distance(spawnedNPCs[0].transform.position, spawnedNPCs[1].transform.position) < 3f) {
                break;
            }
            yield return null;
        }
        foreach (GameObject go in spawnedNPCs) {
            Destroy(go);
        }
        spawnedNPCs.Clear();
        //Destroy(spawnedNPCs[0]);
        //Destroy(spawnedNPCs[1]);
        EnterMapStateFour();
    }

    private void EnterMapStateFour() {
        narrator.text = "The hunter catches the wolf! Meanwhile, Little Red appears, heading to grandma's house";
        SpawnText1.text = "";
        SpawnText2.text = "";
        GameObject RedRiding = SpawnItem(spawner1, RedPrefab, null, SpawnText3, 6);
        spawnedNPCs.Add(RedRiding);
        CreatePath(spawnedNPCs[0]);
        Invoke("EnterMapStateFive", 5f);
    }

    private void EnterMapStateFive() {
        narrator.text = "But wait! The wolf shows up again, and makes his way to Red";
        GameObject wolf = SpawnItem(spawner3, WolfPrefab, spawnedNPCs[0].GetComponent<NPCController>(), SpawnText2, 1);
        spawnedNPCs.Add(wolf);
        //spawnedNPCs[1] = SpawnItem(spawner3, WolfPrefab, spawnedNPCs[2].GetComponent<NPCController>(), SpawnText2, 1);
        StartCoroutine("redWolfMeeting");
    }

    private IEnumerator redWolfMeeting() {
        while (true) {
            if (Vector3.Distance(spawnedNPCs[1].transform.position, spawnedNPCs[0].transform.position) < 3f) {
                break;
            }
            yield return null;
        }
        spawnedNPCs[1].GetComponent<NPCController>().phase = 7;
        spawnedNPCs[0].GetComponent<NPCController>().phase = 7;
        Invoke("EnterMapStateSix", 4);
    }

    private void EnterMapStateSix() {
        narrator.text = "Red continues on her way while the wolf rushes to the house...";
        spawnedNPCs[0].GetComponent<NPCController>().phase = 6;
        spawnedNPCs[1].GetComponent<NPCController>().GetComponent<SteeringBehavior>().target = house;
        spawnedNPCs[1].GetComponent<NPCController>().phase = 3;
        SetArrive(spawnedNPCs[1]);
        Invoke("EnterMapStateSeven", 2);
    }

    private void EnterMapStateSeven() {
        narrator.text = "But the hunter shows up again, and sees the wolf rushing to the home, he must stop it!";
        GameObject hunter = SpawnItem(spawner3, HunterPrefab, house, SpawnText1, 3);
        spawnedNPCs.Add(hunter);
        spawnedNPCs[2].GetComponent<NPCController>().isHunter = true;
        spawnedNPCs[2].GetComponent<NPCController>().phase = 3;
        //SetArrive(spawnedNPCs[2]);
        StartCoroutine("gotToHouse");
        //spawnedNPCs[0] = SpawnItem(spawner3, HunterPrefab, house, SpawnText1, 3);
    }
    private IEnumerator gotToHouse() {
        while (true) {
            if (Vector3.Distance(spawnedNPCs[2].transform.position, spawnedNPCs[2].GetComponent<SteeringBehavior>().target.position) < 10f) {
                break;
            }
            yield return null;
        }
        Invoke("EnterEnd", 2);
    }
    private void EnterEnd() {
        narrator.text = "suddenly, a darkness overcame the woods, and everyone decided to get boogie with it!";
        SpawnText1.text = "";
        SpawnText2.text = "";
        SpawnText3.text = "";
        spawnedNPCs[0].GetComponent<NPCController>().phase = 7;
        spawnedNPCs[1].GetComponent<NPCController>().phase = 7;
        spawnedNPCs[2].GetComponent<NPCController>().phase = 7;
        theEnd = true;
        

       
    }
        // ... Etc. Etc.

        /// <summary>
        /// SpawnItem placess an NPC of the desired type into the game and sets up the neighboring 
        /// floating text items nearby (diegetic UI elements), which will follow the movement of the NPC.
        /// </summary>
        /// <param name="spawner"></param>
        /// <param name="spawnPrefab"></param>
        /// <param name="target"></param>
        /// <param name="spawnText"></param>
        /// <param name="phase"></param>
        /// <returns></returns>
        private GameObject SpawnItem(GameObject spawner, GameObject spawnPrefab, NPCController target, Text spawnText, int phase)
    {
        Vector3 size = spawner.transform.localScale;
        Vector3 position = spawner.transform.position + new Vector3(UnityEngine.Random.Range(-size.x / 2, size.x / 2), 0, UnityEngine.Random.Range(-size.z / 2, size.z / 2));
        GameObject temp = Instantiate(spawnPrefab, position, Quaternion.identity);
        if (target)
        {
            temp.GetComponent<SteeringBehavior>().target = target;
        }
        temp.GetComponent<NPCController>().label = spawnText;
        temp.GetComponent<NPCController>().phase = phase;
        Camera.main.GetComponent<CameraController>().player = temp;
        return temp;
    }

    /// <summary>
    /// SpawnTrees will randomly place tree prefabs all over the map. The diameters
    /// of the trees are also varied randomly.
    /// 
    /// Note that it isn't particularly smart about this (yet): notably, it doesn't
    /// check first to see if there is something already there. This should get fixed.
    /// </summary>
    /// <param name="numTrees">desired number of trees</param>
    private void SpawnTrees(int numTrees)
    {
        float MAX_X = 20;  // Size of the map; ideally, these shouldn't be hard coded
        float MAX_Z = 25;
        float less_X = MAX_X - 1;
        float less_Z = MAX_Z - 1;

        float diameter;

        for (int i = 0; i < numTrees; i++)
        {
            //Vector3 size = spawner.transform.localScale;
            Vector3 position = new Vector3(UnityEngine.Random.Range(-less_X, less_X), 0, UnityEngine.Random.Range(-less_Z, less_Z));
            GameObject temp = Instantiate(TreePrefab, position, Quaternion.identity);

            // diameter will be somewhere between .2 and .7 for both X and Z:
            diameter = UnityEngine.Random.Range(0.2F, 0.7F);
            temp.transform.localScale = new Vector3(diameter, 1.0F, diameter);

            trees.Add(temp);
          
        }
    }

    private void DestroyTrees()
    {
        GameObject temp;
        for (int i = 0; i < trees.Count; i++)
        {
            temp = trees[i];
            Destroy(temp);
        }
        // Following this, write whatever methods you need that you can bolt together to 
        // create more complex movement behaviors.
    }
    private void SpawnWolf()
    {
        narrator.text = "The Wolf appears. Most wolves are ferocious, but this one is docile.";
        spawnedNPCs.Add(SpawnItem(spawner2, WolfPrefab, null, SpawnText2, 4));
    }

    private void Meeting1() {
        if (currentPhase == 0) {
            spawnedNPCs[0].GetComponent<SteeringBehavior>().target = spawnedNPCs[1].GetComponent<NPCController>();
            spawnedNPCs[1].GetComponent<SteeringBehavior>().target = spawnedNPCs[0].GetComponent<NPCController>();
            SetArrive(spawnedNPCs[0]);
            SetArrive(spawnedNPCs[1]);
        }
    }

    private void SpawnWolf2() {
        narrator.text = "The Wolf looks for shelter, and spots little Red.";
        spawnedNPCs.Add(SpawnItem(spawner3, WolfPrefab, spawnedNPCs[2].GetComponent<NPCController>(), SpawnText1, 1));
        spawnedNPCs[3].GetComponent<NPCController>().label.enabled = true;
    }

    private void Meeting2() {
        narrator.text = "The two converse, and little Red directs the Wolf to her house.";
        spawnedNPCs[2].GetComponent<NPCController>().DestroyPoints();
        spawnedNPCs[2].GetComponent<NPCController>().phase = 5;
        spawnedNPCs[3].GetComponent<SteeringBehavior>().target = house;
        spawnedNPCs[3].GetComponent<NPCController>().phase = 1; ;
        Invoke("SpawnHunter", 10);
    }

    private void SpawnHunter() {
        narrator.text = "The Hunter arrives, determined to catch the killer. He spots a house and moves accordingly.";
        spawnedNPCs.Add(SpawnItem(spawner3, HunterPrefab, house, SpawnText2, 1));
        spawnedNPCs[4].GetComponent<NPCController>().label.enabled = true;
    }

    private void End() {
        narrator.text = "Days later, reports come in. The killer is still at large, but police have found one clue on its identity. "
            +"A little red hood. END";
        currentPhase++;
    }

    private void SetArrive(GameObject character) {

        //character.GetComponent<NPCController>().phase = 3;
        character.GetComponent<NPCController>().DrawConcentricCircle(character.GetComponent<SteeringBehavior>().slowRadiusL);
    }

    /*
    private void CreatePath()
    {
        line = GetComponent<LineRenderer>();
        line.positionCount = Path.Length;
        for (int i = 0; i < Path.Length; i++)
        {
            line.SetPosition(i, Path[i].transform.position);
        }
    }
    */
    private void CreatePath(GameObject character) {
        line = GetComponent<LineRenderer>();
        line.positionCount = character.GetComponent<SteeringBehavior>().Path.Length;
        for (int i = 0; i < character.GetComponent<SteeringBehavior>().Path.Length; i++) {
            line.SetPosition(i, character.GetComponent<SteeringBehavior>().Path[i].transform.position);
        }
    }

    void OnDrawGizmosSelected() {
        Gizmos.color = new Color(1, 0, 0, 0.5f);
        Gizmos.DrawCube(spawner1.transform.position, spawner1.transform.localScale);
        Gizmos.DrawCube(spawner2.transform.position, spawner2.transform.localScale);
        Gizmos.DrawCube(spawner3.transform.position, spawner3.transform.localScale);
    }
}
