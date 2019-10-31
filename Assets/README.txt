README
Single Agent Movement 
Patrick Butler, Jessica Lichter
- Story happens using state changes; we wait a bit, wait for event to occur,
and move forward with event.
- a lot of the behavior is finnicky; we tried to correct this by 
multiplying angulars and linears in NPCController to get a better result.
- Arrive(), Wander(), and followPath() have obstacle avoidance built into their functions.
- Displaying their visuals was a bit difficult and finnicky as well,
so only the wolf does.
- functions used in SteeringBehavior were Arrive(), Pursue(), Evade(),
WallAvoidance(), Align(), Face(), Wander(), and followPath()
- NPCController contains two bools: isHouse (so NPCController attached to 
house doesn't get wacky) which is switched to true when house is assigned as
target for hunter and wolf and isHunter (so hunter moves faster when respawned)
which is switched to true when hunter performs the behavior of arrive towards 
the house. 

-- Note: apologies for submitting so late. There was confusion when submitting our --
   remake of assignment 2 and various git errors. We put a lot of time 
   into trying to improve our gameplay and algorithms and again apolgoize for the
   mistaken dates/ issues. 