# RobotGame Bots

A collection of bots I created for [RobotGame.org](robotgame.org) mainly to mess around a bit with Python.

 - basic.py
   - First bot - hangs around the middle and attacks its closest enemy
 - tard.py 
  - Sits there and takes its licks by just guarding. Always helpful to have terrible bots to make sure my good bots can at least beat the monkey
 - pairs.py
   - Adds cooperative path-finding (we won't route bots to the same location and will plan routes that allow them to move in lock-step)
   - Buddies up bots so that can burn down enemies together and win more fights
   - Keeps away from spawn squares (always; ideally we should compute when spawn turns are and use the spaces strategically to lure enemies into danger)
   - Uses a priority list of actions for the pairs: If ungrouped, meet up with a partner => If adjacent to an enemy and out-numbered, flee => If grouped, find nearest enemy, preferring fights where we outnumber the enemy. 
 - gnats.py
   - Builds on pairs.py to add arbitrary group sizes and more intelligent fleeing. Basically it had been working too well and the fleeing bots were hanging on taking up enough space that the pairs of bots couldn't path around effectively, so enemies were taking pot shots when we got too close and slowly degrading the swarm's health. Now we try fleeing N (default 5) times, prefer staying still if there's no enemy that close, and lay down and wait to lash out at enemies once we'd fled N times.
   - Paths for bots are saved in a data structure of future locations. Ie, turn A will have bot B at location (X, Y). This greatly improves the cooperative path-finding.
   - Add timers to make sure our path-finding and move scheduling doesn't time out (the game server imposes limits)
   - This was my most effective bot by far.
   
 - ring_of_death.py
   - An attempt at making an amorphous blob of bots that surrounds enemies, essentially eating them into the blob and then attacks them from all directions.
   - Uses the blob wall as a defense mechanism, with injured bots rotated to the side of the ring with the fewest enemies, or inside the ring.
   - I liked the idea behind this bot but it didn't work well in practice. The ring only really works when you have a bot count advantage over the enemy. It would be best used as part of a multi-mode bot that detects it's in the lead and falls into Ring Of Death mode.