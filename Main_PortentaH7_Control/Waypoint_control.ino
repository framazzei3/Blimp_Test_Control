

void loadCurrentWaypoint() {
  if (currentWaypointIndex < 0 || currentWaypointIndex >= waypointCount) {
    log("[WAYPOINT] Invalid waypoint index");
    return;
  }

  target = waypointQueue[currentWaypointIndex];

  hoverX   = target.x;
  hoverY   = target.y;
  hoverZ   = target.z;
  hoverYaw = target.yaw;

  log("[WAYPOINT] Target set to WP#" + String(currentWaypointIndex));
}

void applyWaypointControl() {
  if (!missionActive || waypointCount == 0) {
    return;
  }

  if (currentWaypointIndex >= waypointCount) {
    log("[WAYPOINT] Waypoint index out of range");
    missionActive = false;
    return;
  }

  // Carica il waypoint solo quando cambia
  static int lastWaypointIndex = -1;
  if (currentWaypointIndex != lastWaypointIndex) {
    loadCurrentWaypoint();
    lastWaypointIndex = currentWaypointIndex;
    waypointReached = false;
  }
  if (currentWaypointIndex != lastWaypointIndex) {
    loadCurrentWaypoint();
    lastWaypointIndex = currentWaypointIndex;
  }

  // Esegui il controllo standard di hover
  applyHoverControl();

  // Calcola distanza residua                             
  float dx = target.x - mocapData.posX;
  float dy = target.y - mocapData.posY;
  float dz = target.z - mocapData.posZ;

  distToTarget  = sqrt(dx*dx + dy*dy + dz*dz);

  // Verifica raggiungimento waypoint
  if (distToTarget < 0.2f) {   // soglia in metri
    waypointReached = true;
    log("[WAYPOINT] WP#" + String(currentWaypointIndex) + " reached");

    // Passa al waypoint successivo
    if (currentWaypointIndex < waypointCount - 1) {
      currentWaypointIndex++;
      log("[WAYPOINT] Moving to WP#" + String(currentWaypointIndex));
    }
    // Missione completata
    else {
      log("[MISSION] Mission complete");

      missionActive = false;
      currentControlMode = MODE_HOVER;

      // Reset target
      target.x = 0.0f;
      target.y = 0.0f;
      target.z = 0.0f;
      target.yaw = 0.0f;

      distToTarget = 0.0f;
      waypointReached = false;

      log("[MISSION] Mission stopped, target reset");
    }
  }
}








