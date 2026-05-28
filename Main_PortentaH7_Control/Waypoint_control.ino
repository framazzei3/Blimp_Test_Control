

void loadCurrentWaypoint() {
  if (currentWaypointIndex < 0 || currentWaypointIndex >= waypointCount) {
    log("[WAYPOINT] Invalid waypoint index");
    return;
  }

  target = waypointQueue[currentWaypointIndex];

  hoverX   = target.x;
  hoverY   = target.y;
  hoverZ   = target.z;
  z_ref = target.z;
  hoverYaw = target.yaw;
  
  // Reset integratori
  int_v = 0.0f;
  int_w = 0.0f;

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
  if (currentWaypointIndex != lastWaypointIndex) {
    loadCurrentWaypoint();
    lastWaypointIndex = currentWaypointIndex;
    waypointReached = false;
  }

  // Esegui il controllo standard di hover
  applyHoverControl();
  applyVerticalControl();

  // Calcola distanza residua                             
  float dx = target.x - mocapData.posX;
  float dy = target.y - mocapData.posY;
  float dz = target.z - mocapData.posZ;

  distToTarget  = sqrt(dx*dx + dy*dy + dz*dz);

  logCSVMissionRow();  // sincronizzato con il loop a 20Hz

  // Verifica raggiungimento waypoint
  if (distToTarget < 0.4f) {   // soglia in metri (30 cm)
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
      currentControlMode = MODE_SAFETY;
      hoverMode = HOVER_HOLD; // doppio check, gia viene chiamato in MODE_HOVER
     
      distToTarget = 0.0f;
      waypointReached = false;

      log("[MISSION] Switched to SAFETY MODE");
    }
  }
}
