export const WORKSPACE_SIMULATION = "simulation";
export const WORKSPACE_REAL_ROBOT = "real-robot";

export const EXPERIENCE_SIMPLE = "simple";
export const EXPERIENCE_EXPERT = "expert";

const VALID_WORKSPACES = new Set([
  WORKSPACE_SIMULATION,
  WORKSPACE_REAL_ROBOT,
]);

const VALID_EXPERIENCES = new Set([
  EXPERIENCE_SIMPLE,
  EXPERIENCE_EXPERT,
]);

export function createApplicationState(overrides = {}) {
  return {
    workspace: VALID_WORKSPACES.has(overrides.workspace)
      ? overrides.workspace
      : WORKSPACE_SIMULATION,
    experience: VALID_EXPERIENCES.has(overrides.experience)
      ? overrides.experience
      : EXPERIENCE_SIMPLE,
  };
}

export function selectWorkspace(state, workspace) {
  if (!VALID_WORKSPACES.has(workspace)) {
    throw new RangeError(`Unknown Domino workspace: ${workspace}`);
  }
  state.workspace = workspace;
  return state;
}

export function simulationCanOwnControl(state, visibilityState = "visible") {
  return state.workspace === WORKSPACE_SIMULATION && visibilityState === "visible";
}

export function selectExperience(state, experience) {
  if (!VALID_EXPERIENCES.has(experience)) {
    throw new RangeError(`Unknown Domino experience level: ${experience}`);
  }
  state.experience = experience;
  return state;
}
