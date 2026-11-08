export type VisualizerViewPreferences = {
  robotStyle: "mesh" | "skeleton";
  showGripperMeshes: boolean;
  showTrajectoryPaths: boolean;
  showBaseAxes: boolean;
  showActualTcp: boolean;
  showTargetTcp: boolean;
  showGrid: boolean;
  showSceneObjects: boolean;
  showObjectLabels: boolean;
  cameraPosition: [number, number, number];
  cameraTarget: [number, number, number];
  objectLabels: Record<string, boolean>;
  hiddenObjects: Record<string, boolean>;
};

export const VISUALIZER_VIEW_STORAGE_KEY = "cr5a-visualizer-view-preferences-v1";

export const DEFAULT_VISUALIZER_VIEW_PREFERENCES: VisualizerViewPreferences = {
  robotStyle: "mesh",
  showGripperMeshes: true,
  showTrajectoryPaths: true,
  showBaseAxes: false,
  showActualTcp: false,
  showTargetTcp: false,
  showGrid: true,
  showSceneObjects: true,
  showObjectLabels: false,
  cameraPosition: [1.25, 1.05, 1.35],
  cameraTarget: [0, 0.25, 0],
  objectLabels: {},
  hiddenObjects: {},
};

function isRecord(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function readBooleanMap(value: unknown): Record<string, boolean> {
  if (!isRecord(value)) return {};
  const result: Record<string, boolean> = {};
  Object.entries(value).forEach(([key, item]) => {
    if (key !== "__proto__" && key !== "constructor" && typeof item === "boolean") {
      result[key] = item;
    }
  });
  return result;
}

function readVector(value: unknown, fallback: [number, number, number]): [number, number, number] {
  if (!Array.isArray(value) || value.length !== 3 || !value.every(
    (item) => typeof item === "number" && Number.isFinite(item)
  )) return fallback;
  return [value[0], value[1], value[2]];
}

export function loadVisualizerViewPreferences(): VisualizerViewPreferences {
  if (typeof window === "undefined") return DEFAULT_VISUALIZER_VIEW_PREFERENCES;
  try {
    const stored = window.localStorage.getItem(VISUALIZER_VIEW_STORAGE_KEY);
    if (!stored) return DEFAULT_VISUALIZER_VIEW_PREFERENCES;
    const parsed: unknown = JSON.parse(stored);
    if (!isRecord(parsed)) return DEFAULT_VISUALIZER_VIEW_PREFERENCES;
    const defaults = DEFAULT_VISUALIZER_VIEW_PREFERENCES;
    return {
      robotStyle: parsed.robotStyle === "skeleton" ? "skeleton" : "mesh",
      showGripperMeshes: typeof parsed.showGripperMeshes === "boolean" ? parsed.showGripperMeshes : defaults.showGripperMeshes,
      showTrajectoryPaths: typeof parsed.showTrajectoryPaths === "boolean" ? parsed.showTrajectoryPaths : defaults.showTrajectoryPaths,
      showBaseAxes: typeof parsed.showBaseAxes === "boolean" ? parsed.showBaseAxes : defaults.showBaseAxes,
      showActualTcp: typeof parsed.showActualTcp === "boolean" ? parsed.showActualTcp : defaults.showActualTcp,
      showTargetTcp: typeof parsed.showTargetTcp === "boolean" ? parsed.showTargetTcp : defaults.showTargetTcp,
      showGrid: typeof parsed.showGrid === "boolean" ? parsed.showGrid : defaults.showGrid,
      showSceneObjects: typeof parsed.showSceneObjects === "boolean" ? parsed.showSceneObjects : defaults.showSceneObjects,
      showObjectLabels: typeof parsed.showObjectLabels === "boolean" ? parsed.showObjectLabels : defaults.showObjectLabels,
      cameraPosition: readVector(parsed.cameraPosition, defaults.cameraPosition),
      cameraTarget: readVector(parsed.cameraTarget, defaults.cameraTarget),
      objectLabels: readBooleanMap(parsed.objectLabels),
      hiddenObjects: readBooleanMap(parsed.hiddenObjects),
    };
  } catch {
    return DEFAULT_VISUALIZER_VIEW_PREFERENCES;
  }
}

export function saveVisualizerViewPreferences(preferences: VisualizerViewPreferences): void {
  try {
    window.localStorage.setItem(VISUALIZER_VIEW_STORAGE_KEY, JSON.stringify(preferences));
  } catch {
    // Storage can be unavailable or full; the visualizer remains usable in memory.
  }
}
