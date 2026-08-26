import { navigationMissionJson, parseNavigationMissionJson } from "./live-navigation-state.js";

export const NAVIGATION_PLAN_LIBRARY_SCHEMA_VERSION = 1;
export const MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES = 20;
export const NAVIGATION_PLAN_NAME_MAX_LENGTH = 64;

function boundedName(value, fallback = "Untitled route") {
  const name = String(value ?? "")
    .replace(/[\u0000-\u001f\u007f]/g, " ")
    .replace(/\s+/g, " ")
    .trim()
    .slice(0, NAVIGATION_PLAN_NAME_MAX_LENGTH);
  return name || fallback;
}

function finiteTimestamp(value, fallback = Date.now()) {
  const timestamp = Number(value);
  return Number.isFinite(timestamp) && timestamp >= 0 ? timestamp : fallback;
}

function planJson(plan) {
  if (typeof plan === "string") return plan;
  if (plan && typeof plan === "object") {
    if (Array.isArray(plan.missionDraft)) return navigationMissionJson(plan);
    return JSON.stringify(plan);
  }
  return "{}";
}

export function normalizeNavigationPlanEntry(candidate, now = Date.now()) {
  if (!candidate || typeof candidate !== "object" || Array.isArray(candidate)) {
    throw new Error("Saved route entry must be an object.");
  }
  const planSource = candidate.plan && typeof candidate.plan === "object" ? candidate.plan : candidate;
  const plan = parseNavigationMissionJson(planJson(planSource));
  return {
    name: boundedName(candidate.name || plan.name),
    savedAt: finiteTimestamp(candidate.savedAt, now),
    plan,
  };
}

export function parseNavigationPlanLibraryJson(text, now = Date.now()) {
  let parsed;
  try {
    parsed = JSON.parse(String(text));
  } catch {
    throw new Error("Route library is not valid JSON.");
  }
  const entries = Array.isArray(parsed) ? parsed : parsed?.schemaVersion === NAVIGATION_PLAN_LIBRARY_SCHEMA_VERSION ? parsed.plans : null;
  if (!Array.isArray(entries) || entries.length > MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES) {
    throw new Error(`Route library must contain up to ${MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES} saved plans.`);
  }
  return entries.map((entry) => normalizeNavigationPlanEntry(entry, now));
}

export function navigationPlanLibraryJson(entries) {
  const plans = (Array.isArray(entries) ? entries : [])
    .slice(0, MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES)
    .map((entry) => normalizeNavigationPlanEntry(entry));
  return `${JSON.stringify({
    schemaVersion: NAVIGATION_PLAN_LIBRARY_SCHEMA_VERSION,
    plans,
  }, null, 2)}\n`;
}

export function upsertNavigationPlanLibraryEntry(entries, name, plan, savedAt = Date.now()) {
  const next = Array.isArray(entries) ? entries.map((entry) => normalizeNavigationPlanEntry(entry, savedAt)) : [];
  const normalized = normalizeNavigationPlanEntry({ name, plan, savedAt }, savedAt);
  const existingIndex = next.findIndex((entry) => entry.name.toLowerCase() === normalized.name.toLowerCase());
  if (existingIndex >= 0) next.splice(existingIndex, 1);
  next.unshift(normalized);
  return next.slice(0, MAX_NAVIGATION_PLAN_LIBRARY_ENTRIES);
}

export function removeNavigationPlanLibraryEntry(entries, name) {
  const target = boundedName(name, "").toLowerCase();
  if (!target) return Array.isArray(entries) ? entries.slice() : [];
  return (Array.isArray(entries) ? entries : [])
    .filter((entry) => boundedName(entry?.name, "").toLowerCase() !== target)
    .map((entry) => normalizeNavigationPlanEntry(entry));
}
