import {
  LIVE_SESSION_MAX_ARCHIVE_ENTRIES,
  sanitizeArchivedLiveSession,
} from "./live-session-state.js";

const DATABASE_NAME = "domino-virtual-lab";
const DATABASE_VERSION = 1;
const STORE_NAME = "live-sessions";

const requestResult = (request) => new Promise((resolve, reject) => {
  request.onsuccess = () => resolve(request.result);
  request.onerror = () => reject(request.error || new Error("IndexedDB request failed"));
});

function openDatabase(factory) {
  return new Promise((resolve, reject) => {
    const request = factory.open(DATABASE_NAME, DATABASE_VERSION);
    request.onupgradeneeded = () => {
      const database = request.result;
      if (!database.objectStoreNames.contains(STORE_NAME)) {
        database.createObjectStore(STORE_NAME, { keyPath: "id" });
      }
    };
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error || new Error("Unable to open the session archive"));
  });
}

function transactionDone(transaction) {
  return new Promise((resolve, reject) => {
    transaction.oncomplete = () => resolve();
    transaction.onabort = () => reject(transaction.error || new Error("Session archive transaction aborted"));
    transaction.onerror = () => reject(transaction.error || new Error("Session archive transaction failed"));
  });
}

export function createLiveSessionRepository(factory = globalThis.indexedDB) {
  if (!factory?.open) {
    return {
      available: false,
      load: async () => [],
      save: async () => false,
      remove: async () => false,
    };
  }
  const database = openDatabase(factory);
  return {
    available: true,
    async load() {
      const db = await database;
      const transaction = db.transaction(STORE_NAME, "readonly");
      const values = await requestResult(transaction.objectStore(STORE_NAME).getAll());
      return values.map(sanitizeArchivedLiveSession).filter(Boolean)
        .sort((left, right) => right.stoppedAt - left.stoppedAt)
        .slice(0, LIVE_SESSION_MAX_ARCHIVE_ENTRIES);
    },
    async save(candidate) {
      const session = sanitizeArchivedLiveSession(candidate);
      if (!session) return false;
      const db = await database;
      const transaction = db.transaction(STORE_NAME, "readwrite");
      const store = transaction.objectStore(STORE_NAME);
      store.put(session);
      const values = await requestResult(store.getAll());
      values.sort((left, right) => Number(right.stoppedAt) - Number(left.stoppedAt));
      values.slice(LIVE_SESSION_MAX_ARCHIVE_ENTRIES).forEach((entry) => store.delete(entry.id));
      await transactionDone(transaction);
      return true;
    },
    async remove(identifier) {
      const db = await database;
      const transaction = db.transaction(STORE_NAME, "readwrite");
      transaction.objectStore(STORE_NAME).delete(String(identifier));
      await transactionDone(transaction);
      return true;
    },
  };
}
