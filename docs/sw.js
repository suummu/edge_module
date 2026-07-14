/* 엣지알리미 서비스 워커 — 앱 셸 캐시(오프라인 데모 모드 지원)
 * 판정 데이터(/data, /history)는 항상 네트워크 우선: 감시 화면에
 * 낡은 판정을 보여주는 것이 최악이므로, 실패 시 캐시 대신 실패를 드러낸다. */
const SHELL = "edgealimi-shell-v1";
self.addEventListener("install", e => {
  e.waitUntil(caches.open(SHELL).then(c => c.addAll(["/"])));
  self.skipWaiting();
});
self.addEventListener("activate", e => {
  e.waitUntil(caches.keys().then(keys =>
    Promise.all(keys.filter(k => k !== SHELL).map(k => caches.delete(k)))));
});
self.addEventListener("fetch", e => {
  const url = new URL(e.request.url);
  if (url.pathname === "/data" || url.pathname === "/history"
      || url.pathname === "/health" || e.request.method !== "GET") return;
  e.respondWith(
    caches.match(e.request).then(hit => hit || fetch(e.request))
  );
});
