// Lets Vellum open with no internet once installed. Always tries the network
// first, so a device online gets the newest version, not a stale copy.
var CACHE = 'vellum-v1';
var FILES = ['/', '/qrcode.min.js', '/jsQR.min.js', '/manifest.webmanifest', '/icon-192.png', '/icon-512.png'];

self.addEventListener('install', function(e){
  e.waitUntil(caches.open(CACHE).then(function(c){ return c.addAll(FILES); }).then(function(){ return self.skipWaiting(); }));
});
self.addEventListener('activate', function(e){
  e.waitUntil(caches.keys().then(function(keys){
    return Promise.all(keys.filter(function(k){ return k !== CACHE; }).map(function(k){ return caches.delete(k); }));
  }).then(function(){ return self.clients.claim(); }));
});
self.addEventListener('fetch', function(e){
  var url = new URL(e.request.url);
  // Relay logins are temporary and must never be stored.
  if (e.request.method !== 'GET' || url.origin !== location.origin || url.pathname.indexOf('/api/') === 0) return;
  var key = e.request.mode === 'navigate' ? '/' : e.request;
  e.respondWith(fetch(e.request).then(function(res){
    if (res.ok) {
      var copy = res.clone();
      caches.open(CACHE).then(function(c){ c.put(key, copy); });
    }
    return res;
  }).catch(function(){ return caches.match(key); }));
});
