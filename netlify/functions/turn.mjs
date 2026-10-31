// Hands each Vellum session a temporary relay login, so the long-term
// Cloudflare key stays on the server and never reaches a browser.
// Without the two environment variables this answers 503 and Vellum simply
// connects directly, the same as before.
const ALLOWED_HOSTS = new Set(['vellumproject.netlify.app', 'localhost:8888']);
const TTL_SECONDS = 12 * 60 * 60; // longer than a painting session

export default async (req) => {
  // Keep other sites from borrowing this relay allowance.
  const origin = req.headers.get('origin');
  const site = origin || req.headers.get('referer');
  let host = '';
  try { host = new URL(site).host; } catch (e) {}
  // Same-site requests don't always carry those headers; browsers mark them.
  const sameSite = req.headers.get('sec-fetch-site') === 'same-origin';
  if (!sameSite && !ALLOWED_HOSTS.has(host)) return new Response('Forbidden', { status: 403 });

  const keyId = process.env.TURN_KEY_ID;
  const token = process.env.TURN_KEY_API_TOKEN;
  if (!keyId || !token) return new Response('Relay not configured', { status: 503 });

  const res = await fetch(
    `https://rtc.live.cloudflare.com/v1/turn/keys/${keyId}/credentials/generate-ice-servers`,
    {
      method: 'POST',
      headers: { Authorization: `Bearer ${token}`, 'Content-Type': 'application/json' },
      body: JSON.stringify({ ttl: TTL_SECONDS }),
    }
  );
  if (!res.ok) return new Response('Relay unavailable', { status: 502 });

  const { iceServers } = await res.json();
  // Browsers block port 53, and Vellum waits for every address before making
  // its pairing code, so a blocked one would only add a delay.
  const usable = iceServers.map((s) => ({
    ...s,
    urls: [].concat(s.urls).filter((u) => !/:53(\?|$)/.test(u)),
  }));
  return Response.json(usable, { headers: { 'Cache-Control': 'no-store' } });
};

export const config = { path: '/api/turn' };
