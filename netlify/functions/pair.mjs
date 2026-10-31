// One-scan pairing: the viewer leaves its reply here and the tripod collects
// it, so nobody has to hold a second QR code up to a camera. A reply holds
// connection details only, never video or images, and is deleted once read.
import { getStore } from '@netlify/blobs';

const ALLOWED_HOSTS = new Set(['vellumproject.netlify.app', 'localhost:8888']);
const MAX_AGE_MS = 10 * 60 * 1000;

export default async (req) => {
  const site = req.headers.get('origin') || req.headers.get('referer');
  let host = '';
  try { host = new URL(site).host; } catch (e) {}
  const sameSite = req.headers.get('sec-fetch-site') === 'same-origin';
  if (!sameSite && !ALLOWED_HOSTS.has(host)) return new Response('Forbidden', { status: 403 });

  const id = new URL(req.url).searchParams.get('id') || '';
  if (!/^[A-Za-z0-9_-]{16,32}$/.test(id)) return new Response('Bad id', { status: 400 });
  const store = getStore({ name: 'vellum-pair', consistency: 'strong' });

  if (req.method === 'POST') {
    const reply = await req.text();
    if (reply.length > 1024 || !reply.startsWith('VA~1~')) return new Response('Bad reply', { status: 400 });
    await store.setJSON(id, { reply, at: Date.now() });
    return new Response(null, { status: 204 });
  }
  if (req.method !== 'GET') return new Response('Method not allowed', { status: 405 });

  const entry = await store.get(id, { type: 'json' });
  if (!entry) return new Response(null, { status: 204, headers: { 'Cache-Control': 'no-store' } });
  await store.delete(id);
  if (Date.now() - entry.at > MAX_AGE_MS) return new Response(null, { status: 204 });
  return new Response(entry.reply, { headers: { 'Cache-Control': 'no-store' } });
};

export const config = { path: '/api/pair' };
