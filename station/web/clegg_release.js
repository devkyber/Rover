// Intent-only Release metadata. No sensor polling, plots or motion commands.
// The hammer controller dispatches this event once after an explicit gated press.
export function installCleggReleaseBridge({ target = document, canRecord = true,
  request = (...args) => fetch(...args), onResult = () => {} } = {}) {
  let closed = false;
  const seenRequestIds = new Set();
  const report = (id, text, bad = false) => {
    if (!closed) onResult(`${text} · Request ${id}`, bad);
  };
  const send = async detail => {
    if (closed || !canRecord || typeof detail?.request_id !== 'string' || !detail.request_id
        || !Number.isSafeInteger(detail.hammer_seq) || detail.hammer_seq < 0
        || seenRequestIds.has(detail.request_id)) return;
    const id = detail.request_id;
    seenRequestIds.add(id);
    const body = { request_id: detail.request_id, hammer_seq: detail.hammer_seq };
    if (Number.isFinite(detail.browser_time_ms)) body.browser_time_ms = detail.browser_time_ms;
    try {
      const response = await request('/clegg/release', { method: 'POST', cache: 'no-store',
        headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
        signal: AbortSignal.timeout(4000) });
      if (!response?.ok) {
        const code = Number.isInteger(response?.status) ? `HTTP ${response.status}` : 'no confirmation';
        report(id, `Release metadata failed · ${code}`, true);
        return;
      }
      report(id, 'Release metadata accepted');
    } catch (error) {
      const reason = ['TimeoutError', 'AbortError'].includes(error?.name) ? 'timeout' : 'network error';
      report(id, `Release metadata unconfirmed · ${reason}`, true);
    }
  };
  const listener = event => { void send(event.detail); };
  target.addEventListener('clegg-release-requested', listener);
  return { close() { closed = true; target.removeEventListener('clegg-release-requested', listener); } };
}
