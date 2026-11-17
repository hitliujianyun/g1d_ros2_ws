// WebSocket link to g1d_web: JSON text messages + binary frames
// [u8 kind][3 reserved][u32 LE header length][header JSON][payload, 4-byte aligned].

export const KIND = { SCAN: 1, MAP: 2, MAP_RESET: 3, OBST: 4, CHASSIS: 5, LOCAL: 6, GLOBAL: 7 };

export class Net {
  constructor({ onJson, onBinary, onOpen, onClose }) {
    this.onJson = onJson; this.onBinary = onBinary; this.onOpen = onOpen; this.onClose = onClose;
    this.ws = null; this.decoder = new TextDecoder(); this.retry = 500;
  }

  connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;
    ws.onopen = () => { this.retry = 500; this.onOpen && this.onOpen(); };
    ws.onclose = () => {
      this.onClose && this.onClose();
      setTimeout(() => this.connect(), this.retry);
      this.retry = Math.min(this.retry * 2, 5000);
    };
    ws.onerror = () => ws.close();
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') {
        let msg;
        try { msg = JSON.parse(ev.data); } catch (e) { return; }
        this.onJson(msg);
      } else {
        const buf = ev.data;
        const dv = new DataView(buf);
        const kind = dv.getUint8(0);
        const hl = dv.getUint32(4, true);
        const header = JSON.parse(this.decoder.decode(new Uint8Array(buf, 8, hl)));
        this.onBinary(kind, header, buf, 8 + hl);
      }
    };
  }

  get open() { return this.ws && this.ws.readyState === WebSocket.OPEN; }

  send(obj) {
    if (this.open) { this.ws.send(JSON.stringify(obj)); return true; }
    return false;
  }
}
