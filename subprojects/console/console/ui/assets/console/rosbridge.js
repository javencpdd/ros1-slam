/* Minimal rosbridge v2 client for the Lite3 console.
 * Only publish / subscribe / call_service are implemented; no roslibjs CDN. */
(function (global) {
  'use strict';

  class RosBridge {
    constructor(url, handlers) {
      this.url = url;
      this.handlers = handlers || {};
      this.ws = null;
      this.advertised = new Set();
      this.subscribed = new Set();
      this.subs = Object.create(null);
      this.calls = Object.create(null);
      this.nextId = 1;
      this.online = false;
    }

    connect() {
      this.close();
      const ws = new WebSocket(this.url);
      this.ws = ws;
      ws.onopen = () => {
        this.online = true;
        this.advertised.clear();
        this.subscribed.clear();
        this._status('online');
        if (this.handlers.open) this.handlers.open();
      };
      ws.onclose = () => {
        this.online = false;
        this._status('offline');
        if (this.handlers.close) this.handlers.close();
      };
      ws.onerror = () => {
        this._status('error');
      };
      ws.onmessage = (event) => {
        let msg;
        try { msg = JSON.parse(event.data); } catch (e) { return; }
        if (msg.op === 'publish' && this.subs[msg.topic]) {
          this.subs[msg.topic](msg.msg);
        } else if (msg.op === 'service_response' && this.calls[msg.id]) {
          const cb = this.calls[msg.id];
          delete this.calls[msg.id];
          cb(msg);
        }
      };
    }

    close() {
      if (this.ws) {
        try { this.ws.onclose = null; this.ws.close(); } catch (e) {}
        this.ws = null;
      }
      this.online = false;
    }

    _status(state) {
      if (this.handlers.status) this.handlers.status(state);
    }

    _send(obj) {
      if (!this.ws || this.ws.readyState !== 1) throw new Error('rosbridge not connected');
      this.ws.send(JSON.stringify(obj));
    }

    advertise(topic, type) {
      if (this.advertised.has(topic)) return;
      this._send({ op: 'advertise', topic: topic, type: type });
      this.advertised.add(topic);
    }

    publish(topic, type, msg) {
      this.advertise(topic, type);
      this._send({ op: 'publish', topic: topic, msg: msg });
    }

    subscribe(topic, type, cb, throttle_rate) {
      this.subs[topic] = cb;
      if (this.subscribed.has(topic)) return;
      const msg = { op: 'subscribe', topic: topic, type: type, queue_length: 1 };
      if (throttle_rate) msg.throttle_rate = throttle_rate;
      this._send(msg);
      this.subscribed.add(topic);
    }

    unsubscribe(topic) {
      delete this.subs[topic];
      if (!this.subscribed.has(topic)) return;
      this._send({ op: 'unsubscribe', topic: topic });
      this.subscribed.delete(topic);
    }

    callService(service, args, cb) {
      const id = 'call_' + (this.nextId++);
      this.calls[id] = cb;
      this._send({ op: 'call_service', service: service, args: args || {}, id: id });
      setTimeout(() => {
        if (this.calls[id]) {
          delete this.calls[id];
          cb({ result: false, values: {}, error: 'timeout' });
        }
      }, 5000);
    }
  }

  global.RosBridge = RosBridge;
})(window);
