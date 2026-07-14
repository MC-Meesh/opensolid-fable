#!/usr/bin/env node
// MCP stdio server for the OpenSolid kernel. Speaks JSON-RPC 2.0 over
// newline-delimited stdin/stdout (the MCP stdio transport). Diagnostics go to
// stderr so they never corrupt the protocol stream.

import { createInterface } from 'node:readline';
import { createTools } from './tools.js';

const PROTOCOL_VERSION = '2024-11-05';
const SERVER_INFO = { name: 'opensolid-mcp-server', version: '0.1.0' };

const tools = createTools({ outputDir: process.env.OPENSOLID_MCP_OUTPUT_DIR });

function send(message) {
  process.stdout.write(JSON.stringify(message) + '\n');
}

function respond(id, result) {
  send({ jsonrpc: '2.0', id, result });
}

function respondError(id, code, message) {
  send({ jsonrpc: '2.0', id, error: { code, message } });
}

// JSON-RPC / MCP method dispatch. Returns a result object, or throws
// {code, message} for a protocol-level error. Notifications (no id) return
// undefined and produce no response.
function handle(msg) {
  switch (msg.method) {
    case 'initialize':
      return {
        protocolVersion: msg.params?.protocolVersion || PROTOCOL_VERSION,
        capabilities: { tools: {} },
        serverInfo: SERVER_INFO,
      };
    case 'ping':
      return {};
    case 'tools/list':
      return { tools: tools.definitions };
    case 'tools/call': {
      const name = msg.params?.name;
      const args = msg.params?.arguments || {};
      return tools.call(name, args);
    }
    case 'notifications/initialized':
    case 'notifications/cancelled':
      return undefined; // notification, no reply
    default:
      throw { code: -32601, message: `method not found: ${msg.method}` };
  }
}

const rl = createInterface({ input: process.stdin });

rl.on('line', (line) => {
  const trimmed = line.trim();
  if (!trimmed) return;

  let msg;
  try {
    msg = JSON.parse(trimmed);
  } catch {
    respondError(null, -32700, 'parse error');
    return;
  }

  const isNotification = msg.id === undefined || msg.id === null;
  try {
    const result = handle(msg);
    if (!isNotification) {
      respond(msg.id, result);
    }
  } catch (err) {
    if (isNotification) return;
    const code = typeof err?.code === 'number' ? err.code : -32603;
    const message = err?.message || 'internal error';
    respondError(msg.id, code, message);
  }
});

rl.on('close', () => process.exit(0));

process.stderr.write(
  `opensolid-mcp-server ready (output dir: ${tools.outputDir})\n`,
);
