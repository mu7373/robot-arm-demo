/**
 * server.mjs —— 零依赖静态服务器
 *
 * 为什么必须有它：index.html / analysis_test.html 里用的是 ES Module，
 * 双击打开（file:// 协议）会被浏览器的同源策略拦掉，报
 *   "Failed to resolve module specifier" 或 "CORS policy"。
 * 必须走 http:// 才能加载 .js 模块和 .glb 模型。
 *
 * 用法：
 *   node server.mjs              默认 8123 端口
 *   PORT=8080 node server.mjs    换端口
 *   HOST=127.0.0.1 node server.mjs  只允许本机访问（默认 0.0.0.0，同局域网可访问）
 */

import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { extname, join, normalize, sep } from 'node:path';
import { networkInterfaces } from 'node:os';

const PORT = Number(process.env.PORT) || 8123;
const HOST = process.env.HOST || '0.0.0.0';
const ROOT = process.cwd();

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.glb': 'model/gltf-binary',
  '.gltf': 'model/gltf+json',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.gif': 'image/gif',
  '.svg': 'image/svg+xml',
  '.yaml': 'text/yaml; charset=utf-8',
  '.yml': 'text/yaml; charset=utf-8',
  '.urdf': 'text/xml; charset=utf-8',
  '.docx': 'application/vnd.openxmlformats-officedocument.wordprocessingml.document',
  '.xlsx': 'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet'
};

const server = createServer(async (req, res) => {
  try {
    const url = new URL(req.url, `http://${req.headers.host}`);
    let rel = decodeURIComponent(url.pathname);
    if (rel === '/' || rel === '') rel = '/index.html';

    // 防目录穿越：解析后必须仍在 ROOT 之内
    const target = normalize(join(ROOT, rel));
    if (!target.startsWith(ROOT.endsWith(sep) ? ROOT : ROOT + sep)) {
      res.writeHead(403).end('Forbidden');
      return;
    }

    const st = await stat(target).catch(() => null);
    if (!st || st.isDirectory()) {
      res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
      res.end(`404 Not Found: ${rel}\n\n确认文件存在，且服务是在项目根目录启动的。`);
      return;
    }

    const body = await readFile(target);
    res.writeHead(200, {
      'Content-Type': MIME[extname(target).toLowerCase()] ?? 'application/octet-stream',
      'Content-Length': body.length,
      'Cache-Control': 'no-store'
    });
    res.end(body);
  } catch (e) {
    res.writeHead(500, { 'Content-Type': 'text/plain; charset=utf-8' }).end('500 ' + e.message);
  }
});

server.listen(PORT, HOST, () => {
  console.log(`\n  robot-arm-demo 静态服务已启动`);
  console.log(`  本机：     http://127.0.0.1:${PORT}/analysis_test.html`);
  console.log(`  示教器：   http://127.0.0.1:${PORT}/index.html`);

  if (HOST === '0.0.0.0') {
    const nets = networkInterfaces();
    const ips = [];
    for (const list of Object.values(nets)) {
      for (const n of list ?? []) {
        if (n.family === 'IPv4' && !n.internal) ips.push(n.address);
      }
    }
    if (ips.length) {
      console.log(`  局域网（同 WiFi 的同学可用）：`);
      for (const ip of ips) console.log(`     http://${ip}:${PORT}/analysis_test.html`);
      console.log(`  若打不开，多半是 Windows 防火墙，需要用管理员 PowerShell 放行：`);
      console.log(`     netsh advfirewall firewall add rule name="robot-arm ${PORT}" dir=in action=allow protocol=TCP localport=${PORT}`);
    }
  }
  console.log(`\n  按 Ctrl+C 停止。这个窗口就是服务本身，用的时候别关。\n`);
});
