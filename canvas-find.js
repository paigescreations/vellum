// Vellum canvas finder: spots the canvas in a camera frame, measures how
// square the camera is to it (for the coaching), and compares two frames
// for the bump alerts. No page code in here, so it also runs as a
// background worker and the camera picture never stutters while it looks.
var VellumCore = (function(){
  "use strict";

  // ---------- small geometry helpers ----------
  function d2(a, b){ var dx = b.x - a.x, dy = b.y - a.y; return Math.sqrt(dx*dx + dy*dy); }
  function polyArea(q){
    var s = 0;
    for (var i = 0; i < q.length; i++){
      var a = q[i], b = q[(i + 1) % q.length];
      s += a.x * b.y - b.x * a.y;
    }
    return Math.abs(s) / 2;
  }
  // interior angle at b (degrees) for a-b-c
  function cornerAngle(a, b, c){
    var v1x = a.x - b.x, v1y = a.y - b.y, v2x = c.x - b.x, v2y = c.y - b.y;
    var dot = v1x*v2x + v1y*v2y;
    var m = Math.sqrt((v1x*v1x + v1y*v1y) * (v2x*v2x + v2y*v2y)) || 1;
    return Math.acos(Math.max(-1, Math.min(1, dot / m))) * 180 / Math.PI;
  }

  // ---------- homography (maps 4 points to 4 points) ----------
  // Returns [a,b,c,d,e,f,g,h] with the bottom-right term fixed at 1.
  function solveHomography(src, dst){
    var A = [], i;
    for (i = 0; i < 4; i++){
      var x = src[i].x, y = src[i].y, X = dst[i].x, Y = dst[i].y;
      A.push([x, y, 1, 0, 0, 0, -x*X, -y*X, X]);
      A.push([0, 0, 0, x, y, 1, -x*Y, -y*Y, Y]);
    }
    // Gaussian elimination with partial pivoting
    for (var col = 0; col < 8; col++){
      var piv = col;
      for (var r = col + 1; r < 8; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
      if (Math.abs(A[piv][col]) < 1e-12) return null;
      var tmp = A[col]; A[col] = A[piv]; A[piv] = tmp;
      for (r = 0; r < 8; r++){
        if (r === col) continue;
        var f = A[r][col] / A[col][col];
        if (!f) continue;
        for (var k = col; k < 9; k++) A[r][k] -= f * A[col][k];
      }
    }
    var h = [];
    for (i = 0; i < 8; i++) h.push(A[i][8] / A[i][i]);
    return h;
  }
  function applyH(h, x, y){
    var w = h[6]*x + h[7]*y + 1;
    return { x: (h[0]*x + h[1]*y + h[2]) / w, y: (h[3]*x + h[4]*y + h[5]) / w };
  }
  function invertH(h){
    var a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],k=h[7],i=1;
    var A =  (e*i - f*k), B = -(d*i - f*g), C =  (d*k - e*g);
    var D = -(b*i - c*k), E =  (a*i - c*g), F = -(a*k - b*g);
    var G =  (b*f - c*e), H = -(a*f - c*d), I =  (a*e - b*d);
    var det = a*A + b*B + c*C;
    if (Math.abs(det) < 1e-12) return null;
    // inverse = adjugate^T / det; normalise so last term is 1
    var inv = [A, D, G, B, E, H, C, F, I];
    var n = inv[8];
    if (Math.abs(n) < 1e-12) return null;
    return [inv[0]/n, inv[1]/n, inv[2]/n, inv[3]/n, inv[4]/n, inv[5]/n, inv[6]/n, inv[7]/n];
  }
  function cssMatrix3d(h){
    var v = [h[0], h[3], 0, h[6], h[1], h[4], 0, h[7], 0, 0, 1, 0, h[2], h[5], 0, 1];
    return 'matrix3d(' + v.map(function(n){ return +n.toFixed(10); }).join(',') + ')';
  }

  // ---------- image helpers for detection ----------
  function boxBlur(src, w, h){
    var out = new Uint8ClampedArray(w*h), tmp = new Float32Array(w*h), x, y;
    for (y = 0; y < h; y++){
      var row = y*w;
      for (x = 0; x < w; x++){
        var l = x > 0 ? src[row+x-1] : src[row+x], r = x < w-1 ? src[row+x+1] : src[row+x];
        tmp[row+x] = (l + src[row+x] + r) / 3;
      }
    }
    for (y = 0; y < h; y++){
      for (x = 0; x < w; x++){
        var u = y > 0 ? tmp[(y-1)*w+x] : tmp[y*w+x], dn = y < h-1 ? tmp[(y+1)*w+x] : tmp[y*w+x];
        out[y*w+x] = (u + tmp[y*w+x] + dn) / 3;
      }
    }
    return out;
  }
  function otsu(img){
    var hist = new Float64Array(256), n = img.length, i;
    for (i = 0; i < n; i++) hist[img[i]]++;
    var sum = 0; for (i = 0; i < 256; i++) sum += i * hist[i];
    var sumB = 0, wB = 0, best = 0, thr = 128;
    for (i = 0; i < 256; i++){
      wB += hist[i]; if (!wB) continue;
      var wF = n - wB; if (!wF) break;
      sumB += i * hist[i];
      var mB = sumB / wB, mF = (sum - sumB) / wF;
      var between = wB * wF * (mB - mF) * (mB - mF);
      if (between > best){ best = between; thr = i; }
    }
    return { t: thr, spread: Math.sqrt(best / (n*n)) };
  }
  function dilate(m, w, h){
    var o = new Uint8Array(w*h);
    for (var y = 0; y < h; y++) for (var x = 0; x < w; x++){
      var v = 0;
      for (var dy = -1; dy <= 1 && !v; dy++){
        var yy = y + dy; if (yy < 0 || yy >= h) continue;
        for (var dx = -1; dx <= 1; dx++){
          var xx = x + dx; if (xx < 0 || xx >= w) continue;
          if (m[yy*w+xx]){ v = 1; break; }
        }
      }
      o[y*w+x] = v;
    }
    return o;
  }
  function erode(m, w, h){
    var o = new Uint8Array(w*h);
    for (var y = 0; y < h; y++) for (var x = 0; x < w; x++){
      var v = 1;
      for (var dy = -1; dy <= 1 && v; dy++){
        var yy = y + dy; if (yy < 0 || yy >= h) continue;
        for (var dx = -1; dx <= 1; dx++){
          var xx = x + dx; if (xx < 0 || xx >= w) continue;
          if (!m[yy*w+xx]){ v = 0; break; }
        }
      }
      o[y*w+x] = v;
    }
    return o;
  }

  // Label 4-connected components of a binary mask. Returns {labels, comps:[{id,area,minX,...}]}
  function components(mask, w, h, minArea){
    var labels = new Int32Array(w*h), stack = new Int32Array(w*h), comps = [], next = 1;
    for (var p = 0; p < w*h; p++){
      if (!mask[p] || labels[p]) continue;
      var sp = 0, area = 0, minX = w, minY = h, maxX = 0, maxY = 0;
      stack[sp++] = p; labels[p] = next;
      while (sp){
        var q = stack[--sp], qx = q % w, qy = (q / w) | 0;
        area++;
        if (qx < minX) minX = qx; if (qx > maxX) maxX = qx;
        if (qy < minY) minY = qy; if (qy > maxY) maxY = qy;
        if (qx > 0     && mask[q-1] && !labels[q-1]){ labels[q-1] = next; stack[sp++] = q-1; }
        if (qx < w-1   && mask[q+1] && !labels[q+1]){ labels[q+1] = next; stack[sp++] = q+1; }
        if (qy > 0     && mask[q-w] && !labels[q-w]){ labels[q-w] = next; stack[sp++] = q-w; }
        if (qy < h-1   && mask[q+w] && !labels[q+w]){ labels[q+w] = next; stack[sp++] = q+w; }
      }
      if (area >= minArea) comps.push({ id: next, area: area, minX: minX, minY: minY, maxX: maxX, maxY: maxY });
      next++;
    }
    return { labels: labels, comps: comps };
  }

  // Fit a line through points (total least squares). Returns {px,py,dx,dy} or null.
  function fitLine(pts){
    if (pts.length < 6) return null;
    var mx = 0, my = 0, i;
    for (i = 0; i < pts.length; i++){ mx += pts[i].x; my += pts[i].y; }
    mx /= pts.length; my /= pts.length;
    var sxx = 0, syy = 0, sxy = 0;
    for (i = 0; i < pts.length; i++){
      var dx = pts[i].x - mx, dy = pts[i].y - my;
      sxx += dx*dx; syy += dy*dy; sxy += dx*dy;
    }
    var ang = 0.5 * Math.atan2(2*sxy, sxx - syy);
    return { px: mx, py: my, dx: Math.cos(ang), dy: Math.sin(ang) };
  }
  function lineDist(L, p){ return Math.abs((p.x - L.px) * L.dy - (p.y - L.py) * L.dx); }
  function robustFit(pts){
    var L = fitLine(pts); if (!L) return null;
    for (var pass = 0; pass < 2; pass++){
      var keep = pts.filter(function(p){ return lineDist(L, p) < 1.6; });
      if (keep.length < Math.max(6, pts.length * 0.5)) break;
      L = fitLine(keep) || L;
    }
    return L;
  }
  function intersect(L1, L2){
    var den = L1.dx * L2.dy - L1.dy * L2.dx;
    if (Math.abs(den) < 1e-9) return null;
    var t = ((L2.px - L1.px) * L2.dy - (L2.py - L1.py) * L2.dx) / den;
    return { x: L1.px + t * L1.dx, y: L1.py + t * L1.dy };
  }

  // From one component, work out the four corners and how canvas-like it is.
  function quadFromComponent(labels, w, h, c){
    var id = c.id, tl = null, tr = null, br = null, bl = null;
    var sTL = Infinity, sBR = -Infinity, sTR = -Infinity, sBL = Infinity, x, y, p;
    for (y = c.minY; y <= c.maxY; y++){
      for (x = c.minX; x <= c.maxX; x++){
        if (labels[y*w+x] !== id) continue;
        var s = x + y, t = x - y;
        if (s < sTL){ sTL = s; tl = { x: x, y: y }; }
        if (s > sBR){ sBR = s; br = { x: x, y: y }; }
        if (t > sTR){ sTR = t; tr = { x: x, y: y }; }
        if (t < sBL){ sBL = t; bl = { x: x, y: y }; }
      }
    }
    if (!tl) return null;
    var rough = [tl, tr, br, bl];

    // Refine each side with a line fit through the outer edge, skipping
    // the rounded-off bits near the corners.
    function collect(a, b, dir){
      var pts = [], n = Math.max(12, Math.round(d2(a, b))), m = 0.14;
      for (var i = 0; i <= n; i++){
        var f = m + (1 - 2*m) * (i / n);
        var px = a.x + (b.x - a.x) * f, py = a.y + (b.y - a.y) * f;
        var ix = Math.round(px), iy = Math.round(py), k, reach = 14;
        if (dir === 'top' || dir === 'bottom'){
          if (ix < 0 || ix >= w) continue;
          var step = dir === 'top' ? 1 : -1, yy = iy - step * reach;
          for (k = 0; k < reach * 2; k++, yy += step){
            if (yy < 0 || yy >= h) continue;
            if (labels[yy*w+ix] === id){ pts.push({ x: ix, y: yy - step * 0.5 }); break; }
          }
        } else {
          if (iy < 0 || iy >= h) continue;
          var stepx = dir === 'left' ? 1 : -1, xx = ix - stepx * reach;
          for (k = 0; k < reach * 2; k++, xx += stepx){
            if (xx < 0 || xx >= w) continue;
            if (labels[iy*w+xx] === id){ pts.push({ x: xx - stepx * 0.5, y: iy }); break; }
          }
        }
      }
      return pts;
    }
    var Lt = robustFit(collect(tl, tr, 'top')),
        Lr = robustFit(collect(tr, br, 'right')),
        Lb = robustFit(collect(bl, br, 'bottom')),
        Ll = robustFit(collect(tl, bl, 'left'));
    var quad = rough;
    if (Lt && Lr && Lb && Ll){
      var q = [intersect(Ll, Lt), intersect(Lt, Lr), intersect(Lr, Lb), intersect(Lb, Ll)];
      var diag = Math.sqrt(w*w + h*h), okFit = true;
      for (p = 0; p < 4; p++){
        if (!q[p] || d2(q[p], rough[p]) > diag * 0.06){ okFit = false; break; }
      }
      if (okFit) quad = q;
    }

    var qa = polyArea(quad);
    if (qa < 1) return null;
    // Area with holes filled (paint or sketch marks inside the canvas aren't gaps):
    // count each row from its first to its last pixel of this region.
    var filled = 0;
    for (y = c.minY; y <= c.maxY; y++){
      var first = -1, last = -1;
      for (x = c.minX; x <= c.maxX; x++) if (labels[y*w+x] === id){ if (first < 0) first = x; last = x; }
      if (first >= 0) filled += last - first + 1;
    }
    var rect = Math.min(1, filled / qa) * Math.min(1, qa / filled);
    var angles = [cornerAngle(quad[3], quad[0], quad[1]), cornerAngle(quad[0], quad[1], quad[2]),
                  cornerAngle(quad[1], quad[2], quad[3]), cornerAngle(quad[2], quad[3], quad[0])];
    var angOk = angles.every(function(a){ return a > 55 && a < 125; });
    return { quad: quad, rect: rect, angOk: angOk, area: filled };
  }

  // Snap each side of a rough quad onto the real edge in the image.
  // Along each side we look for brightness steps just inside/outside the rough
  // edge, then find the straight lines those steps make. Stray marks inside the
  // canvas don't form a straight line along the edge, so they're ignored.
  // Next to a canvas edge there's often another straight line (a shadow, the
  // easel ledge), so we take the outermost line that still has canvas surface
  // on its inner side.
  // Returns {quad, support} or null when the sides don't line up with real edges.
  function refineQuad(quad, img, w, h){
    function sample(x, y){
      if (x < 0 || y < 0 || x > w - 1.001 || y > h - 1.001) return -1;
      var x0 = x | 0, y0 = y | 0, fx = x - x0, fy = y - y0, p = y0*w + x0;
      return (img[p] * (1-fx) + img[p+1] * fx) * (1-fy) + (img[p+w] * (1-fx) + img[p+w+1] * fx) * fy;
    }
    var S0 = -20, S1 = 4, lines = [], support = 1, clean = 0;
    for (var sIdx = 0; sIdx < 4; sIdx++){
      var a = quad[sIdx], b = quad[(sIdx + 1) % 4];
      var len = d2(a, b); if (len < 10) return null;
      var ux = (b.x - a.x) / len, uy = (b.y - a.y) / len, nx = uy, ny = -ux; // outward for clockwise quad
      var n = Math.max(10, Math.round(len / 2)), rows = [], i, k, s;
      // how far across the canvas this side looks, to sample its surface well inside
      var oa = quad[(sIdx + 2) % 4], ob = quad[(sIdx + 3) % 4];
      var across = d2({ x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 }, { x: (oa.x + ob.x) / 2, y: (oa.y + ob.y) / 2 });
      var sDeep = -Math.max(16, 0.15 * across);
      for (i = 0; i <= n; i++){
        var t = 0.12 + 0.76 * i / n;
        var px = a.x + (b.x - a.x) * t, py = a.y + (b.y - a.y) * t;
        var prof = [];
        for (s = S0; s <= S1; s += 0.5) prof.push(sample(px + nx * s, py + ny * s));
        if (prof.indexOf(-1) >= 0){ rows.push(null); continue; }
        var der = [];
        for (k = 1; k < prof.length - 1; k++) der.push(Math.abs(prof[k+1] - prof[k-1]));
        var peaks = [];
        for (k = 1; k < der.length - 1; k++){
          if (der[k] >= 4 && der[k] >= der[k-1] && der[k] > der[k+1]){
            var den = der[k-1] - 2*der[k] + der[k+1], off = den ? 0.5 * (der[k-1] - der[k+1]) / den : 0;
            peaks.push(S0 + (k + 1 + Math.max(-0.5, Math.min(0.5, off))) * 0.5);
          }
        }
        var dA = sample(px + nx * sDeep, py + ny * sDeep), dB = sample(px + nx * (sDeep - 1.5), py + ny * (sDeep - 1.5));
        rows.push({ t: t - 0.5, px: px, py: py, peaks: peaks, deep: (dA >= 0 && dB >= 0) ? (dA + dB) / 2 : prof[0] });
      }
      // vote for straight lines s = s0 + m * t across the side
      var valid = rows.filter(function(r){ return r; }).length;
      if (valid < n * 0.6) return null;
      var found = [];
      for (var m = -3; m <= 3.001; m += 0.25){
        for (var s0 = S0 + 0.5; s0 <= S1 - 0.5; s0 += 0.25){
          var cnt = 0;
          for (i = 0; i < rows.length; i++){
            var r = rows[i]; if (!r) continue;
            var want = s0 + m * r.t;
            for (k = 0; k < r.peaks.length; k++) if (Math.abs(r.peaks[k] - want) < 0.75){ cnt++; break; }
          }
          if (cnt >= valid * 0.5) found.push({ s0: s0, m: m, cnt: cnt });
        }
      }
      if (!found.length) return null;
      // collapse near-duplicate lines, keeping the best-supported of each cluster
      found.sort(function(p, q){ return q.cnt - p.cnt; });
      var distinct = [];
      found.forEach(function(L){
        if (!distinct.some(function(D){ return Math.abs(D.s0 - L.s0) < 1.25; })) distinct.push(L);
      });
      distinct.sort(function(p, q){ return p.s0 - q.s0; });
      // what the canvas surface looks like near this side, from well inside it
      var deeps = rows.filter(function(r){ return r; }).map(function(r){ return r.deep; }).sort(function(p, q){ return p - q; });
      var surface = deeps[deeps.length >> 1];
      var pick = null;
      for (k = distinct.length - 1; k >= 0 && !pick; k--){
        var Lk = distinct[k], ins = [], outs = [];
        rows.forEach(function(r){
          if (!r) return;
          var sl = Lk.s0 + Lk.m * r.t;
          var vi = sample(r.px + nx * (sl - 1.5), r.py + ny * (sl - 1.5)), vo = sample(r.px + nx * (sl + 1.5), r.py + ny * (sl + 1.5));
          if (vi >= 0 && vo >= 0){ ins.push(vi); outs.push(vo); }
        });
        if (!ins.length) continue;
        ins.sort(function(p, q){ return p - q; }); outs.sort(function(p, q){ return p - q; });
        var vin = ins[ins.length >> 1], vout = outs[outs.length >> 1];
        if (Math.abs(vin - surface) <= Math.max(10, 0.35 * Math.abs(vin - vout))) pick = Lk;
      }
      if (pick) clean++;
      else {
        // fall back: innermost line in the outer edge band
        var outer = distinct[distinct.length - 1]; pick = outer;
        for (k = distinct.length - 1; k >= 0 && distinct[k].s0 >= outer.s0 - 4.5; k--) pick = distinct[k];
      }
      // gather the steps on the chosen line and fit it precisely
      var pts = [];
      rows.forEach(function(r){
        if (!r) return;
        var want = pick.s0 + pick.m * r.t, bestK = -1, bestD = 0.75;
        for (var j = 0; j < r.peaks.length; j++){ var dd = Math.abs(r.peaks[j] - want); if (dd < bestD){ bestD = dd; bestK = j; } }
        if (bestK >= 0) pts.push({ x: r.px + nx * r.peaks[bestK], y: r.py + ny * r.peaks[bestK] });
      });
      support = Math.min(support, pick.cnt / Math.max(1, valid));
      var L = fitLine(pts);
      if (!L) return null;
      lines.push(L);
    }
    var q = [intersect(lines[3], lines[0]), intersect(lines[0], lines[1]),
             intersect(lines[1], lines[2]), intersect(lines[2], lines[3])];
    for (var c = 0; c < 4; c++) if (!q[c] || d2(q[c], quad[c]) > 20) return null;
    return { quad: q, support: support, clean: clean };
  }

  // Find the canvas in a grayscale frame. Returns {quad:[tl,tr,br,bl], score} in
  // the frame's pixel coordinates, or null if nothing canvas-like is visible.
  function detectCanvas(gray, w, h){
    var img = boxBlur(boxBlur(gray, w, h), w, h);
    var N = w * h, minArea = N * 0.04, cx = w / 2, cy = h / 2;
    var halfDiag = Math.sqrt(cx*cx + cy*cy);
    var candidates = [];

    function consider(mask, source){
      var cc = components(mask, w, h, minArea);
      cc.comps.forEach(function(c){
        if (c.area > N * 0.9) return;
        var touches = (c.minX <= 1 ? 1 : 0) + (c.maxX >= w - 2 ? 1 : 0) + (c.minY <= 1 ? 1 : 0) + (c.maxY >= h - 2 ? 1 : 0);
        if (touches > 0) return; // the whole canvas has to be on screen
        var r = quadFromComponent(cc.labels, w, h, c);
        if (!r || !r.angOk || r.rect < 0.9) return;
        var ref = refineQuad(r.quad, img, w, h);
        if (!ref) return;
        var quad = ref.quad;
        var qc = { x: (quad[0].x + quad[1].x + quad[2].x + quad[3].x) / 4,
                   y: (quad[0].y + quad[1].y + quad[2].y + quad[3].y) / 4 };
        var off = d2(qc, { x: cx, y: cy }) / halfDiag;
        var score = r.rect * r.rect * ref.support * Math.sqrt(r.area / N) * (1 - 0.6 * off) * (ref.clean === 4 ? 1 : 0.5);
        candidates.push({ quad: quad, score: score, rect: r.rect, support: ref.support, areaFrac: r.area / N, source: source });
      });
    }

    // 1) Brightness split: a canvas is usually lighter (or darker) than what's around it.
    var o = otsu(img), i, x, y;
    if (o.spread > 3){
      var bright = new Uint8Array(N), dark = new Uint8Array(N);
      for (i = 0; i < N; i++){ if (img[i] > o.t) bright[i] = 1; else dark[i] = 1; }
      consider(erode(dilate(bright, w, h), w, h), 'bright');
      consider(erode(dilate(dark, w, h), w, h), 'dark');
    }
    // 1b) The same split, judged from the middle of the picture only. A dark
    // floor or easel in view can make the whole-picture split put the wall
    // and the canvas on the same side; the middle is mostly canvas and wall.
    var x0 = Math.round(w * 0.2), x1 = Math.round(w * 0.8), y0 = Math.round(h * 0.2), y1 = Math.round(h * 0.8);
    var mid = new Uint8Array((x1 - x0) * (y1 - y0)), mk = 0;
    for (y = y0; y < y1; y++) for (x = x0; x < x1; x++) mid[mk++] = img[y * w + x];
    var oc = otsu(mid);
    if (oc.spread > 3 && Math.abs(oc.t - o.t) > 8){
      var brightC = new Uint8Array(N), darkC = new Uint8Array(N);
      for (i = 0; i < N; i++){ if (img[i] > oc.t) brightC[i] = 1; else darkC[i] = 1; }
      consider(erode(dilate(brightC, w, h), w, h), 'bright');
      consider(erode(dilate(darkC, w, h), w, h), 'dark');
    }

    // 2) Edge outline: for a canvas close in color to the wall. Flood the
    // background in from the screen edges without crossing any visible edge;
    // whatever the flood can't reach is an object. Marks inside the canvas
    // don't matter because the flood never gets in there.
    var grad = new Float32Array(N), gvals = [];
    for (y = 1; y < h - 1; y++){
      for (x = 1; x < w - 1; x++){
        var p = y*w + x;
        var gx = img[p-w+1] + 2*img[p+1] + img[p+w+1] - img[p-w-1] - 2*img[p-1] - img[p+w-1];
        var gy = img[p+w-1] + 2*img[p+w] + img[p+w+1] - img[p-w-1] - 2*img[p-w] - img[p-w+1];
        grad[p] = Math.abs(gx) + Math.abs(gy);
        if ((x + y) % 3 === 0) gvals.push(grad[p]);
      }
    }
    gvals.sort(function(a, b){ return a - b; });
    var p50 = gvals[Math.floor(gvals.length * 0.5)] || 0;
    var p98 = gvals[Math.floor(gvals.length * 0.98)] || 0;
    var gt = Math.max(8, p50 * 4, Math.min(60, p98 * 0.22));
    var open = new Uint8Array(N);
    for (i = 0; i < N; i++) open[i] = grad[i] < gt ? 1 : 0;
    open = erode(open, w, h); // thicken edges so small gaps don't leak
    var outside = new Uint8Array(N), stack = new Int32Array(N), sp = 0;
    function push(q){ if (open[q] && !outside[q]){ outside[q] = 1; stack[sp++] = q; } }
    for (x = 0; x < w; x++){ push(x); push((h-1)*w + x); }
    for (y = 0; y < h; y++){ push(y*w); push(y*w + w - 1); }
    while (sp){
      var q = stack[--sp], qx = q % w;
      if (qx > 0) push(q-1);
      if (qx < w-1) push(q+1);
      if (q >= w) push(q-w);
      if (q < N - w) push(q+w);
    }
    var inside = new Uint8Array(N);
    for (i = 0; i < N; i++) inside[i] = outside[i] ? 0 : 1;
    // drop thin lines (easel legs, cords) that touch the canvas
    inside = erode(erode(inside, w, h), w, h);
    inside = dilate(dilate(inside, w, h), w, h);
    consider(inside, 'edges');

    if (!candidates.length) return null;
    candidates.sort(function(a, b){ return b.score - a.score; });
    return candidates[0];
  }

  // ---------- coaching ----------
  // quad: [tl,tr,br,bl] in screen pixels; W,H: screen (stage) size.
  var TOL = 0.02, ROLL_TOL = 0.75;
  function measure(quad){
    var top = d2(quad[0], quad[1]), right = d2(quad[1], quad[2]),
        bottom = d2(quad[3], quad[2]), left = d2(quad[0], quad[3]);
    var a1 = Math.atan2(quad[1].y - quad[0].y, quad[1].x - quad[0].x) * 180 / Math.PI;
    var a2 = Math.atan2(quad[2].y - quad[3].y, quad[2].x - quad[3].x) * 180 / Math.PI;
    return { tb: top / bottom - 1, lr: left / right - 1, roll: (a1 + a2) / 2,
             avgW: (top + bottom) / 2, avgH: (left + right) / 2 };
  }
  function coach(quad, W, H){
    var m = measure(quad);
    var ok = { tilt: Math.abs(m.tb) <= TOL, turn: Math.abs(m.lr) <= TOL, level: Math.abs(m.roll) <= ROLL_TOL };
    var margin = Infinity;
    quad.forEach(function(p){ margin = Math.min(margin, p.x / W, (W - p.x) / W, p.y / H, (H - p.y) / H); });
    var areaFrac = polyArea(quad) / (W * H);
    var res = { metrics: m, ok: ok, areaFrac: areaFrac, square: ok.tilt && ok.turn && ok.level };

    if (margin < 0.015){ res.key = 'back'; res.text = 'Move back so the whole canvas fits on screen'; return res; }
    if (areaFrac < 0.1){ res.key = 'closer'; res.text = 'Move closer so the canvas fills more of the screen'; return res; }
    if (res.square){ res.key = 'square'; res.text = 'Square to the canvas'; return res; }

    var errs = [
      { k: 'tilt', e: Math.abs(m.tb) / TOL },
      { k: 'turn', e: Math.abs(m.lr) / TOL },
      { k: 'level', e: Math.abs(m.roll) / ROLL_TOL }
    ].filter(function(x){ return x.e > 1; }).sort(function(a, b){ return b.e - a.e; });
    var top = errs[0], little = top.e < 3 ? ' a little' : '';
    if (top.k === 'tilt'){
      if (m.tb < 0){ res.key = 'tiltIn'; res.text = 'Tilt the top of your phone' + little + ' toward the canvas'; }
      else { res.key = 'tiltOut'; res.text = 'Tilt the top of your phone' + little + ' back toward you'; }
    } else if (top.k === 'turn'){
      // the far side of the canvas looks shorter; turn the phone away from it
      if (m.lr < 0){ res.key = 'turnRight'; res.text = 'Turn your phone' + little + ' to the right'; }
      else { res.key = 'turnLeft'; res.text = 'Turn your phone' + little + ' to the left'; }
    } else {
      if (m.roll > 0){ res.key = 'rotCW'; res.text = 'Rotate your phone' + little + ' clockwise'; }
      else { res.key = 'rotCCW'; res.text = 'Rotate your phone' + little + ' counterclockwise'; }
    }
    return res;
  }

  // ---------- bump compare ----------
  // How far each corner moved, as a fraction of the frame diagonal.
  function quadShift(a, b, diag){
    var s = [];
    for (var i = 0; i < 4; i++) s.push(d2(a[i], b[i]) / diag);
    return { min: Math.min.apply(null, s), max: Math.max.apply(null, s),
             mean: (s[0] + s[1] + s[2] + s[3]) / 4 };
  }

  return {
    solveHomography: solveHomography, applyH: applyH, invertH: invertH, cssMatrix3d: cssMatrix3d,
    detectCanvas: detectCanvas, coach: coach, measure: measure, quadShift: quadShift,
    polyArea: polyArea
  };
})();
if (typeof module !== 'undefined' && module.exports) module.exports = VellumCore;

// Loaded as a background worker: find the canvas in each frame sent over.
if (typeof window === 'undefined' && typeof self !== 'undefined' && typeof importScripts === 'function') {
  self.onmessage = function(e){
    var d = e.data, r = null;
    try { r = VellumCore.detectCanvas(new Uint8Array(d.buf), d.w, d.h); } catch (err) {}
    self.postMessage({ id: d.id, r: r });
  };
}
