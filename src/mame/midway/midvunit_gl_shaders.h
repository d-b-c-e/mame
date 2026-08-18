// GENERATED from cruisn-poc/gpu/renderer.py - do not edit by hand.
// These shaders are verified 100.0000% bit-exact against MAME's
// software rasterizer (see cruisn-poc/results/RESULTS.md).

static const char *MVGL_VS = R"GLSL(
#version 430
uniform vec2 uCanvas;      // coarse canvas size (W, H)
in vec2 in_corner;         // bbox corner, coarse pixel space
in vec2 in_v0; in vec2 in_v1; in vec2 in_v2; in vec2 in_v3;
in vec4 in_uv01;           // u0,v0,u1,v1
in vec4 in_uv23;           // u2,v2,u3,v3
in uvec4 in_meta;          // pixdata, mode, dither, texbase
flat out vec2 v0; flat out vec2 v1; flat out vec2 v2; flat out vec2 v3;
flat out vec4 uv01; flat out vec4 uv23;
flat out uvec4 meta;
void main() {
    v0 = in_v0; v1 = in_v1; v2 = in_v2; v3 = in_v3;
    uv01 = in_uv01; uv23 = in_uv23; meta = in_meta;
    // y-down pixel space -> NDC (y flipped)
    gl_Position = vec4(in_corner.x / uCanvas.x * 2.0 - 1.0,
                       1.0 - in_corner.y / uCanvas.y * 2.0, 0.0, 1.0);
}
)GLSL";

static const char *MVGL_FS = R"GLSL(
#version 430
uniform int  uScale;       // 1 = exact mode, >1 = quality mode
uniform vec2 uCanvas;      // coarse canvas size
uniform int  uClipRight;   // coarse cliprect right (W-1)
uniform usampler2D texram; // 4096-wide R8UI, 8 MB of texture RAM
uniform int  texMask;      // byte-size mask (size-1)
flat in vec2 v0; flat in vec2 v1; flat in vec2 v2; flat in vec2 v3;
flat in vec4 uv01; flat in vec4 uv23;
flat in uvec4 meta;
out uint outIndex;

int round_coord(float f) {           // poly.h: floor, +1 iff frac > 0.5
    float ip = floor(f);
    return int(ip) + (((f - ip) > 0.5) ? 1 : 0);
}
int c_int32(float f) {               // C float->int32: trunc, OOR -> INT_MIN
    if (!(abs(f) < 2147483648.0)) return -2147483648;
    return int(f);
}
uint fetch_texel(int idx) {
    idx &= texMask;
    return texelFetch(texram, ivec2(idx & 4095, idx >> 12), 0).r;
}

void main() {
    vec2 vx[4] = vec2[4](v0, v1, v2, v3);
    float fx = gl_FragCoord.x;
    float fy = uCanvas.y * float(uScale) - gl_FragCoord.y;   // y-down
    float cx = fx / float(uScale);
    float cy = fy / float(uScale);
    int px = int(floor(cx));
    int py = int(floor(cy));
    // exact mode evaluates at MAME's scanline centre; quality mode uses the
    // fine fragment's own continuous coordinate
    precise float fully = (uScale == 1) ? float(py) + 0.5 : cy;

    // ---- min/max Y vertices (poly.h render_polygon) ----
    int minv = 0, maxv = 0;
    for (int i = 1; i < 4; i++) {
        if (vx[i].y < vx[minv].y) minv = i;
        else if (vx[i].y > vx[maxv].y) maxv = i;
    }
    if (round_coord(vx[maxv].y) - round_coord(vx[minv].y) <= 0) discard;
    float maxvy = vx[maxv].y;

    // poly.h renders scanlines [round(miny), round(maxy)) only - the expanded
    // bounding box generates fragments beyond that, where extrapolated edges
    // still yield plausible x-extents. Without this cut, later quads steal
    // their neighbours' shared-edge rows.
    if (uScale == 1) {
        if (py < round_coord(vx[minv].y) || py >= round_coord(maxvy)) discard;
    } else {
        if (cy < vx[minv].y || cy >= maxvy) discard;
    }

    // ---- forward / backward edge lists (<=3 each) ----
    // e[k] = (v1x, v1y, v2y, dxdy), p[k] = (u1, v1_p, dudy, dvdy)
    precise vec4 fe[3]; precise vec4 fp[3]; int fn = 0;
    precise vec4 be[3]; precise vec4 bp[3]; int bn = 0;
    vec4 uvs[4] = vec4[4](vec4(uv01.xy, 0, 0), vec4(uv01.zw, 0, 0),
                          vec4(uv23.xy, 0, 0), vec4(uv23.zw, 0, 0));
    for (int curv = minv; curv != maxv; curv = (curv + 1) & 3) {
        int nxt = (curv + 1) & 3;
        if (vx[nxt].y != vx[curv].y) {
            precise float ooy = 1.0 / (vx[nxt].y - vx[curv].y);
            fe[fn] = vec4(vx[curv].x, vx[curv].y, vx[nxt].y,
                          (vx[nxt].x - vx[curv].x) * ooy);
            fp[fn] = vec4(uvs[curv].xy,
                          (uvs[nxt].x - uvs[curv].x) * ooy,
                          (uvs[nxt].y - uvs[curv].y) * ooy);
            fn++;
        }
    }
    for (int curv = minv; curv != maxv; curv = (curv - 1) & 3) {
        int nxt = (curv - 1) & 3;
        if (vx[nxt].y != vx[curv].y) {
            precise float ooy = 1.0 / (vx[nxt].y - vx[curv].y);
            be[bn] = vec4(vx[curv].x, vx[curv].y, vx[nxt].y,
                          (vx[nxt].x - vx[curv].x) * ooy);
            bp[bn] = vec4(uvs[curv].xy,
                          (uvs[nxt].x - uvs[curv].x) * ooy,
                          (uvs[nxt].y - uvs[curv].y) * ooy);
            bn++;
        }
    }
    if (fn == 0 || bn == 0) discard;

    // ---- left/right decision (poly.h:1194) ----
    bool sharedFirst = (fe[0].x == be[0].x) && (fe[0].y == be[0].y);
    bool fwd_left = (sharedFirst && fe[0].w < be[0].w)
                 || (!sharedFirst && fe[0].x < be[0].x);

    // ---- advance to the edge pair spanning this scanline ----
    int li = 0, ri = 0;
    vec4 le, lp, re, rp;
    if (fwd_left) {
        while (fully > fe[li].z && fully < maxvy && li + 1 < fn) li++;
        while (fully > be[ri].z && fully < maxvy && ri + 1 < bn) ri++;
        le = fe[li]; lp = fp[li]; re = be[ri]; rp = bp[ri];
    } else {
        while (fully > be[li].z && fully < maxvy && li + 1 < bn) li++;
        while (fully > fe[ri].z && fully < maxvy && ri + 1 < fn) ri++;
        le = be[li]; lp = bp[li]; re = fe[ri]; rp = fp[ri];
    }

    precise float startx = le.x + (fully - le.y) * le.w;
    precise float stopx  = re.x + (fully - re.y) * re.w;
    int istartx = round_coord(startx);
    int istopx  = round_coord(stopx);
    if (istartx > istopx) { int t = istartx; istartx = istopx; istopx = t; }

    // ---- params at this scanline (poly.h:1250) ----
    precise float ldy = fully - le.y;
    precise float rdy = fully - re.y;
    precise float oox = 1.0 / (stopx - startx);
    precise float lu = lp.x + ldy * lp.z;
    precise float lv = lp.y + ldy * lp.w;
    precise float dudx = (rp.x + rdy * rp.z - lu) * oox;
    precise float dvdx = (rp.y + rdy * rp.w - lv) * oox;
    precise float su = lu + (float(istartx) + 0.5 - startx) * dudx;
    precise float sv = lv + (float(istartx) + 0.5 - startx) * dvdx;

    // ---- left/right clip with param adjust ----
    if (istartx < 0) {
        su += float(-istartx) * dudx;
        sv += float(-istartx) * dvdx;
        istartx = 0;
    }
    if (istopx > uClipRight) istopx = uClipRight + 1;
    if (istartx >= istopx) discard;

    // ---- coverage ----
    if (uScale == 1) {
        if (px < istartx || px >= istopx) discard;
    } else {
        // continuous edges at fine resolution; clip window still applies
        float lo = max(min(startx, stopx), 0.0);
        float hi = min(max(startx, stopx), float(uClipRight + 1));
        if (cx < lo || cx >= hi) discard;
    }

    uint pixdata = meta.x, mode = meta.y, dither = meta.z;
    if (dither == 1u && ((px ^ py) & 1) != 0) discard;   // coarse-space mask

    if (mode == 0u) { outIndex = pixdata & 0xffffu; return; }

    int ui, vi;
    if (uScale == 1) {   // MAME's integer DDA, analytically
        ui = c_int32(su) + (px - istartx) * c_int32(dudx);
        vi = c_int32(sv) + (px - istartx) * c_int32(dvdx);
    } else {             // sub-pixel float interpolation
        ui = c_int32(lu + (cx - startx) * dudx);
        vi = c_int32(lv + (cx - startx) * dvdx);
    }
    uint texel = fetch_texel(int(meta.w) + ((vi >> 8) & 0xff00) + (ui >> 16));
    if (mode == 1u)      outIndex = (pixdata + texel) & 0xffffu;
    else if (mode == 2u) { if (texel == 0u) discard;
                           outIndex = (pixdata + texel) & 0xffffu; }
    else                 { if (texel == 0u) discard;
                           outIndex = pixdata & 0xffffu; }
}
)GLSL";

static const char *MVGL_PAL_VS = R"GLSL(
#version 430
out vec2 uv;
void main() {  // full-screen triangle
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    uv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

static const char *MVGL_PAL_FS = R"GLSL(
#version 430
uniform usampler2D idxTex;
uniform usampler2D palTex;   // 256x128 R32UI - 32768 palette words
uniform int uCrop;           // fine pixels to crop from each side (2D screens)
in vec2 uv;
out vec4 color;
void main() {
    ivec2 sz = textureSize(idxTex, 0);
    ivec2 p = ivec2(float(uCrop) + uv.x * float(sz.x - 2 * uCrop),
                    uv.y * float(sz.y));
    uint pen = texelFetch(idxTex, p, 0).r & 0x7fffu;
    uint w = texelFetch(palTex, ivec2(pen & 255u, pen >> 8), 0).r;
    uint r = (w >> 10) & 31u, g = (w >> 5) & 31u, b = w & 31u;
    color = vec4(float((r << 3) | (r >> 2)) / 255.0,
                 float((g << 3) | (g >> 2)) / 255.0,
                 float((b << 3) | (b >> 2)) / 255.0, 1.0);
}
)GLSL";
