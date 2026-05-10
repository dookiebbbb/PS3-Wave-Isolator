/*
 * PS3 XMB Wave Screensaver
 * Homebrew application for PS3 CFW (PSL1GHT SDK)
 *
 * Reads lines.qrc directly from /dev_rebug/resource/vsh/resource/qgl/
 * Parses MNU parameters, simulates wave physics on SPU,
 * renders fullscreen via RSX. Press any button to exit.
 *
 * Build requires: PSL1GHT SDK, ps3toolchain
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <zlib.h>

/* PSL1GHT headers */
#include <rsx/rsx.h>
#include <rsx/gcm_sys.h>
#include <sysutil/sysutil_sysparam.h>
#include <io/pad.h>
#include <sys/process.h>
#include <sys/memory.h>
#include <ppu_intrinsics.h>

/* ── Constants ─────────────────────────────────────────────────────── */

#define LINES_QRC_PATH  "/dev_rebug/resource/vsh/resource/qgl/lines.qrc"
#define LINES_QRC_ALT   "/dev_flash/vsh/resource/qgl/lines.qrc"

#define SCREEN_W        1280
#define SCREEN_H        720
#define DISP_BUF_COUNT  2

/* Displacement texture dimensions (matches PS3 spline.elf output) */
#define DISP_W          256
#define DISP_H          64

/* Wave mesh grid resolution */
#define MESH_COLS       256
#define MESH_ROWS       64

#define PI              3.14159265358979f
#define TWO_PI          6.28318530717959f

/* ── MNU Parameter block ───────────────────────────────────────────── */

typedef struct {
    /* Wave physics */
    float damping;
    float length;
    float tension;
    float thinness;
    float spacing;
    float perturbation;
    float timestep;
    float brightness;
    float fresnel;
    float falloff;
    /* Camera */
    float fovy;
    float angRot;
    float posZ;
    /* FFD */
    float ffdParam1;
    float ffdScale2Y;
    float ffdScale2Z;
    /* Colour slots */
    float slot1[3];
    float slot2[3];
    float slot3[3];
    float slot4[3];
    /* HDR */
    float exposure;
    float whiteLevel;
    float glareLevel;
    float glareThresh;
    float gaussR;
    float gaussG;
    float gaussB;
    float glareSumPow;
    /* DOF */
    int   blur;
} WaveParams;

/* ── Vertex structure ──────────────────────────────────────────────── */

typedef struct {
    float x, y, z;
    float u, v;
    float r, g, b, a;
} Vertex;

/* ── RSX state ─────────────────────────────────────────────────────── */

static gcmContextData  *gfx_ctx       = NULL;
static void            *host_mem      = NULL;
static uint32_t         display_buf[DISP_BUF_COUNT];
static uint32_t         display_offset[DISP_BUF_COUNT];
static uint32_t         depth_offset;
static int              cur_buf       = 0;

/* ── Wave simulation state ─────────────────────────────────────────── */

typedef struct {
    float phase[8];
    float speed[8];
    float amp[8];
    float freq[8];
    float phase2[8];
    float speed2[8];
    float time;
} SplineState;

static SplineState  g_spline;
static float        g_disp[DISP_W * DISP_H];   /* displacement texture data */
static WaveParams   g_params;

/* ── Pseudo-random number generator (Mulberry32) ───────────────────── */

static uint32_t rng_state = 0xDEADBEEF;

static float rng_next(void) {
    rng_state += 0x6D2B79F5;
    uint32_t t = (rng_state ^ (rng_state >> 15)) * (1 | rng_state);
    t = (t + ((t ^ (t >> 7)) * (61 | t))) ^ t;
    return (float)((t ^ (t >> 14)) >> 0) / 4294967296.0f;
}

/* ── QRC / MNU parser ──────────────────────────────────────────────── */

/*
 * Extract a float value from MNU text block.
 * Looks for "KEY:float:VALUE" pattern.
 * Returns def if not found.
 */
static float mnu_get_float(const char *text, size_t text_len,
                            const char *key, float def)
{
    char pattern[128];
    snprintf(pattern, sizeof(pattern), "%s:float:", key);
    size_t klen = strlen(pattern);

    for (size_t i = 0; i + klen < text_len; i++) {
        if (memcmp(text + i, pattern, klen) == 0) {
            return (float)atof(text + i + klen);
        }
    }
    return def;
}

static int mnu_get_int(const char *text, size_t text_len,
                        const char *key, int def)
{
    char pattern[128];
    snprintf(pattern, sizeof(pattern), "%s:int:", key);
    size_t klen = strlen(pattern);
    for (size_t i = 0; i + klen < text_len; i++) {
        if (memcmp(text + i, pattern, klen) == 0) {
            return atoi(text + i + klen);
        }
    }
    return def;
}

/*
 * Get first occurrence of "N RED/GREEN/BLUE" slots
 * where N is 1..4
 */
static float mnu_get_slot(const char *text, size_t text_len,
                            int slot, const char *channel, float def)
{
    char key[32];
    snprintf(key, sizeof(key), "%d %s", slot, channel);
    return mnu_get_float(text, text_len, key, def);
}

/*
 * Parse all relevant parameters from decompressed MNU text.
 */
static void parse_mnu(const char *text, size_t len, WaveParams *p)
{
    p->damping      = mnu_get_float(text, len, "DAMPING",       0.0001f);
    p->length       = mnu_get_float(text, len, "LENGTH",        0.306001f);
    p->tension      = mnu_get_float(text, len, "TENSION",       0.25f);
    p->thinness     = mnu_get_float(text, len, "THINNESS",      1.0f);
    p->spacing      = mnu_get_float(text, len, "SPACING",       407.658f);
    p->perturbation = mnu_get_float(text, len, "PERTURBATION",  0.0998587f);
    p->timestep     = mnu_get_float(text, len, "TIMESTEP",      4.0f);
    p->brightness   = mnu_get_float(text, len, "BRIGHTNESS",    0.701917f);
    p->fresnel      = mnu_get_float(text, len, "FRESNEL",       0.638971f);
    p->falloff      = mnu_get_float(text, len, "FALLOFF",       0.630683f);
    p->fovy         = mnu_get_float(text, len, "FOVY",          71.846f);
    p->angRot       = mnu_get_float(text, len, "ANG ROT",       18.1208f);
    p->posZ         = mnu_get_float(text, len, "POS Z",        -6.4f);
    p->ffdParam1    = mnu_get_float(text, len, "FFD PARAM 1",  -1.33509f);
    p->ffdScale2Y   = mnu_get_float(text, len, "FFD SCALE2 Y",  1.27579f);
    p->ffdScale2Z   = mnu_get_float(text, len, "FFD SCALE2 Z",  2.88782f);

    /* Colour slots */
    p->slot1[0] = mnu_get_slot(text, len, 1, "RED",   1.0f);
    p->slot1[1] = mnu_get_slot(text, len, 1, "GREEN", 1.0f);
    p->slot1[2] = mnu_get_slot(text, len, 1, "BLUE",  1.0f);
    p->slot2[0] = mnu_get_slot(text, len, 2, "RED",   1.0f);
    p->slot2[1] = mnu_get_slot(text, len, 2, "GREEN", 1.0f);
    p->slot2[2] = mnu_get_slot(text, len, 2, "BLUE",  1.0f);
    p->slot3[0] = mnu_get_slot(text, len, 3, "RED",   0.847347f);
    p->slot3[1] = mnu_get_slot(text, len, 3, "GREEN", 0.846155f);
    p->slot3[2] = mnu_get_slot(text, len, 3, "BLUE",  0.846448f);
    p->slot4[0] = mnu_get_slot(text, len, 4, "RED",   0.92487f);
    p->slot4[1] = mnu_get_slot(text, len, 4, "GREEN", 0.922603f);
    p->slot4[2] = mnu_get_slot(text, len, 4, "BLUE",  0.922933f);

    /* HDR */
    p->exposure    = mnu_get_float(text, len, "EXPOSURE",       1.05f);
    p->whiteLevel  = mnu_get_float(text, len, "WHITE LEVEL",    0.899181f);
    p->glareLevel  = mnu_get_float(text, len, "GLARE LEVEL",    1.10245f);
    p->glareThresh = mnu_get_float(text, len, "GLARE THRESH",   0.738857f);
    p->gaussR      = mnu_get_float(text, len, "GAUSSIAN RAD R", 1.23888f);
    p->gaussG      = mnu_get_float(text, len, "GAUSSIAN RAD G", 1.43176f);
    p->gaussB      = mnu_get_float(text, len, "GAUSSIAN RAD B", 1.55787f);
    p->glareSumPow = mnu_get_float(text, len, "GLARE SUM POW",  0.557478f);
    p->blur        = mnu_get_int  (text, len, "BLUR",           0);
}

/*
 * Load and decompress lines.qrc.
 * Tries the Rebug path first, falls back to dev_flash.
 * Returns allocated decompressed buffer (caller must free),
 * or NULL on failure.
 */
static char *load_qrc(size_t *out_len)
{
    FILE *f = fopen(LINES_QRC_PATH, "rb");
    if (!f) f = fopen(LINES_QRC_ALT, "rb");
    if (!f) {
        printf("[wave] Could not open lines.qrc\n");
        return NULL;
    }

    /* Get file size */
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t *raw = (uint8_t *)malloc(fsize);
    if (!raw) { fclose(f); return NULL; }
    fread(raw, 1, fsize, f);
    fclose(f);

    /* Verify QRCC magic */
    if (memcmp(raw, "QRCC", 4) != 0) {
        printf("[wave] Bad magic, not a QRCC file\n");
        free(raw);
        return NULL;
    }

    /* Uncompressed size is at bytes 4-7 (big-endian) */
    uint32_t uncomp_size = ((uint32_t)raw[4] << 24) |
                           ((uint32_t)raw[5] << 16) |
                           ((uint32_t)raw[6] <<  8) |
                            (uint32_t)raw[7];

    char *decompressed = (char *)malloc(uncomp_size + 1);
    if (!decompressed) { free(raw); return NULL; }

    uLongf dest_len = uncomp_size;
    int ret = uncompress((Bytef *)decompressed, &dest_len,
                         raw + 8, (uLong)(fsize - 8));
    free(raw);

    if (ret != Z_OK) {
        printf("[wave] zlib decompress failed: %d\n", ret);
        free(decompressed);
        return NULL;
    }

    decompressed[dest_len] = '\0';
    *out_len = (size_t)dest_len;
    printf("[wave] Loaded lines.qrc, decompressed to %u bytes\n",
           (unsigned)dest_len);
    return decompressed;
}

/* ── Spline displacement simulation ───────────────────────────────── */

/*
 * Initialise the 8 spline kernel ribbon states.
 * These correspond to the 8 loop iterations in FUN_000045c0 (spline.elf).
 */
static void spline_init(SplineState *s, const WaveParams *p)
{
    (void)p;
    s->time = 0.0f;
    for (int i = 0; i < 8; i++) {
        s->phase[i]  = rng_next() * TWO_PI;
        s->speed[i]  = 0.08f + i * 0.035f;
        s->amp[i]    = 0.5f + rng_next() * 0.5f;
        s->freq[i]   = 0.5f + i * 0.22f;
        s->phase2[i] = rng_next() * TWO_PI;
        s->speed2[i] = 0.05f + i * 0.02f;
    }
}

/*
 * Generate displacement texture for this frame.
 * Output: g_disp[DISP_H][DISP_W] — Y displacement per (row, col).
 *
 * Each row = one wave line.
 * Each col = X sample along the line.
 *
 * This approximates the SPU spline kernel output using layered
 * sinusoids driven by the MNU physics parameters.
 */
static void spline_update(SplineState *s, const WaveParams *p, float dt)
{
    const float sim_speed    = p->timestep * 0.25f;
    s->time += dt * sim_speed;

    const float t            = s->time;
    const float tension_sc   = 0.3f + p->tension * 1.4f;
    const float damp_amp     = 1.0f / (1.0f + p->damping * 80.0f);
    const float len_sc       = 0.306001f / p->length;
    const float pert_sc      = p->perturbation * 12.0f;

    for (int row = 0; row < DISP_H; row++) {
        float line_t     = (float)row / (float)(DISP_H - 1);
        float line_phase = line_t * PI * 2.8f;

        for (int col = 0; col < DISP_W; col++) {
            float x = (float)col / (float)(DISP_W - 1);
            float y = 0.0f;

            /* 8 kernel ribbon iterations */
            for (int k = 0; k < 8; k++) {
                float w1 = sinf(
                    x * s->freq[k] * len_sc * tension_sc * TWO_PI
                    + t * s->speed[k]
                    + line_phase * (0.3f + k * 0.08f)
                    + s->phase[k]
                );
                float w2 = sinf(
                    x * s->freq[k] * len_sc * tension_sc * PI * 1.618f
                    + t * s->speed2[k]
                    + line_phase * (0.15f + k * 0.04f)
                    + s->phase2[k]
                );
                y += (w1 + w2 * 0.4f) * s->amp[k];
            }

            /* Perturbation noise */
            if (pert_sc > 0.0f) {
                y += sinf(x * 17.3f + t * 0.7f + line_phase * 0.5f)
                     * pert_sc * 0.01f;
                y += sinf(x * 31.7f + t * 0.4f + line_phase * 0.3f)
                     * pert_sc * 0.005f;
            }

            g_disp[row * DISP_W + col] = (y / 8.0f) * damp_amp;
        }
    }
}

/* ── Reinhard tonemapping ──────────────────────────────────────────── */

static inline float tonemap(float v, float exposure, float white)
{
    v *= exposure;
    return v * (1.0f + v / (white * white)) / (1.0f + v);
}

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── Immediate-mode software renderer ─────────────────────────────── */

/*
 * Since we can't use GLSL shaders from userland homebrew without
 * significant additional toolchain setup, we implement a software
 * rasterizer that writes directly to the RSX framebuffer.
 *
 * For each frame:
 *   1. Fill background gradient (slot1 top → slot2 bottom)
 *   2. Draw wave lines (DISP_H lines, each sampled from displacement)
 *   3. Bloom pass (simple box blur on bright pixels)
 *   4. Composite with tonemapping
 *
 * The framebuffer is 32-bit ARGB at SCREEN_W x SCREEN_H.
 */

static uint32_t *g_framebuf[DISP_BUF_COUNT] = {NULL, NULL};
static uint32_t  g_backbuf[SCREEN_W * SCREEN_H];  /* working buffer */
static uint32_t  g_bloombuf[SCREEN_W * SCREEN_H]; /* bloom buffer */

static inline uint32_t pack_argb(float r, float g, float b, float a)
{
    uint8_t ri = (uint8_t)(clampf(r, 0.0f, 1.0f) * 255.0f);
    uint8_t gi = (uint8_t)(clampf(g, 0.0f, 1.0f) * 255.0f);
    uint8_t bi = (uint8_t)(clampf(b, 0.0f, 1.0f) * 255.0f);
    uint8_t ai = (uint8_t)(clampf(a, 0.0f, 1.0f) * 255.0f);
    return ((uint32_t)ai << 24) | ((uint32_t)ri << 16) |
           ((uint32_t)gi <<  8) |  (uint32_t)bi;
}

static inline void unpack_argb(uint32_t c,
                                 float *r, float *g, float *b, float *a)
{
    *a = ((c >> 24) & 0xFF) / 255.0f;
    *r = ((c >> 16) & 0xFF) / 255.0f;
    *g = ((c >>  8) & 0xFF) / 255.0f;
    *b = ( c        & 0xFF) / 255.0f;
}

/* Draw a single pixel with alpha blending */
static inline void plot(int x, int y, float r, float g, float b, float a)
{
    if (x < 0 || x >= SCREEN_W || y < 0 || y >= SCREEN_H) return;
    uint32_t *dst = &g_backbuf[y * SCREEN_W + x];
    float dr, dg, db, da;
    unpack_argb(*dst, &dr, &dg, &db, &da);
    *dst = pack_argb(dr + r * a, dg + g * a, db + b * a, 1.0f);
}

/* Draw anti-aliased line between two points */
static void draw_line_aa(float x0, float y0, float x1, float y1,
                          float r, float g, float b, float alpha,
                          float width)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.001f) return;

    int steps = (int)(len * 1.5f) + 1;
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        float px = x0 + dx * t;
        float py = y0 + dy * t;

        /* Width — plot neighbouring pixels for thickness */
        int w = (int)(width * 0.5f) + 1;
        for (int wy = -w; wy <= w; wy++) {
            float dist = fabsf((float)wy);
            float fade = clampf(1.0f - dist / (width * 0.6f), 0.0f, 1.0f);
            plot((int)px, (int)(py + wy), r, g, b, alpha * fade);
        }
    }
}

/* ── Main render function ──────────────────────────────────────────── */

static void render_frame(const WaveParams *p)
{
    const float e  = p->exposure;
    const float wl = p->whiteLevel;
    const float ang_rad = (p->angRot - 90.0f) * PI / 180.0f;
    const float cos_a   = cosf(ang_rad);
    const float sin_a   = sinf(ang_rad);
    const float fov_eff = fabsf(p->fovy) < 1.0f ? 71.846f : fabsf(p->fovy);
    const float fov_sc  = 71.846f / fov_eff;

    /* ── 1. Background gradient ──────────────────────────────────── */
    for (int y = 0; y < SCREEN_H; y++) {
        float yt = 1.0f - (float)y / (float)SCREEN_H;  /* 0=bottom, 1=top */
        float r = tonemap(p->slot1[0] * 0.58f * yt
                        + p->slot2[0] * 0.28f * (1.0f-yt), e, wl);
        float g = tonemap(p->slot1[1] * 0.58f * yt
                        + p->slot2[1] * 0.28f * (1.0f-yt), e, wl);
        float b = tonemap(p->slot1[2] * 0.58f * yt
                        + p->slot2[2] * 0.28f * (1.0f-yt), e, wl);
        uint32_t pixel = pack_argb(r, g, b, 1.0f);
        for (int x = 0; x < SCREEN_W; x++)
            g_backbuf[y * SCREEN_W + x] = pixel;
    }

    /* ── 2. Vignette on background ───────────────────────────────── */
    for (int y = 0; y < SCREEN_H; y++) {
        for (int x = 0; x < SCREEN_W; x++) {
            float nx = (float)x / SCREEN_W - 0.5f;
            float ny = (float)y / SCREEN_H - 0.5f;
            float vig = clampf(1.0f - (nx*nx + ny*ny) * 1.35f, 0.0f, 1.0f);
            float r, g, b, a;
            unpack_argb(g_backbuf[y*SCREEN_W+x], &r, &g, &b, &a);
            g_backbuf[y*SCREEN_W+x] = pack_argb(r*vig, g*vig, b*vig, 1.0f);
        }
    }

    /* ── 3. Draw wave lines ──────────────────────────────────────── */
    /* Spacing in screen pixels */
    float spacing_px = (p->spacing / 407.658f) * SCREEN_H * 0.17f;
    int   num_lines  = (int)((float)SCREEN_H * 1.7f / fmaxf(spacing_px, 3.0f));
    num_lines = num_lines < 4 ? 4 : (num_lines > DISP_H ? DISP_H : num_lines);
    float total_h    = (float)(num_lines - 1) * spacing_px;
    float line_w     = 0.4f + p->thinness * 0.32f;

    /* Displacement amplitude scale */
    float disp_sc = (407.658f / fmaxf(p->spacing, 10.0f)) * 0.28f
                  * (1.0f + p->perturbation * 0.3f);

    for (int li = 0; li < num_lines; li++) {
        float line_t  = (float)li / (float)(num_lines - 1);

        /* Colour: blend slot3/slot4 via fresnel */
        float blend = powf(sinf(line_t * PI), fmaxf(p->fresnel, 0.05f));
        float cr = p->slot3[0] * (1.0f-blend) + p->slot4[0] * blend;
        float cg = p->slot3[1] * (1.0f-blend) + p->slot4[1] * blend;
        float cb = p->slot3[2] * (1.0f-blend) + p->slot4[2] * blend;
        cr = tonemap(cr * p->brightness, e, wl);
        cg = tonemap(cg * p->brightness, e, wl);
        cb = tonemap(cb * p->brightness, e, wl);

        /* Alpha: edge lines slightly transparent */
        float stack_fade = 1.0f - fabsf(line_t - 0.5f) * 2.0f * p->falloff * 0.6f;
        float alpha = clampf(stack_fade, 0.08f, 0.88f);

        /* Use displacement row proportional to line index */
        int disp_row = (int)(line_t * (float)(DISP_H - 1));

        /* Base Y position for this line */
        float base_y_world = (line_t - 0.5f) * 2.0f * p->ffdScale2Y * 0.4f;

        /* Draw line as connected segments */
        float prev_sx = 0.0f, prev_sy = 0.0f;
        int   first   = 1;

        for (int col = 0; col < DISP_W; col++) {
            float u = (float)col / (float)(DISP_W - 1);

            /* World X */
            float wx = (u - 0.5f) * 2.2f;

            /* FFD arch + displacement */
            float arch = p->ffdParam1 * (u-0.5f)*(u-0.5f)
                       * p->ffdScale2Y * 0.35f;
            float disp  = g_disp[disp_row * DISP_W + col] * disp_sc;
            float wy    = base_y_world + arch + disp;

            /* Apply ANG ROT */
            float rx = wx * cos_a - wy * sin_a;
            float ry = wx * sin_a + wy * cos_a;

            /* Project to screen (aspect-corrected) */
            float aspect = (float)SCREEN_W / (float)SCREEN_H;
            float sx = (rx / aspect * fov_sc + 1.0f) * 0.5f * SCREEN_W;
            float sy = (-ry * fov_sc + 1.0f) * 0.5f * SCREEN_H;

            if (!first) {
                /* Edge X fade */
                float ef = clampf(u / 0.06f, 0.0f, 1.0f)
                         * clampf((1.0f-u) / 0.06f, 0.0f, 1.0f);
                /* Fresnel brightening at displacement peaks */
                float disp_bright = 1.0f + fabsf(disp) * p->fresnel * 1.2f;
                draw_line_aa(prev_sx, prev_sy, sx, sy,
                             cr * disp_bright, cg * disp_bright,
                             cb * disp_bright,
                             alpha * ef, line_w);
            }
            prev_sx = sx;
            prev_sy = sy;
            first   = 0;
        }
    }

    /* ── 4. Simple bloom pass ───────────────────────────────────── */
    if (p->glareLevel > 0.05f && p->glareThresh < 1.0f) {
        memset(g_bloombuf, 0, sizeof(g_bloombuf));
        float avg_gauss = (p->gaussR + p->gaussG + p->gaussB) / 3.0f;
        int blur_r = (int)(avg_gauss * (SCREEN_H / 720.0f) * 8.5f);
        blur_r = blur_r < 1 ? 1 : (blur_r > 24 ? 24 : blur_r);

        /* Threshold extract into bloom buffer */
        for (int i = 0; i < SCREEN_W * SCREEN_H; i++) {
            float r, g, b, a;
            unpack_argb(g_backbuf[i], &r, &g, &b, &a);
            float lum = 0.2126f*r + 0.7152f*g + 0.0722f*b;
            if (lum > p->glareThresh) {
                float br = fmaxf(r - p->glareThresh, 0.0f) * p->glareLevel;
                float bg = fmaxf(g - p->glareThresh, 0.0f) * p->glareLevel;
                float bb = fmaxf(b - p->glareThresh, 0.0f) * p->glareLevel;
                float pw = p->glareSumPow;
                g_bloombuf[i] = pack_argb(
                    powf(br, pw), powf(bg, pw), powf(bb, pw), 1.0f);
            }
        }

        /* Horizontal blur */
        uint32_t tmp[SCREEN_W * SCREEN_H];
        for (int y = 0; y < SCREEN_H; y++) {
            for (int x = 0; x < SCREEN_W; x++) {
                float r=0,g=0,b=0,a,w=0;
                for (int k = -blur_r; k <= blur_r; k++) {
                    int sx = x + k;
                    if (sx < 0 || sx >= SCREEN_W) continue;
                    float fr,fg,fb;
                    unpack_argb(g_bloombuf[y*SCREEN_W+sx],&fr,&fg,&fb,&a);
                    float wk = 1.0f / (1.0f + (float)(k*k));
                    r+=fr*wk; g+=fg*wk; b+=fb*wk; w+=wk;
                }
                tmp[y*SCREEN_W+x] = pack_argb(r/w,g/w,b/w,1.0f);
            }
        }
        /* Vertical blur */
        for (int y = 0; y < SCREEN_H; y++) {
            for (int x = 0; x < SCREEN_W; x++) {
                float r=0,g=0,b=0,a,w=0;
                for (int k = -blur_r; k <= blur_r; k++) {
                    int sy = y + k;
                    if (sy < 0 || sy >= SCREEN_H) continue;
                    float fr,fg,fb;
                    unpack_argb(tmp[sy*SCREEN_W+x],&fr,&fg,&fb,&a);
                    float wk = 1.0f / (1.0f + (float)(k*k));
                    r+=fr*wk; g+=fg*wk; b+=fb*wk; w+=wk;
                }
                g_bloombuf[y*SCREEN_W+x] = pack_argb(r/w,g/w,b/w,1.0f);
            }
        }

        /* Screen blend bloom onto backbuf */
        float bloom_str = fminf(2.5f, p->glareLevel * 0.82f);
        for (int i = 0; i < SCREEN_W * SCREEN_H; i++) {
            float sr,sg,sb,sa, br,bg,bb,ba;
            unpack_argb(g_backbuf[i],  &sr,&sg,&sb,&sa);
            unpack_argb(g_bloombuf[i], &br,&bg,&bb,&ba);
            g_backbuf[i] = pack_argb(
                1.0f-(1.0f-sr)*(1.0f-br*bloom_str),
                1.0f-(1.0f-sg)*(1.0f-bg*bloom_str),
                1.0f-(1.0f-sb)*(1.0f-bb*bloom_str),
                1.0f);
        }
    }

    /* ── 5. Copy to RSX framebuffer ─────────────────────────────── */
    memcpy(g_framebuf[cur_buf], g_backbuf, SCREEN_W * SCREEN_H * 4);
}

/* ── RSX initialisation ────────────────────────────────────────────── */

#define HOST_SIZE   (32 * 1024 * 1024)
#define CB_SIZE     (0x100000)

static int rsx_init(void)
{
    host_mem = memalign(0x100000, HOST_SIZE);
    if (!host_mem) return -1;

    gfx_ctx = (gcmContextData *)memalign(0x100, sizeof(gcmContextData));
    if (!gfx_ctx) return -1;

    rsxInit(gfx_ctx, CB_SIZE, HOST_SIZE, host_mem);

    gcmSetFlipMode(GCM_FLIP_VSYNC);

    /* Allocate display buffers */
    for (int i = 0; i < DISP_BUF_COUNT; i++) {
        g_framebuf[i] = (uint32_t *)rsxMemalign(64, SCREEN_W*SCREEN_H*4);
        if (!g_framebuf[i]) return -1;
        rsxAddressToOffset(g_framebuf[i], &display_offset[i]);
        gcmSetDisplayBuffer(i, display_offset[i], SCREEN_W*4, SCREEN_W, SCREEN_H);
    }

    return 0;
}

static void rsx_flip(void)
{
    gcmFlip(gfx_ctx, cur_buf);
    rsxFlushBuffer(gfx_ctx);
    gcmSetWaitFlip(gfx_ctx);
    cur_buf = (cur_buf + 1) % DISP_BUF_COUNT;
}

/* ── Entry point ───────────────────────────────────────────────────── */

SYS_PROCESS_PARAM(1001, 0x10000)

int main(void)
{
    printf("[wave] PS3 Wave Screensaver starting\n");

    /* Initialise RSX */
    if (rsx_init() != 0) {
        printf("[wave] RSX init failed\n");
        return 1;
    }

    /* Initialise pad */
    ioPadInit(7);

    /* Load and parse lines.qrc */
    size_t qrc_len = 0;
    char  *qrc_text = load_qrc(&qrc_len);
    if (!qrc_text) {
        printf("[wave] Failed to load lines.qrc, using defaults\n");
        /* Use Sony defaults so we still show something */
        memset(&g_params, 0, sizeof(g_params));
        g_params.damping      = 0.0001f;
        g_params.length       = 0.306001f;
        g_params.tension      = 0.25f;
        g_params.thinness     = 1.0f;
        g_params.spacing      = 407.658f;
        g_params.perturbation = 0.0998587f;
        g_params.timestep     = 4.0f;
        g_params.brightness   = 0.701917f;
        g_params.fresnel      = 0.638971f;
        g_params.falloff      = 0.630683f;
        g_params.fovy         = 71.846f;
        g_params.angRot       = 18.1208f;
        g_params.ffdParam1    = -1.33509f;
        g_params.ffdScale2Y   = 1.27579f;
        g_params.slot1[0]=g_params.slot1[1]=g_params.slot1[2]=1.0f;
        g_params.slot2[0]=g_params.slot2[1]=g_params.slot2[2]=1.0f;
        g_params.slot3[0]=0.847f; g_params.slot3[1]=0.846f; g_params.slot3[2]=0.846f;
        g_params.slot4[0]=0.924f; g_params.slot4[1]=0.922f; g_params.slot4[2]=0.922f;
        g_params.exposure    = 1.05f;
        g_params.whiteLevel  = 0.899181f;
        g_params.glareLevel  = 1.10245f;
        g_params.glareThresh = 0.738857f;
        g_params.gaussR      = 1.23888f;
        g_params.gaussG      = 1.43176f;
        g_params.gaussB      = 1.55787f;
        g_params.glareSumPow = 0.557478f;
    } else {
        parse_mnu(qrc_text, qrc_len, &g_params);
        free(qrc_text);
        printf("[wave] Parsed wave params OK\n");
        printf("[wave]   angRot=%.2f fovy=%.3f spacing=%.1f\n",
               g_params.angRot, g_params.fovy, g_params.spacing);
        printf("[wave]   slot3=[%.3f,%.3f,%.3f]\n",
               g_params.slot3[0], g_params.slot3[1], g_params.slot3[2]);
    }

    /* Initialise spline simulation */
    spline_init(&g_spline, &g_params);

    /* Frame timing */
    uint64_t last_time = __mftb();
    const uint64_t TB_FREQ = 79800000ULL; /* PS3 timebase ~79.8MHz */

    printf("[wave] Entering render loop\n");

    /* ── Main loop ──────────────────────────────────────────────── */
    while (1) {
        /* Delta time */
        uint64_t now = __mftb();
        float dt = (float)(now - last_time) / (float)TB_FREQ;
        if (dt > 0.05f) dt = 0.05f;
        last_time = now;

        /* Check for any button press to exit */
        padInfo pad_info;
        ioPadGetInfo(&pad_info);
        for (int i = 0; i < 7; i++) {
            if (!pad_info.status[i]) continue;
            padData pad_data;
            ioPadGetData(i, &pad_data);
            /* Any digital button pressed */
            if (pad_data.BTN_CROSS   || pad_data.BTN_CIRCLE  ||
                pad_data.BTN_SQUARE  || pad_data.BTN_TRIANGLE ||
                pad_data.BTN_START   || pad_data.BTN_SELECT   ||
                pad_data.BTN_R1      || pad_data.BTN_L1       ||
                pad_data.BTN_PS) {
                goto exit_loop;
            }
        }

        /* Update spline displacement */
        spline_update(&g_spline, &g_params, dt);

        /* Render frame */
        render_frame(&g_params);

        /* Flip to display */
        rsx_flip();
    }

exit_loop:
    printf("[wave] Exiting\n");
    ioPadEnd();

    return 0;
}
