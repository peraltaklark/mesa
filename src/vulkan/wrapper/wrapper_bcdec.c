#include "wrapper_bcdec.h"
#include "wrapper_log.h"
#include "wrapper_util.h"

#define XXH_STATIC_LINKING_ONLY
#define XXH_IMPLEMENTATION
#include "util/xxhash.h"

#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <dlfcn.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define BCDEC_BC4BC5_PRECISE
#define BCDEC_IMPLEMENTATION

#include "bcdec.h"
#include "wrapper_astc.h"

/* Environment switches, read once and shared by every upload thread.
 *   WRAPPER_BCN_ASTC=0       decode to RGBA instead of transcoding to ASTC
 *   WRAPPER_ASTC_BLOCK=8x8   ASTC 8x8 (2bpp, 4x smaller/softer) instead of 4x4
 *   WRAPPER_MARK_BCN=1       fill emulated textures with a per-format colour
 *   WRAPPER_NO_BCN_THREAD=1  decode on the calling thread
 *   WRAPPER_USE_BCN_CACHE=0  disable the transcode disk cache
 *   WRAPPER_BCN_UPLOAD=1     leave .src sidecars for the server
 *   WRAPPER_CACHE_PATH=dir   cache directory; by default <imagefs>/usr/cache of
 *                            the application (id) this process belongs to */
struct bcn_config {
   int astc;          /* default on: BCn -> ASTC (Mali-native, stays compressed) */
   int block8;
   int mark;
   int no_thread;
   int use_cache;     /* default on */
   int upload;
   char *cache_dir;   /* never NULL after init (unless out of memory) */
};

static struct bcn_config bcn_cfg;
static pthread_once_t bcn_cfg_once = PTHREAD_ONCE_INIT;

static int
bcn_env_int(const char *name, int def)
{
   const char *e = getenv(name);
   return e ? atoi(e) : def;
}

static void
bcn_cfg_init(void)
{
   const char *e;

   bcn_cfg.astc = bcn_env_int("WRAPPER_BCN_ASTC", 1);
   e = getenv("WRAPPER_ASTC_BLOCK");
   bcn_cfg.block8 = e && strstr(e, "8x8");
   bcn_cfg.mark = bcn_env_int("WRAPPER_MARK_BCN", 0) != 0;
   bcn_cfg.no_thread = bcn_env_int("WRAPPER_NO_BCN_THREAD", 0) != 0;
   bcn_cfg.use_cache = bcn_env_int("WRAPPER_USE_BCN_CACHE", 1);
   e = getenv("WRAPPER_BCN_UPLOAD");
   bcn_cfg.upload = e && !strcmp(e, "1");

   e = getenv("WRAPPER_CACHE_PATH");
   bcn_cfg.cache_dir = (e && e[0]) ? strdup(e) : wrapper_imagefs_path("cache");
   if (bcn_cfg.cache_dir) {
      WRAPPER_LOG(bcn, "BCn cache dir %s (application id %s)", bcn_cfg.cache_dir,
                  wrapper_app_id() ? wrapper_app_id() : "unknown");
      if ((bcn_cfg.use_cache || bcn_cfg.upload) &&
          wrapper_mkdir_p(bcn_cfg.cache_dir, 0700) != 0)
         WRAPPER_LOG(error, "Cannot create BCn cache dir %s", bcn_cfg.cache_dir);
   }
}

static const struct bcn_config *
bcn_config(void)
{
   pthread_once(&bcn_cfg_once, bcn_cfg_init);
   return &bcn_cfg;
}

static int
astc_enabled(void)
{
   return bcn_config()->astc;
}

/* ASTC block footprint: 0 = 4x4 (8bpp, crisp), 1 = 8x8 (2bpp, 4x smaller/softer).
 * Selected by WRAPPER_ASTC_BLOCK ("4x4" default, "8x8"). */
static int
astc_block8(void)
{
   return bcn_config()->block8;
}

/* WRAPPER_BCN_POLICY=bc1=6x6,bc2=4x4,...,bc6h=4x4,maxdim=1024
 * Per-format ASTC block (4x4, 6x6, 8x8) and a base-level cap. A key left out
 * keeps the stock behaviour (WRAPPER_ASTC_BLOCK, no cap). 6x6 has an encoder
 * for bc1 only; bc6h takes 4x4 only (ASTC HDR, when the device has it). */
enum {
   BCN_POL_BC1, BCN_POL_BC2, BCN_POL_BC3, BCN_POL_BC4, BCN_POL_BC5,
   BCN_POL_BC7, BCN_POL_BC6H, BCN_POL_COUNT
};

static const char *const bcn_pol_keys[BCN_POL_COUNT] = {
   "bc1", "bc2", "bc3", "bc4", "bc5", "bc7", "bc6h"
};

static struct {
   int block[BCN_POL_COUNT]; /* 0 = stock, else 4/6/8 */
   uint32_t maxdim;          /* 0 = no cap */
   char desc[160];
} bcn_pol;

static pthread_once_t bcn_pol_once = PTHREAD_ONCE_INIT;
static int bcn_astc_hdr;

static void
bcn_policy_parse(void)
{
   const char *e = getenv("WRAPPER_BCN_POLICY");
   if (!e) {
      snprintf(bcn_pol.desc, sizeof(bcn_pol.desc), "unset");
      return;
   }
   char *copy = strdup(e), *save = NULL;
   for (char *it = copy ? strtok_r(copy, ",", &save) : NULL; it;
        it = strtok_r(NULL, ",", &save)) {
      while (*it == ' ')
         it++;
      char *eq = strchr(it, '=');
      if (!eq) {
         WRAPPER_LOG(info, "BCn policy: ignoring '%s'", it);
         continue;
      }
      *eq = 0;
      char *val = eq + 1;
      while (*val == ' ')
         val++;
      for (char *t = val + strlen(val); t > val && t[-1] == ' '; t--)
         t[-1] = 0;
      for (char *t = eq; t > it && t[-1] == ' '; t--)
         t[-1] = 0;
      if (!strcmp(it, "maxdim")) {
         char *end = NULL;
         unsigned long v = strtoul(val, &end, 10);
         if (end == val || *end || v > 65536)
            WRAPPER_LOG(info, "BCn policy: ignoring maxdim=%s", val);
         else
            bcn_pol.maxdim = (uint32_t)v;
         continue;
      }
      int idx = -1;
      for (int i = 0; i < BCN_POL_COUNT; i++)
         if (!strcmp(it, bcn_pol_keys[i]))
            idx = i;
      int b = !strcmp(val, "4x4") ? 4 : !strcmp(val, "6x6") ? 6 :
              !strcmp(val, "8x8") ? 8 : 0;
      if (idx < 0 || !b) {
         WRAPPER_LOG(info, "BCn policy: ignoring %s=%s", it, val);
         continue;
      }
      if ((b == 6 && idx != BCN_POL_BC1) ||
          ((idx == BCN_POL_BC6H || idx == BCN_POL_BC4 || idx == BCN_POL_BC5) && b != 4)) {
         WRAPPER_LOG(info, "BCn policy: no %dx%d encoder for %s, stock", b, b, it);
         continue;
      }
      bcn_pol.block[idx] = b;
   }
   free(copy);
   int n = 0;
   for (int i = 0; i < BCN_POL_COUNT; i++) {
      int b = bcn_pol.block[i];
      n += snprintf(bcn_pol.desc + n, sizeof(bcn_pol.desc) - n,
                    b ? "%s=%dx%d " : "%s=stock ", bcn_pol_keys[i], b, b);
   }
   snprintf(bcn_pol.desc + n, sizeof(bcn_pol.desc) - n, "maxdim=%u", bcn_pol.maxdim);
   WRAPPER_LOG(info, "BCn policy: %s", bcn_pol.desc);
}

const char *
bcn_policy_desc(void)
{
   pthread_once(&bcn_pol_once, bcn_policy_parse);
   return bcn_pol.desc;
}

static int
bcn_policy_block(int idx)
{
   pthread_once(&bcn_pol_once, bcn_policy_parse);
   return bcn_pol.block[idx];
}

int
bcn_policy_bc6h_hdr(void)
{
   return astc_enabled() && bcn_policy_block(BCN_POL_BC6H) == 4;
}

/* Set per device: BC6H goes to ASTC 4x4 HDR only when the device enabled it. */
void
bcn_set_astc_hdr(int on)
{
   __atomic_store_n(&bcn_astc_hdr, on, __ATOMIC_RELAXED);
}

/* ASTC block edge for a BCn format: the policy entry, else WRAPPER_ASTC_BLOCK. */
static int
bcn_astc_edge(int idx)
{
   if (idx == BCN_POL_BC4 || idx == BCN_POL_BC5)
      return 4; /* one/two-channel maps are too lossy at 8x8 */
   int b = bcn_policy_block(idx);
   return b ? b : (astc_block8() ? 8 : 4);
}

int
is_astc_4x4(VkFormat format)
{
   return format == VK_FORMAT_ASTC_4x4_UNORM_BLOCK ||
          format == VK_FORMAT_ASTC_4x4_SRGB_BLOCK;
}

int
is_astc_6x6(VkFormat format)
{
   return format == VK_FORMAT_ASTC_6x6_UNORM_BLOCK ||
          format == VK_FORMAT_ASTC_6x6_SRGB_BLOCK;
}

int
is_astc_8x8(VkFormat format)
{
   return format == VK_FORMAT_ASTC_8x8_UNORM_BLOCK ||
          format == VK_FORMAT_ASTC_8x8_SRGB_BLOCK;
}

int
is_astc(VkFormat format)
{
   return is_astc_4x4(format) || is_astc_6x6(format) || is_astc_8x8(format);
}

int
is_astc_hdr_4x4(VkFormat format)
{
   return format == VK_FORMAT_ASTC_4x4_SFLOAT_BLOCK;
}

static int
bcn_is_bc1(VkFormat f)
{
   return f == VK_FORMAT_BC1_RGB_UNORM_BLOCK || f == VK_FORMAT_BC1_RGB_SRGB_BLOCK ||
          f == VK_FORMAT_BC1_RGBA_UNORM_BLOCK || f == VK_FORMAT_BC1_RGBA_SRGB_BLOCK;
}

static int
bcn_is_bc1_rgb(VkFormat f)
{
   return f == VK_FORMAT_BC1_RGB_UNORM_BLOCK || f == VK_FORMAT_BC1_RGB_SRGB_BLOCK;
}

/* Whether the transcode must carry alpha. BC1_RGB has none; BC1_RGBA is
 * punch-through and each block is refined below (bc1_block_has_alpha). */
static int
bcn_has_alpha(VkFormat bcn_format)
{
   switch (bcn_format) {
   case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
   case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
   case VK_FORMAT_BC2_UNORM_BLOCK:
   case VK_FORMAT_BC2_SRGB_BLOCK:
   case VK_FORMAT_BC3_UNORM_BLOCK:
   case VK_FORMAT_BC3_SRGB_BLOCK:
   case VK_FORMAT_BC7_UNORM_BLOCK:
   case VK_FORMAT_BC7_SRGB_BLOCK:
      return 1;
   default:
      return 0;
   }
}

/* A BC1 block has transparent texels only in 3-colour mode (c0 <= c1) and only
 * if some 2-bit index is 3. Everything else is opaque and takes the finer RGB
 * encoder. */
static int
bc1_block_has_alpha(const unsigned char *b)
{
   unsigned c0 = b[0] | (b[1] << 8), c1 = b[2] | (b[3] << 8);
   if (c0 > c1)
      return 0;
   uint32_t idx = (uint32_t)b[4] | ((uint32_t)b[5] << 8) |
                  ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
   return ((idx & (idx >> 1)) & 0x55555555u) != 0;
}

struct decompression_params {
   int block_x;       /* destination grid width, in blocks of the target format */
   int block_x_src;   /* source row stride, in BC blocks */
   int block_y_count;
   int block_y_start;
   size_t stride;     /* destination row pitch in bytes (decode-to-texels only) */
   int texel_size;
   int astc;
   int astc8;
   int bc_bx;      /* BC grid width in blocks (astc8/astc6 bounds) */
   int bc_by;      /* BC grid height in blocks (astc8/astc6 bounds) */
   int astc6;
   int astc_by;    /* ASTC 6x6 grid height (astc6 bounds) */
   int w;
   int h;
   int has_alpha;
   VkFormat format;
   const char *src;
   char *dst;
};

static int
get_block_size(VkFormat format) 
{
    switch(format) {
       case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
       case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
       case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
       case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
       case VK_FORMAT_BC4_UNORM_BLOCK:
       case VK_FORMAT_BC4_SNORM_BLOCK:
          return 8;
       default:
          return 16;
    }
}

/* BC4/BC5 into an RGBA scratch with the channel layout the shaders expect
 * without a view swizzle: BC4 replicates its value over RGB (A=255), BC5 keeps
 * R/G from the block with B=0, A=255. */
static void
bcn_decode_rg_rgba(VkFormat format, const char *src, unsigned char *dst, int stride)
{
   unsigned char tmp[32];

   if (format == VK_FORMAT_BC5_UNORM_BLOCK) {
      bcdec_bc5(src, tmp, 8, 0);
      for (int y = 0; y < 4; y++) {
         for (int x = 0; x < 4; x++) {
            unsigned char *p = dst + y * stride + x * 4;
            p[0] = tmp[y * 8 + x * 2];
            p[1] = tmp[y * 8 + x * 2 + 1];
            p[2] = 0;
            p[3] = 255;
         }
      }
   } else {
      bcdec_bc4(src, tmp, 4, 0);
      for (int y = 0; y < 4; y++) {
         for (int x = 0; x < 4; x++) {
            unsigned char *p = dst + y * stride + x * 4;
            p[0] = p[1] = p[2] = tmp[y * 4 + x];
            p[3] = 255;
         }
      }
   }
}

/* One BC block to a 4x4 RGBA8 scratch (row pitch in bytes) for the ASTC encoders. */
static void
bcn_decode_rgba_block(VkFormat format, const char *src, unsigned char *dst, int pitch)
{
   switch (format) {
   case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
   case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
   case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
   case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
      bcdec_bc1(src, dst, pitch);
      break;
   case VK_FORMAT_BC2_SRGB_BLOCK:
   case VK_FORMAT_BC2_UNORM_BLOCK:
      bcdec_bc2(src, dst, pitch);
      break;
   case VK_FORMAT_BC3_UNORM_BLOCK:
   case VK_FORMAT_BC3_SRGB_BLOCK:
      bcdec_bc3(src, dst, pitch);
      break;
   case VK_FORMAT_BC7_SRGB_BLOCK:
   case VK_FORMAT_BC7_UNORM_BLOCK:
      bcdec_bc7(src, dst, pitch);
      break;
   case VK_FORMAT_BC4_UNORM_BLOCK:
   case VK_FORMAT_BC5_UNORM_BLOCK:
      bcn_decode_rg_rgba(format, src, dst, pitch);
      break;
   default:
      for (int y = 0; y < 4; y++)
         memset(dst + (size_t)y * pitch, 0, 16);
      break;
   }
}

/* One BC block to the plain decoded layout, 4 rows of `stride` bytes. Always
 * writes a full 4x4 block, so callers hand it a scratch for edge blocks. */
static void
bcn_decode_native_block(VkFormat format, const char *src, char *dst,
                        size_t stride, int texel_size)
{
   int pitch = (int)stride;

   switch (format) {
   case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
   case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
   case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
   case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
      bcdec_bc1(src, dst, pitch);
      if (bcn_is_bc1_rgb(format)) {
         /* bcdec writes alpha 0 for the 3-colour index; RGB has no alpha. */
         for (int y = 0; y < 4; y++)
            for (int x = 0; x < 4; x++)
               dst[(size_t)y * stride + x * 4 + 3] = (char)0xFF;
      }
      break;
   case VK_FORMAT_BC2_SRGB_BLOCK:
   case VK_FORMAT_BC2_UNORM_BLOCK:
      bcdec_bc2(src, dst, pitch);
      break;
   case VK_FORMAT_BC3_UNORM_BLOCK:
   case VK_FORMAT_BC3_SRGB_BLOCK:
      bcdec_bc3(src, dst, pitch);
      break;
   case VK_FORMAT_BC4_UNORM_BLOCK:
   case VK_FORMAT_BC4_SNORM_BLOCK:
      bcdec_bc4(src, dst, pitch, format == VK_FORMAT_BC4_SNORM_BLOCK);
      break;
   case VK_FORMAT_BC5_SNORM_BLOCK:
   case VK_FORMAT_BC5_UNORM_BLOCK:
      bcdec_bc5(src, dst, pitch, format == VK_FORMAT_BC5_SNORM_BLOCK);
      break;
   case VK_FORMAT_BC6H_SFLOAT_BLOCK:
   case VK_FORMAT_BC6H_UFLOAT_BLOCK:
      bcdec_bc6h_half(src, dst, (pitch / texel_size) * 3,
                      format == VK_FORMAT_BC6H_SFLOAT_BLOCK);
      break;
   case VK_FORMAT_BC7_SRGB_BLOCK:
   case VK_FORMAT_BC7_UNORM_BLOCK:
      bcdec_bc7(src, dst, pitch);
      break;
   default:
      break;
   }
}

static void
decompression_routine(struct decompression_params *params)
{
   char *dst_base = params->dst;
   int block_size = get_block_size(params->format);
   int bc1 = bcn_is_bc1(params->format);

   /* 6x6 ASTC (BC1 only): a 3x3 group of BC blocks (12x12 texels) covers 2x2
    * ASTC blocks. BC blocks past the grid are clamped to the last one and texels
    * past the mip edge replicate the edge texel. block_y_start/count are group
    * rows. */
   if (params->astc6) {
      int gx_count = (params->block_x + 1) / 2;
      for (int gy = params->block_y_start;
           gy < params->block_y_start + params->block_y_count; gy++) {
         for (int gx = 0; gx < gx_count; gx++) {
            unsigned char scratch[12 * 48];
            for (int dy = 0; dy < 3; dy++) {
               int bcy = 3 * gy + dy;
               if (bcy >= params->bc_by)
                  bcy = params->bc_by - 1;
               for (int dx = 0; dx < 3; dx++) {
                  int bcx = 3 * gx + dx;
                  if (bcx >= params->bc_bx)
                     bcx = params->bc_bx - 1;
                  const char *sblk = params->src +
                     ((size_t)bcy * params->block_x_src + bcx) * block_size;
                  bcdec_bc1(sblk, scratch + (dy * 4) * 48 + (dx * 4) * 4, 48);
               }
            }
            for (int sy = 0; sy < 2; sy++) {
               int BY = 2 * gy + sy;
               if (BY >= params->astc_by)
                  continue;
               for (int sx = 0; sx < 2; sx++) {
                  int BX = 2 * gx + sx;
                  if (BX >= params->block_x)
                     continue;
                  uint8_t texels[144];
                  for (int y = 0; y < 6; y++) {
                     int py = BY * 6 + y;
                     if (py >= params->h)
                        py = params->h - 1;
                     py -= gy * 12;
                     for (int x = 0; x < 6; x++) {
                        int px = BX * 6 + x;
                        if (px >= params->w)
                           px = params->w - 1;
                        px -= gx * 12;
                        memcpy(texels + (y * 6 + x) * 4, scratch + py * 48 + px * 4, 4);
                     }
                  }
                  unsigned char *blk = (unsigned char *)dst_base +
                     ((size_t)BY * params->block_x + BX) * 16;
                  astc_encode_block_6x6(texels, params->has_alpha, blk);
               }
            }
         }
      }
      return;
   }

   /* 8x8 ASTC: one block per 2x2 group of BC blocks. Decode the 4 BC blocks into
    * an 8x8 RGBA scratch (row stride 32 bytes), then encode one ASTC 8x8 block. */
   if (params->astc8) {
      for (int by = 0; by < params->block_y_count; by++) {
         int BY = params->block_y_start + by;
         for (int BX = 0; BX < params->block_x; BX++) {
            unsigned char scratch[256];
            int blk_alpha = 0;
            memset(scratch, 0, sizeof(scratch));
            for (int dy = 0; dy < 2; dy++) {
               for (int dx = 0; dx < 2; dx++) {
                  int bcx = 2 * BX + dx, bcy = 2 * BY + dy;
                  if (bcx >= params->bc_bx || bcy >= params->bc_by)
                     continue;
                  const char *sblk = params->src +
                     ((size_t)bcy * params->block_x_src + bcx) * block_size;
                  unsigned char *dsub = scratch + (dy * 4) * 32 + (dx * 4) * 4;
                  bcn_decode_rgba_block(params->format, sblk, dsub, 32);
                  if (params->has_alpha &&
                      (!bc1 || bc1_block_has_alpha((const unsigned char *)sblk)))
                     blk_alpha = 1;
               }
            }
            unsigned char *blk = (unsigned char *)dst_base +
               ((size_t)BY * params->block_x + BX) * 16;
            if (params->format == VK_FORMAT_BC5_UNORM_BLOCK)
               astc_encode_rg_8x8(scratch, blk);
            else
               astc_encode_block_8x8(scratch, blk_alpha, blk);
         }
      }
      return;
   }

   for (int by = 0; by < params->block_y_count; by++) {
      int row = params->block_y_start + by;
      for (int bx = 0; bx < params->block_x; bx++) {
         /* Absolute source addressing with the source row block stride, so a
          * padded bufferRowLength does not misalign subsequent rows. */
         const char *src = params->src +
            ((size_t)row * params->block_x_src + bx) * block_size;

         /* ASTC target: decode the BCn block into a 4x4 RGBA scratch, then
          * re-encode it as one ASTC 4x4 block written to the block grid. */
         if (params->astc) {
            uint8_t scratch[64];
            bcn_decode_rgba_block(params->format, src, scratch, 16);
            uint8_t *blk = (uint8_t *)dst_base +
               ((size_t)row * params->block_x + bx) * 16;
            if (params->format == VK_FORMAT_BC5_UNORM_BLOCK)
               astc_encode_rg_4x4(scratch, blk);
            else
               astc_encode_block_4x4(scratch,
                  params->has_alpha &&
                     (!bc1 || bc1_block_has_alpha((const unsigned char *)src)),
                  blk);
            continue;
         }

         size_t pixel_x = (size_t)bx * 4;
         size_t pixel_y = (size_t)row * 4;
         char *dst = dst_base + pixel_y * params->stride +
                     pixel_x * params->texel_size;
         int bw = params->w - (int)pixel_x;
         int bh = params->h - (int)pixel_y;

         if (bw >= 4 && bh >= 4) {
            bcn_decode_native_block(params->format, src, dst, params->stride,
                                    params->texel_size);
         } else {
            /* Edge block of a mip that is not a multiple of 4: the decoder
             * writes all 16 texels, so go through a scratch and keep only the
             * ones inside the image instead of running into the next row (or
             * past the end of the buffer). */
            char tmp[4 * 4 * 8];
            size_t tmp_stride = (size_t)4 * params->texel_size;
            bcn_decode_native_block(params->format, src, tmp, tmp_stride,
                                    params->texel_size);
            for (int y = 0; y < bh && y < 4; y++)
               memcpy(dst + (size_t)y * params->stride, tmp + y * tmp_stride,
                      (size_t)(bw < 4 ? bw : 4) * params->texel_size);
         }
      }
   }
}

/* ------------------------------------------------------------------------ *
 * Worker pool. A transcode used to create and join a fresh set of threads per
 * mip; textures arrive by the thousand, so keep the workers around. The caller
 * always runs one slice itself, which also keeps single-core devices (and a
 * failed pthread_create) working: no workers just means everything runs inline.
 * ------------------------------------------------------------------------ */
#define BCN_MAX_THREADS 32

struct bcn_batch {
   pthread_mutex_t m;
   pthread_cond_t c;
   int pending;
};

struct bcn_job {
   struct bcn_job *next;
   struct decompression_params *params;
   struct bcn_batch *batch;
};

static struct {
   pthread_mutex_t m;
   pthread_cond_t c;
   struct bcn_job *head, *tail;
   int workers;
} bcn_pool = { PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, NULL, NULL, 0 };

static pthread_once_t bcn_pool_once = PTHREAD_ONCE_INIT;

static void *
bcn_pool_worker(void *unused)
{
   (void)unused;
   pthread_mutex_lock(&bcn_pool.m);
   for (;;) {
      while (!bcn_pool.head)
         pthread_cond_wait(&bcn_pool.c, &bcn_pool.m);
      struct bcn_job *job = bcn_pool.head;
      bcn_pool.head = job->next;
      if (!bcn_pool.head)
         bcn_pool.tail = NULL;
      pthread_mutex_unlock(&bcn_pool.m);

      /* The job lives on the submitter's stack and dies once pending hits 0. */
      struct bcn_batch *batch = job->batch;
      decompression_routine(job->params);
      pthread_mutex_lock(&batch->m);
      if (--batch->pending == 0)
         pthread_cond_signal(&batch->c);
      pthread_mutex_unlock(&batch->m);

      pthread_mutex_lock(&bcn_pool.m);
   }
   return NULL;
}

static void
bcn_pool_init(void)
{
   long cores = sysconf(_SC_NPROCESSORS_ONLN);
   if (cores < 1)
      cores = 1;
   if (cores > BCN_MAX_THREADS)
      cores = BCN_MAX_THREADS;

   pthread_attr_t attr;
   pthread_attr_init(&attr);
   pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
   pthread_attr_setstacksize(&attr, 1024 * 1024);
   for (long i = 1; i < cores; i++) {
      pthread_t t;
      if (pthread_create(&t, &attr, bcn_pool_worker, NULL) != 0) {
         WRAPPER_LOG(error, "BCn worker %ld not started", i);
         break;
      }
      bcn_pool.workers++;
   }
   pthread_attr_destroy(&attr);
}

/* Run the template over `rows` block rows (ASTC grid rows, or 12x12 group rows
 * for 6x6), split across the pool and the calling thread. */
static void
bcn_run_rows(const struct decompression_params *tmpl, int rows)
{
   const struct bcn_config *cfg = bcn_config();
   int n = 1;

   if (!cfg->no_thread && rows >= 4) {
      pthread_once(&bcn_pool_once, bcn_pool_init);
      n = bcn_pool.workers + 1;
      if (n > rows)
         n = rows;
   }

   struct decompression_params args[BCN_MAX_THREADS];
   struct bcn_job jobs[BCN_MAX_THREADS];
   int per = rows / n, rem = rows % n, cur = 0;

   for (int i = 0; i < n; i++) {
      int count = per + (i < rem ? 1 : 0);
      args[i] = *tmpl;
      args[i].block_y_start = cur;
      args[i].block_y_count = count;
      cur += count;
   }

   if (n == 1) {
      decompression_routine(&args[0]);
      return;
   }

   WRAPPER_LOG(bcn, "Decompressing %dx%d BCN %d texture using %d threads",
               tmpl->w, tmpl->h, tmpl->format, n);

   struct bcn_batch batch = { .pending = n - 1 };
   pthread_mutex_init(&batch.m, NULL);
   pthread_cond_init(&batch.c, NULL);

   pthread_mutex_lock(&bcn_pool.m);
   for (int i = 1; i < n; i++) {
      jobs[i].next = NULL;
      jobs[i].params = &args[i];
      jobs[i].batch = &batch;
      if (bcn_pool.tail)
         bcn_pool.tail->next = &jobs[i];
      else
         bcn_pool.head = &jobs[i];
      bcn_pool.tail = &jobs[i];
   }
   pthread_mutex_unlock(&bcn_pool.m);
   pthread_cond_broadcast(&bcn_pool.c);

   decompression_routine(&args[0]);

   pthread_mutex_lock(&batch.m);
   while (batch.pending)
      pthread_cond_wait(&batch.c, &batch.m);
   pthread_mutex_unlock(&batch.m);
   pthread_cond_destroy(&batch.c);
   pthread_mutex_destroy(&batch.m);
}

VkFormat 
get_decode_format_for_bcn(VkFormat bcn_format)
{
   switch(bcn_format) {
      case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
      case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
      case VK_FORMAT_BC2_SRGB_BLOCK:
      case VK_FORMAT_BC3_SRGB_BLOCK:
      case VK_FORMAT_BC7_SRGB_BLOCK:
         return VK_FORMAT_R8G8B8A8_SRGB;
      case VK_FORMAT_BC4_UNORM_BLOCK:
         return VK_FORMAT_R8_UNORM;
      case VK_FORMAT_BC4_SNORM_BLOCK:
         return VK_FORMAT_R8_SNORM;
      case VK_FORMAT_BC5_UNORM_BLOCK:
          return VK_FORMAT_R8G8_UNORM;
      case VK_FORMAT_BC5_SNORM_BLOCK:
         return VK_FORMAT_R8G8_SNORM;
      case VK_FORMAT_BC6H_SFLOAT_BLOCK:
      case VK_FORMAT_BC6H_UFLOAT_BLOCK:
         return VK_FORMAT_R16G16B16_SFLOAT;
      default:
         return VK_FORMAT_R8G8B8A8_UNORM;
   }
}

/* Storage (image) format. For BC1/2/3/4/5/7 this is ASTC 4x4 (kept compressed);
 * the SNORM BC4/5 variants and BC6H stay uncompressed (decoded). */
VkFormat
get_format_for_bcn(VkFormat bcn_format)
{
   if (astc_enabled()) {
      int idx = -1;
      switch (bcn_format) {
      case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
      case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
      case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
      case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
         idx = BCN_POL_BC1; break;
      case VK_FORMAT_BC2_SRGB_BLOCK:
      case VK_FORMAT_BC2_UNORM_BLOCK:
         idx = BCN_POL_BC2; break;
      case VK_FORMAT_BC3_SRGB_BLOCK:
      case VK_FORMAT_BC3_UNORM_BLOCK:
         idx = BCN_POL_BC3; break;
      case VK_FORMAT_BC7_SRGB_BLOCK:
      case VK_FORMAT_BC7_UNORM_BLOCK:
         idx = BCN_POL_BC7; break;
      case VK_FORMAT_BC4_UNORM_BLOCK:
         idx = BCN_POL_BC4; break;
      case VK_FORMAT_BC5_UNORM_BLOCK:
         idx = BCN_POL_BC5; break;
      case VK_FORMAT_BC6H_UFLOAT_BLOCK:
      case VK_FORMAT_BC6H_SFLOAT_BLOCK:
         if (__atomic_load_n(&bcn_astc_hdr, __ATOMIC_RELAXED) && bcn_policy_bc6h_hdr())
            return VK_FORMAT_ASTC_4x4_SFLOAT_BLOCK;
         break;
      default:
         break;
      }
      if (idx >= 0) {
         int srgb = get_decode_format_for_bcn(bcn_format) == VK_FORMAT_R8G8B8A8_SRGB;
         int b = bcn_astc_edge(idx);
         if (b == 6)
            return srgb ? VK_FORMAT_ASTC_6x6_SRGB_BLOCK : VK_FORMAT_ASTC_6x6_UNORM_BLOCK;
         if (b == 8)
            return srgb ? VK_FORMAT_ASTC_8x8_SRGB_BLOCK : VK_FORMAT_ASTC_8x8_UNORM_BLOCK;
         return srgb ? VK_FORMAT_ASTC_4x4_SRGB_BLOCK : VK_FORMAT_ASTC_4x4_UNORM_BLOCK;
      }
   }
   return get_decode_format_for_bcn(bcn_format);
}

/* Bytes needed to hold the transcoded upload for a w*h mip of this BCn format:
 * ASTC block bytes for ASTC targets, else linear decoded size. */
size_t
bcn_upload_size(VkFormat bcn_format, int w, int h)
{
   VkFormat img = get_format_for_bcn(bcn_format);
   if (is_astc_6x6(img))
      return (size_t)((w + 5) / 6) * ((h + 5) / 6) * 16;
   if (is_astc_8x8(img))
      return (size_t)((w + 7) / 8) * ((h + 7) / 8) * 16;
   if (is_astc_4x4(img) || is_astc_hdr_4x4(img))
      return (size_t)((w + 3) / 4) * ((h + 3) / 4) * 16;
   return (size_t)w * h *
      get_texel_size_for_format(get_decode_format_for_bcn(bcn_format));
}

int 
get_texel_size_for_format(VkFormat format) 
{
   switch (format) {
      case VK_FORMAT_R16G16B16_SFLOAT:
         return 6;
      case VK_FORMAT_R8G8_UNORM:
      case VK_FORMAT_R8G8_SNORM:
         return 2;
      case VK_FORMAT_R8_UNORM:
      case VK_FORMAT_R8_SNORM:
         return 1;
      default:
         return 4;
   }
}

int
is_emulated_bcn(struct wrapper_physical_device *pdev, VkFormat format)
{
   switch(format) {
      case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
      case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
      case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
      case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
      case VK_FORMAT_BC2_SRGB_BLOCK:
      case VK_FORMAT_BC2_UNORM_BLOCK:
      case VK_FORMAT_BC3_UNORM_BLOCK:
      case VK_FORMAT_BC3_SRGB_BLOCK:
         if (pdev->emulate_bcn == 3 && 
             pdev->driver_properties.driverID == VK_DRIVER_ID_SAMSUNG_PROPRIETARY)
         {
            return 0;
         }
         else if (pdev->emulate_bcn > 1) {
            return 1;
         } else {
            return 0;
         }
         break;
      case VK_FORMAT_BC4_UNORM_BLOCK:
      case VK_FORMAT_BC4_SNORM_BLOCK:
      case VK_FORMAT_BC5_SNORM_BLOCK:
      case VK_FORMAT_BC5_UNORM_BLOCK:
      case VK_FORMAT_BC6H_SFLOAT_BLOCK:
      case VK_FORMAT_BC6H_UFLOAT_BLOCK: 
      case VK_FORMAT_BC7_SRGB_BLOCK:
      case VK_FORMAT_BC7_UNORM_BLOCK:
         if (pdev->emulate_bcn > 1)
            return 1;
         else
            return 0;
         break;
      default:
         return 0;
   }
}

/* Disk cache entries are named by content only, so an entry produced on any
 * device (or by a server) matches on every other one:
 *   <src>_<w>x<h>_<xxh64 of the logical block rows>.<out>
 * <src> folds sRGB/UNORM (the transcode output is identical), <out> is the
 * storage format the wrapper picked (a4 = ASTC 4x4, a6 = ASTC 6x6, a8 = ASTC 8x8,
 * h4 = ASTC 4x4 HDR, rgba8 ...).
 * The hash walks the logical rows only, so a padded bufferRowLength yields the
 * same key as a tightly packed upload of the same mip. */
#define BCN_CACHE_HASH_SEED 1
/* Upload mode: larger mips keep only their .src; the server pack has them. */
#define BCN_CACHE_UPLOAD_ENTRY_MAX 1024

static const char *
bcn_cache_src_tag(VkFormat format)
{
   switch (format) {
   case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
   case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
   case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
   case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
      return "bc1";
   case VK_FORMAT_BC2_UNORM_BLOCK:
   case VK_FORMAT_BC2_SRGB_BLOCK:
      return "bc2";
   case VK_FORMAT_BC3_UNORM_BLOCK:
   case VK_FORMAT_BC3_SRGB_BLOCK:
      return "bc3";
   case VK_FORMAT_BC4_UNORM_BLOCK:
      return "bc4";
   case VK_FORMAT_BC4_SNORM_BLOCK:
      return "bc4s";
   case VK_FORMAT_BC5_UNORM_BLOCK:
      return "bc5";
   case VK_FORMAT_BC5_SNORM_BLOCK:
      return "bc5s";
   case VK_FORMAT_BC6H_UFLOAT_BLOCK:
      return "bc6hu";
   case VK_FORMAT_BC6H_SFLOAT_BLOCK:
      return "bc6hs";
   case VK_FORMAT_BC7_UNORM_BLOCK:
   case VK_FORMAT_BC7_SRGB_BLOCK:
      return "bc7";
   default:
      return NULL;
   }
}

static const char *
bcn_cache_out_tag(VkFormat image_format)
{
   if (is_astc_4x4(image_format))
      return "a4";
   if (is_astc_6x6(image_format))
      return "a6";
   if (is_astc_8x8(image_format))
      return "a8";
   if (is_astc_hdr_4x4(image_format))
      return "h4";
   switch (image_format) {
   case VK_FORMAT_R8G8B8A8_UNORM:
   case VK_FORMAT_R8G8B8A8_SRGB:
      return "rgba8";
   case VK_FORMAT_R8G8_UNORM:
      return "rg8";
   case VK_FORMAT_R8G8_SNORM:
      return "rg8s";
   case VK_FORMAT_R8_UNORM:
      return "r8";
   case VK_FORMAT_R8_SNORM:
      return "r8s";
   case VK_FORMAT_R16G16B16_SFLOAT:
      return "rgb16f";
   default:
      return NULL;
   }
}

static uint64_t
bcn_cache_hash(const char *src, int block_x, int block_y, int block_x_src,
               int block_size)
{
   size_t row = (size_t)block_x * block_size;
   size_t stride = (size_t)block_x_src * block_size;
   /* Tightly packed: one shot. Padded rows hash the logical rows only, which
    * produces the same value as the packed upload (streaming == one shot). */
   if (stride == row)
      return XXH64(src, row * block_y, BCN_CACHE_HASH_SEED);

   XXH64_state_t state;
   XXH64_reset(&state, BCN_CACHE_HASH_SEED);
   for (int by = 0; by < block_y; by++)
      XXH64_update(&state, src + (size_t)by * stride, row);
   return XXH64_digest(&state);
}

static char *
bcn_cache_filename(const char *dir, VkFormat src_format, VkFormat image_format,
                   int w, int h, const char *src, int block_x, int block_y,
                   int block_x_src, int block_size)
{
   const char *src_tag = bcn_cache_src_tag(src_format);
   const char *out_tag = bcn_cache_out_tag(image_format);
   if (!src_tag || !out_tag)
      return NULL;
   uint64_t hash = bcn_cache_hash(src, block_x, block_y, block_x_src, block_size);
   char *name = NULL;
   if (asprintf(&name, "%s/%s_%dx%d_%016llx.%s", dir, src_tag, w, h,
                (unsigned long long)hash, out_tag) < 0)
      return NULL;
   return name;
}

/* Server-made entries arrive zstd compressed. libzstd ships in the imagefs,
 * so resolve it lazily; without it compressed entries are simply misses. */
typedef size_t (*bcn_zstd_decompress_t)(void *, size_t, const void *, size_t);
typedef unsigned (*bcn_zstd_is_error_t)(size_t);
typedef unsigned long long (*bcn_zstd_frame_size_t)(const void *, size_t);

static bcn_zstd_decompress_t bcn_zstd_decompress;
static bcn_zstd_is_error_t bcn_zstd_is_error;
static bcn_zstd_frame_size_t bcn_zstd_frame_size;
static pthread_once_t bcn_zstd_once = PTHREAD_ONCE_INIT;

static void
bcn_zstd_init(void)
{
   void *lib = dlopen("libzstd.so.1", RTLD_LAZY);
   if (!lib)
      lib = dlopen("libzstd.so", RTLD_LAZY);
   if (lib) {
      bcn_zstd_decompress = (bcn_zstd_decompress_t)dlsym(lib, "ZSTD_decompress");
      bcn_zstd_is_error = (bcn_zstd_is_error_t)dlsym(lib, "ZSTD_isError");
      bcn_zstd_frame_size = (bcn_zstd_frame_size_t)dlsym(lib, "ZSTD_getFrameContentSize");
   }
   if (!(bcn_zstd_decompress && bcn_zstd_is_error))
      WRAPPER_LOG(bcn, "No libzstd, compressed cache entries are misses");
}

static int
bcn_zstd_load(void)
{
   pthread_once(&bcn_zstd_once, bcn_zstd_init);
   return bcn_zstd_decompress && bcn_zstd_is_error;
}

/* fp is the already open entry, positioned anywhere. */
static int
bcn_cache_read_zstd(FILE *fp, const char *path, void *dst, size_t size,
                    size_t file_size)
{
   /* A frame is never meaningfully bigger than its content. */
   if (file_size < 4 || file_size > size * 2 + 4096 || !bcn_zstd_load())
      return 0;
   unsigned char *buf = malloc(file_size);
   if (!buf)
      return 0;
   rewind(fp);
   size_t length = fread(buf, 1, file_size, fp);
   int ok = 0;
   if (length == file_size && buf[0] == 0x28 && buf[1] == 0xB5 &&
       buf[2] == 0x2F && buf[3] == 0xFD) {
      unsigned long long content = bcn_zstd_frame_size ?
         bcn_zstd_frame_size(buf, length) : (unsigned long long)size;
      if (content == (unsigned long long)size ||
          content == ~0ULL || content == ~0ULL - 1) {
         size_t out = bcn_zstd_decompress(dst, size, buf, length);
         ok = !bcn_zstd_is_error(out) && out == size;
      }
      if (!ok)
         WRAPPER_LOG(bcn, "Failed to decompress texture %s from cache", path);
   }
   free(buf);
   return ok;
}

static int
bcn_cache_read(const char *path, void *dst, size_t size, int *raw)
{
   struct stat sb;
   *raw = 0;
   FILE *fp = fopen(path, "rb");
   if (!fp)
      return 0;
   if (fstat(fileno(fp), &sb) != 0) {
      fclose(fp);
      return 0;
   }
   /* The zstd magic decides, not the size: a frame can be exactly raw size. */
   unsigned char magic[4];
   int ok;
   if (fread(magic, 1, 4, fp) == 4 && magic[0] == 0x28 && magic[1] == 0xB5 &&
       magic[2] == 0x2F && magic[3] == 0xFD) {
      ok = bcn_cache_read_zstd(fp, path, dst, size, (size_t)sb.st_size);
      fclose(fp);
      return ok;
   }
   if ((size_t)sb.st_size != size) {
      fclose(fp);
      return 0;
   }
   rewind(fp);
   size_t length = fread(dst, 1, size, fp);
   fclose(fp);
   if (length != size) {
      unlink(path);
      return 0;
   }
   *raw = 1;
   return 1;
}

static void
bcn_cache_write(const char *path, const void *data, size_t size)
{
   /* pid + a process wide counter: two threads (or processes) producing the
    * same key never share a temp file, so nobody renames a half written one. */
   static unsigned int serial;
   char *tmp = NULL;
   if (asprintf(&tmp, "%s.%d.%u.tmp", path, (int)getpid(),
                __atomic_fetch_add(&serial, 1, __ATOMIC_RELAXED)) < 0)
      return;
   FILE *fp = fopen(tmp, "wb");
   if (!fp && errno == ENOENT) {
      /* The cache dir went away since start-up (imagefs wiped): recreate it. */
      char *dir = strdup(path);
      char *slash = dir ? strrchr(dir, '/') : NULL;
      if (slash) {
         *slash = 0;
         if (wrapper_mkdir_p(dir, 0700) == 0)
            fp = fopen(tmp, "wb");
      }
      free(dir);
   }
   if (!fp) {
      free(tmp);
      return;
   }
   size_t length = fwrite(data, 1, size, fp);
   int closed = fclose(fp);
   if (length == size && closed == 0 && rename(tmp, path) == 0)
      WRAPPER_LOG(bcn, "Saved texture %s to cache", path);
   else {
      WRAPPER_LOG(bcn, "Failed to save texture %s to cache", path);
      unlink(tmp);
   }
   free(tmp);
}

/* Beside a device-made entry, keep the tightly packed source mip as
 * <key>.src so the app can hand it to the server for a better encode and
 * replace this entry later. */
static void
bcn_cache_write_source(const char *entry_path, const char *src, int block_x,
                       int block_y, int block_x_src, int block_size)
{
   if (!bcn_upload_enabled())
      return;
   const char *dot = strrchr(entry_path, '.');
   if (!dot || (strcmp(dot + 1, "a4") != 0 && strcmp(dot + 1, "a6") != 0 &&
                strcmp(dot + 1, "a8") != 0 && strcmp(dot + 1, "h4") != 0))
      return;
   size_t stem = (size_t)(dot - entry_path);
   char *path = malloc(stem + 5);
   if (!path)
      return;
   memcpy(path, entry_path, stem);
   memcpy(path + stem, ".src", 5);
   size_t row = (size_t)block_x * block_size;
   size_t stride = (size_t)block_x_src * block_size;
   if (stride == row) {
      bcn_cache_write(path, src, row * block_y);
   } else {
      char *tight = malloc(row * block_y);
      if (tight) {
         for (int by = 0; by < block_y; by++)
            memcpy(tight + (size_t)by * row, src + (size_t)by * stride, row);
         bcn_cache_write(path, tight, row * block_y);
         free(tight);
      }
   }
   free(path);
}

static int
bcn_cache_source_exists(const char *entry_path)
{
   const char *dot = strrchr(entry_path, '.');
   if (!dot)
      return 1;
   size_t stem = (size_t)(dot - entry_path);
   char *path = malloc(stem + 5);
   if (!path)
      return 1;
   memcpy(path, entry_path, stem);
   memcpy(path + stem, ".src", 5);
   int exists = access(path, F_OK) == 0;
   free(path);
   return exists;
}

/* BC6H -> ASTC 4x4 HDR fallback when no server entry exists: each block
 * becomes a void-extent block holding the mean of its in-bounds texels.
 * Khronos Data Format Spec, ASTC "Void-Extent Blocks" (2D):
 *   bits [8:0]     0x1FC  void-extent block mode
 *   bit  9         1      HDR, the colour is FP16 (0 = UNORM16)
 *   bits [11:10]   0b11   reserved
 *   bits [63:12]   all 1  min/max S and T (13 bits each), no extent
 *   bits [127:64]  R, G, B, A as 16-bit values, R lowest
 * ASTC HDR has no negative values, so SFLOAT texels clamp at 0. */
static void
bcn_encode_bc6h_void(const char *src, uint8_t *dst, int w, int h,
                     int block_x, int block_y, int block_x_src, int is_signed)
{
   static const uint8_t head[8] = { 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
   for (int by = 0; by < block_y; by++) {
      for (int bx = 0; bx < block_x; bx++) {
         unsigned short half[48];
         bcdec_bc6h_half(src + ((size_t)by * block_x_src + bx) * 16, half, 12,
                         is_signed);
         float sum[3] = { 0, 0, 0 };
         int n = 0;
         for (int y = 0; y < 4 && by * 4 + y < h; y++) {
            for (int x = 0; x < 4 && bx * 4 + x < w; x++) {
               for (int c = 0; c < 3; c++) {
                  _Float16 v;
                  memcpy(&v, &half[(y * 4 + x) * 3 + c], 2);
                  sum[c] += (float)v;
               }
               n++;
            }
         }
         uint8_t *blk = dst + ((size_t)by * block_x + bx) * 16;
         memcpy(blk, head, 8);
         for (int c = 0; c < 4; c++) {
            float f = c == 3 ? 1.0f : (n ? sum[c] / n : 0.0f);
            if (!(f > 0.0f))
               f = 0.0f;
            if (f > 65504.0f)
               f = 65504.0f;
            _Float16 hv = (_Float16)f;
            uint16_t bits;
            memcpy(&bits, &hv, 2);
            blk[8 + 2 * c] = bits & 0xFF;
            blk[9 + 2 * c] = bits >> 8;
         }
      }
   }
}

void
decompress_bcn_format(void *srcBuffer,
                      void *dstBuffer,
                      int w,
                      int h,
                      int src_w,
                      VkFormat format,
                      size_t offset)
{
   const struct bcn_config *cfg = bcn_config();
   VkFormat img_format = get_format_for_bcn(format);

   int astc = is_astc_4x4(img_format);
   int astc8 = is_astc_8x8(img_format);
   int astc6 = is_astc_6x6(img_format);
   int hdr = is_astc_hdr_4x4(img_format);
   int has_alpha = bcn_has_alpha(format);
   int texel_size = get_texel_size_for_format(get_decode_format_for_bcn(format));
   int block_size = get_block_size(format);
   int block_x = (w + 3) / 4;
   /* source row stride in blocks (bufferRowLength may pad rows wider than w) */
   int block_x_src = ((src_w > 0 ? src_w : w) + 3) / 4;
   int block_y = (h + 3) / 4;
   int block_x8 = (w + 7) / 8;
   int block_y8 = (h + 7) / 8;
   int block_x6 = (w + 5) / 6;
   int block_y6 = (h + 5) / 6;
   size_t stride = (size_t)w * texel_size;
   size_t uncompressed_size = bcn_upload_size(format, w, h);
   const char *src = (const char *)srcBuffer + offset;
   char *dst = dstBuffer;

   if (cfg->mark && !astc && !astc6 && !astc8 && !hdr) {
      WRAPPER_LOG(bcn, "Filling %dx%d BCn %d texture with custom color", w, h, format);

      for (int i = 0; i < h; i++) {
         for (int j = 0; j < w; j++) {
            char *px = dstBuffer + (size_t)i * stride + (size_t)j * texel_size;
            
            switch(format) {
               case VK_FORMAT_BC1_RGB_UNORM_BLOCK:
               case VK_FORMAT_BC1_RGB_SRGB_BLOCK:
               case VK_FORMAT_BC1_RGBA_SRGB_BLOCK:
               case VK_FORMAT_BC1_RGBA_UNORM_BLOCK:
                  /* Yellow */
                  px[0] = 0xFF;
                  px[1] = 0xFF;
                  px[2] = 0;
                  px[3] = 255;
                  break;
               case VK_FORMAT_BC2_SRGB_BLOCK:
               case VK_FORMAT_BC2_UNORM_BLOCK:
                  /* Blue */
                  px[0] = 0;
                  px[1] = 0;
                  px[2] = 0xFF;
                  px[3] = 255;
                  break;
                case VK_FORMAT_BC3_UNORM_BLOCK:
                case VK_FORMAT_BC3_SRGB_BLOCK:
                  /* Light Blue */
                  px[0] = 0;
                  px[1] = 0xFF;
                  px[2] = 0xFF;
                  px[3] = 255;
                  break;
               case VK_FORMAT_BC4_UNORM_BLOCK:
               case VK_FORMAT_BC4_SNORM_BLOCK:
                  /* Red */
                  px[0] = 0xFF;
                  break;
               case VK_FORMAT_BC5_UNORM_BLOCK:
               case VK_FORMAT_BC5_SNORM_BLOCK:
                  /* Green */
                  px[0] = 0;
                  px[1] = 0xFF;
                  break;
               case VK_FORMAT_BC6H_SFLOAT_BLOCK:
               case VK_FORMAT_BC6H_UFLOAT_BLOCK:
                  /* Purple */
                  px[0] = 0x90;
                  px[1] = 0x40;
                  px[2] = 0xA0;
                  break;
               case VK_FORMAT_BC7_UNORM_BLOCK:
               case VK_FORMAT_BC7_SRGB_BLOCK:
                  /* Black */
                  px[0] = 0xFF;
                  px[1] = 0;
                  px[2] = 0xFF;
                  px[3] = 255;
                  break;
               default:
                  break;
            }
         }
      }

      return;
   }

   /* Optional disk cache of the transcoded output, keyed by a hash of the
    * compressed source. Skips decode+encode on subsequent loads. */
   char *cache_filename = NULL;
   if (cfg->use_cache && cfg->cache_dir) {
      cache_filename = bcn_cache_filename(cfg->cache_dir, format, img_format,
         w, h, src, block_x, block_y, block_x_src, block_size);
      int cache_raw = 0;
      if (cache_filename &&
          bcn_cache_read(cache_filename, dst, uncompressed_size, &cache_raw)) {
         WRAPPER_LOG(bcn, "Restored texture %s from cache", cache_filename);
         if (cache_raw && w >= 8 && h >= 8 &&
             !bcn_cache_source_exists(cache_filename))
            bcn_cache_write_source(cache_filename, src, block_x, block_y,
                                   block_x_src, block_size);
         free(cache_filename);
         return;
      }
   }

   if (hdr) {
      bcn_encode_bc6h_void(src, (uint8_t *)dst, w, h, block_x, block_y,
                           block_x_src, format == VK_FORMAT_BC6H_SFLOAT_BLOCK);
   } else {
      struct decompression_params tmpl = {
         .src = src,
         .dst = dst,
         .block_x_src = block_x_src,
         .format = format,
         .stride = stride,
         .texel_size = texel_size,
         .bc_bx = block_x,
         .bc_by = block_y,
         .w = w,
         .h = h,
         .has_alpha = has_alpha,
      };
      int rows;
      if (astc6) {
         /* 6x6 ASTC: split by rows of 12x12-texel groups (2 ASTC block rows). */
         tmpl.astc6 = 1;
         tmpl.block_x = block_x6;
         tmpl.astc_by = block_y6;
         rows = (block_y6 + 1) / 2;
      } else if (astc8) {
         /* 8x8 ASTC: each block covers a 2x2 group of BC blocks (8 = 2*4, aligned). */
         tmpl.astc8 = 1;
         tmpl.block_x = block_x8;
         rows = block_y8;
      } else {
         tmpl.astc = astc;
         tmpl.block_x = block_x;
         rows = block_y;
      }
      bcn_run_rows(&tmpl, rows);
   }

   if (cfg->use_cache && cache_filename) {
      if (!cfg->upload ||
          (w <= BCN_CACHE_UPLOAD_ENTRY_MAX && h <= BCN_CACHE_UPLOAD_ENTRY_MAX))
         bcn_cache_write(cache_filename, dst, uncompressed_size);
      if (w >= 8 && h >= 8)
         bcn_cache_write_source(cache_filename, src, block_x, block_y, block_x_src, block_size);
   }

   free(cache_filename);
}

/* Policy maxdim: emulated BCn images whose base level exceeds it drop their
 * top mips so the stored base is <= maxdim. 0 = off. */
int
bcn_max_dim(void)
{
   pthread_once(&bcn_pol_once, bcn_policy_parse);
   return (int)bcn_pol.maxdim;
}

/* Levels to drop from an emulated BCn image (caller checks the format): 2D,
 * sampled with at most transfer src/dst besides, not sparse/aliased, and with
 * a full chain or more than mip_drop + 1 levels. */
uint32_t
bcn_cap_mip_drop(const VkImageCreateInfo *ci)
{
   uint32_t cap = (uint32_t)bcn_max_dim();
   if (!cap || ci->imageType != VK_IMAGE_TYPE_2D)
      return 0;
   if (!(ci->usage & VK_IMAGE_USAGE_SAMPLED_BIT) ||
       (ci->usage & ~(VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                      VK_IMAGE_USAGE_TRANSFER_SRC_BIT)))
      return 0;
   if (ci->flags & (VK_IMAGE_CREATE_SPARSE_BINDING_BIT |
                    VK_IMAGE_CREATE_SPARSE_RESIDENCY_BIT |
                    VK_IMAGE_CREATE_SPARSE_ALIASED_BIT |
                    VK_IMAGE_CREATE_ALIAS_BIT))
      return 0;
   uint32_t w = ci->extent.width, h = ci->extent.height, k = 0;
   while ((w >> k) > cap || (h >> k) > cap)
      k++;
   if (!k)
      return 0;
   uint32_t full = 1;
   while (((w > h ? w : h) >> full) > 0)
      full++;
   if (ci->mipLevels < full && ci->mipLevels <= k + 1)
      return 0;
   return k;
}

/* Remap a mip range onto the capped image; false when it only covers dropped
 * levels. VK_REMAINING_MIP_LEVELS stays. */
bool
bcn_cap_range(uint32_t mip_drop, VkImageSubresourceRange *range)
{
   if (!mip_drop)
      return true;
   uint32_t base = range->baseMipLevel;
   uint32_t new_base = base > mip_drop ? base - mip_drop : 0;
   if (range->levelCount != VK_REMAINING_MIP_LEVELS) {
      uint32_t end = base + range->levelCount;
      if (end <= mip_drop)
         return false;
      range->levelCount = end - mip_drop - new_base;
   }
   range->baseMipLevel = new_base;
   return true;
}

/* Copies into dropped levels are removed, the rest move down by mip_drop.
 * Returns the number of regions kept in kept[]. */
uint32_t
bcn_cap_copy_regions(uint32_t mip_drop, const VkBufferImageCopy *regions,
                     uint32_t count, VkBufferImageCopy *kept)
{
   uint32_t n = 0;
   for (uint32_t i = 0; i < count; i++) {
      if (regions[i].imageSubresource.mipLevel < mip_drop)
         continue;
      kept[n] = regions[i];
      kept[n].imageSubresource.mipLevel -= mip_drop;
      n++;
   }
   return n;
}

/* WRAPPER_BCN_UPLOAD=1: leave .src sidecars and needs_full_res for the server. */
int
bcn_upload_enabled(void)
{
   return bcn_config()->upload;
}

int
bcn_cache_enabled(void)
{
   const struct bcn_config *cfg = bcn_config();
   return cfg->use_cache && cfg->cache_dir;
}

/* A dropped (capped) mip is never transcoded, but its .src still goes beside
 * its cache key so the server pack stays complete for uncapped devices. */
void
bcn_cache_note_source(void *srcBuffer, int w, int h, int src_w,
                      VkFormat format, size_t offset)
{
   if (!bcn_cache_enabled() || !bcn_upload_enabled() || w < 8 || h < 8)
      return;
   const char *dir = bcn_config()->cache_dir;
   const char *src = (const char *)srcBuffer + offset;
   int block_size = get_block_size(format);
   int block_x = (w + 3) / 4;
   int block_y = (h + 3) / 4;
   int block_x_src = ((src_w > 0 ? src_w : w) + 3) / 4;
   char *name = bcn_cache_filename(dir, format, get_format_for_bcn(format), w, h,
                                   src, block_x, block_y, block_x_src, block_size);
   if (name && !bcn_cache_source_exists(name))
      bcn_cache_write_source(name, src, block_x, block_y, block_x_src, block_size);
   free(name);
}

/* Cache step of the GPU transcode path, keyed exactly as decompress_bcn_format.
 * Hit: returns the entry (bcn_upload_size bytes, caller frees). Miss: leaves the
 * .src sidecar and returns NULL; the GPU result is never read back. */
void *
bcn_cache_gpu_lookup(void *srcBuffer, int w, int h, int src_w, VkFormat format,
                     size_t offset, size_t *size)
{
   if (!bcn_cache_enabled())
      return NULL;
   const char *dir = bcn_config()->cache_dir;
   const char *src = (const char *)srcBuffer + offset;
   int block_size = get_block_size(format);
   int block_x = (w + 3) / 4;
   int block_y = (h + 3) / 4;
   int block_x_src = ((src_w > 0 ? src_w : w) + 3) / 4;
   char *name = bcn_cache_filename(dir, format, get_format_for_bcn(format), w, h,
                                   src, block_x, block_y, block_x_src, block_size);
   if (!name)
      return NULL;
   size_t sz = bcn_upload_size(format, w, h);
   void *dst = malloc(sz);
   int raw = 0;
   if (dst && bcn_cache_read(name, dst, sz, &raw)) {
      WRAPPER_LOG(bcn, "Restored texture %s from cache", name);
      if (raw && w >= 8 && h >= 8 && !bcn_cache_source_exists(name))
         bcn_cache_write_source(name, src, block_x, block_y, block_x_src, block_size);
      free(name);
      *size = sz;
      return dst;
   }
   free(dst);
   if (w >= 8 && h >= 8 && !bcn_cache_source_exists(name))
      bcn_cache_write_source(name, src, block_x, block_y, block_x_src, block_size);
   free(name);
   return NULL;
}

/* SPIR-V scan, independent of the Mali passes: a shader that takes the size or
 * level count of a 2D sampled image (Dim 2D, MS 0, Sampled 1) or fetches its
 * texels sees capped extents. The first one leaves <cache>/needs_full_res.
 * Returns 1 once such a shader has been seen. */
#define SPV_OP_UNDEF                  1
#define SPV_OP_TYPE_IMAGE             25
#define SPV_OP_TYPE_SAMPLED_IMAGE     27
#define SPV_OP_FUNCTION_PARAMETER     55
#define SPV_OP_LOAD                   61
#define SPV_OP_COPY_OBJECT            83
#define SPV_OP_SAMPLED_IMAGE          86
#define SPV_OP_IMAGE_FETCH            95
#define SPV_OP_IMAGE                  100
#define SPV_OP_IMAGE_QUERY_SIZE_LOD   103
#define SPV_OP_IMAGE_QUERY_SIZE       104
#define SPV_OP_IMAGE_QUERY_LEVELS     106
#define SPV_OP_SELECT                 169
#define SPV_OP_PHI                    245

static int bcn_full_res_seen;

static int
bcn_spirv_needs_full_res(const uint32_t *code, size_t words)
{
   if (words < 5 || code[0] != 0x07230203)
      return 0;
   uint32_t bound = code[3];
   if (!bound || bound > (1u << 22))
      return 0;
   uint8_t *kind = calloc(bound, 1);      /* type id: 1 = 2D sampled image */
   uint32_t *vtype = calloc(bound, 4);    /* value id -> type id */
   int hit = 0;
   if (!kind || !vtype)
      goto out;
   for (size_t i = 5; i < words && !hit;) {
      uint32_t op = code[i] & 0xFFFF, n = code[i] >> 16;
      if (!n || i + n > words)
         break;
      const uint32_t *in = code + i;
      switch (op) {
      case SPV_OP_TYPE_IMAGE:
         if (n >= 8 && in[1] < bound)
            kind[in[1]] = (in[3] == 1 && in[6] == 0 && in[7] == 1) ? 1 : 2;
         break;
      case SPV_OP_TYPE_SAMPLED_IMAGE:
         if (n >= 3 && in[1] < bound && in[2] < bound)
            kind[in[1]] = kind[in[2]];
         break;
      case SPV_OP_UNDEF:
      case SPV_OP_FUNCTION_PARAMETER:
      case SPV_OP_LOAD:
      case SPV_OP_COPY_OBJECT:
      case SPV_OP_SAMPLED_IMAGE:
      case SPV_OP_IMAGE:
      case SPV_OP_SELECT:
      case SPV_OP_PHI:
         if (n >= 3 && in[2] < bound)
            vtype[in[2]] = in[1];
         break;
      case SPV_OP_IMAGE_FETCH:
      case SPV_OP_IMAGE_QUERY_SIZE_LOD:
      case SPV_OP_IMAGE_QUERY_SIZE:
      case SPV_OP_IMAGE_QUERY_LEVELS:
         if (n >= 4 && in[3] < bound && vtype[in[3]] < bound &&
             kind[vtype[in[3]]] == 1)
            hit = 1;
         break;
      default:
         break;
      }
      i += n;
   }
out:
   free(kind);
   free(vtype);
   return hit;
}

int
bcn_scan_shader(const uint32_t *code, size_t size)
{
   if (!bcn_upload_enabled())
      return 0;
   if (__atomic_load_n(&bcn_full_res_seen, __ATOMIC_RELAXED))
      return 1;
   if (!code || !bcn_spirv_needs_full_res(code, size / 4))
      return 0;
   if (__atomic_exchange_n(&bcn_full_res_seen, 1, __ATOMIC_RELAXED))
      return 1;
   const char *dir = bcn_config()->cache_dir;
   char *path = NULL;
   if (dir && asprintf(&path, "%s/needs_full_res", dir) >= 0) {
      wrapper_mkdir_p(dir, 0700);
      FILE *fp = fopen(path, "wb");
      if (fp)
         fclose(fp);
      WRAPPER_LOG(info, "Shader reads 2D sampled image size/texels, %s %s",
                  fp ? "marked" : "could not mark", path);
      free(path);
   }
   return 1;
}
